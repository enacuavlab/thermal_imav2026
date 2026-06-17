#!/usr/bin/env python3

"""Subscribe to MLX90640 stats frames and display grayscale images."""

from __future__ import annotations

import argparse
import json
import sys
from os import getenv, path

import cv2
import numpy as np
import zmq
from scipy.spatial.transform import Rotation

PPRZ_HOME = getenv("PAPARAZZI_HOME", "/home/fabien/paparazzi")
pprz_python_path = path.join(PPRZ_HOME, "var/lib/python")
sys.path.append(pprz_python_path)

import pprzlink.message as pprz_message
from pprzlink.ivy import IvyMessagesInterface


DEFAULT_ENDPOINT = "tcp://127.0.0.1:5555"
MIN_TEMPERATURE_C = 10.0
MAX_TEMPERATURE_C = 60.0
MIN_IMAGE_VALUE = 0.0
MAX_IMAGE_VALUE = 255.0
DISPLAY_SCALE = 10
WINDOW_NAME = "MLX90640 grayscale"
MAP_WINDOW_NAME = "Thermal global map"
IVY_APP_NAME = "ThermalMap"
# in roll, pitch, yaw
THERMAL_SENSOR_ROTATION_DEG = np.array([0.0, 0.0, 0.0], dtype=np.float32)
THERMAL_SENSOR_HORIZONTAL_FOV_DEG = 55.0
THERMAL_SENSOR_VERTICAL_FOV_DEG = 35.0
GROUND_ALTITUDE_M = 0.0
ROTORCRAFT_FP_POS_SCALE = 0.0039063
ROTORCRAFT_FP_ATTITUDE_SCALE_DEG = 0.0139882
AREA_XMIN = -8
AREA_XMAX = 8
AREA_YMIN = -8
AREA_YMAX = 8
# meters per pixel
AREA_RESOLUTION = 0.5
MAP_BLEND_REPLACE = "replace"
MAP_BLEND_WEIGHTED = "weighted"
MAP_BLEND_MODE = MAP_BLEND_WEIGHTED
MAP_BLEND_ALPHA = 0.4


class Mapper:
    """Receive thermal frames and Paparazzi telemetry for mapping."""

    def __init__(self, endpoint: str = DEFAULT_ENDPOINT, ivy_bus: str | None = None):
        self.endpoint = endpoint
        self._closed = False
        self.last_rotorcraft_fp = None
        self.last_position_m: np.ndarray | None = None
        self.last_attitude_deg: np.ndarray | None = None
        self.last_image: np.ndarray | None = None
        self.last_subpage: int | None = None
        self.thermal_sensor_rotation_deg = THERMAL_SENSOR_ROTATION_DEG.copy()
        self.area_xmin = AREA_XMIN
        self.area_xmax = AREA_XMAX
        self.area_ymin = AREA_YMIN
        self.area_ymax = AREA_YMAX
        self.area_resolution = AREA_RESOLUTION
        self.area_image = self.create_area_image()
        self.map_blend_mode = MAP_BLEND_MODE
        self.map_blend_alpha = MAP_BLEND_ALPHA

        self.context = zmq.Context.instance()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.connect(endpoint)

        if ivy_bus is None:
            self._interface = IvyMessagesInterface(IVY_APP_NAME)
        else:
            self._interface = IvyMessagesInterface(IVY_APP_NAME, ivy_bus=ivy_bus)
        self._interface.subscribe(
            self.on_rotorcraft_fp,
            pprz_message.PprzMessage("telemetry", "ROTORCRAFT_FP"),
        )

    def run(self) -> None:
        try:
            while True:
                message = self.socket.recv_string()
                frame = json.loads(message)
                image = self.create_grayscale_image(frame)
                self.last_image = image
                self.last_subpage = int(frame["subpage"])
                self.project_last_image_to_area()
                # print(message, flush=True)
                if self.display_images():
                    break
        finally:
            self.close()

    def close(self) -> None:
        if self._closed:
            return

        self._closed = True
        self.socket.close(0)
        print("Shutting down ivy interface...")
        self._interface.shutdown()
        cv2.destroyAllWindows()

    def on_rotorcraft_fp(self, ac_id, msg) -> None:
        self.last_rotorcraft_fp = msg
        self.last_position_m = self.rotorcraft_position_m(msg)
        self.last_attitude_deg = self.rotorcraft_attitude_deg(msg)
        print(f"ROTORCRAFT_FP from {ac_id}: {msg}", flush=True)

    @staticmethod
    def rotorcraft_position_m(msg) -> np.ndarray:
        return np.array(
            [
                float(msg["east"]) * ROTORCRAFT_FP_POS_SCALE,
                float(msg["north"]) * ROTORCRAFT_FP_POS_SCALE,
                float(msg["up"]) * ROTORCRAFT_FP_POS_SCALE,
            ],
            dtype=np.float32,
        )

    @staticmethod
    def rotorcraft_attitude_deg(msg) -> np.ndarray:
        return np.array(
            [
                float(msg["phi"]) * ROTORCRAFT_FP_ATTITUDE_SCALE_DEG,
                float(msg["theta"]) * ROTORCRAFT_FP_ATTITUDE_SCALE_DEG,
                float(msg["psi"]) * ROTORCRAFT_FP_ATTITUDE_SCALE_DEG,
            ],
            dtype=np.float32,
        )

    def create_grayscale_image(self, frame: dict) -> np.ndarray:
        width = int(frame["width"])
        height = int(frame["height"])
        temperatures = np.array(frame["temperatures"], dtype=np.float32).reshape(
            height, width
        )
        temperatures = np.nan_to_num(temperatures, nan=MIN_IMAGE_VALUE)
        temperatures = np.clip(temperatures, MIN_IMAGE_VALUE, MAX_IMAGE_VALUE)
        return temperatures.astype(np.uint8)

    def create_area_image(self) -> np.ndarray:
        if self.area_resolution <= 0:
            raise ValueError("area resolution must be positive")

        width = int(np.ceil((self.area_xmax - self.area_xmin) / self.area_resolution))
        height = int(np.ceil((self.area_ymax - self.area_ymin) / self.area_resolution))
        if width <= 0 or height <= 0:
            raise ValueError("area bounds must produce a positive image size")
        return np.zeros((height, width), dtype=np.uint8)

    def project_last_image_to_area(self) -> bool:
        if (
            self.last_image is None
            or self.last_position_m is None
            or self.last_attitude_deg is None
        ):
            return False

        destination_points = self.projected_image_corners_px()
        if destination_points is None:
            return False
        if not self.projected_bounds_intersect_area(destination_points):
            return False

        height, width = self.last_image.shape
        source_points = np.array(
            [
                [0.0, 0.0],
                [width - 1.0, 0.0],
                [width - 1.0, height - 1.0],
                [0.0, height - 1.0],
            ],
            dtype=np.float32,
        )
        transform = cv2.getPerspectiveTransform(source_points, destination_points)
        area_height, area_width = self.area_image.shape
        warped_image = cv2.warpPerspective(
            self.last_image,
            transform,
            (area_width, area_height),
            flags=cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0,
        )
        source_mask = self.create_subpage_update_mask(height, width)
        warped_mask = cv2.warpPerspective(
            source_mask,
            transform,
            (area_width, area_height),
            flags=cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0,
        )

        valid_pixels = warped_mask > 0
        self.blend_area_image(warped_image, valid_pixels)
        return True

    def blend_area_image(self, new_image: np.ndarray, update_mask: np.ndarray) -> None:
        empty_pixels = update_mask & (self.area_image == 0)
        existing_pixels = update_mask & (self.area_image != 0)

        self.area_image[empty_pixels] = new_image[empty_pixels]
        if self.map_blend_mode == MAP_BLEND_REPLACE:
            self.area_image[existing_pixels] = new_image[existing_pixels]
        elif self.map_blend_mode == MAP_BLEND_WEIGHTED:
            blended = (
                (1.0 - self.map_blend_alpha)
                * self.area_image[existing_pixels].astype(np.float32)
                + self.map_blend_alpha * new_image[existing_pixels].astype(np.float32)
            )
            self.area_image[existing_pixels] = np.clip(
                blended, MIN_IMAGE_VALUE, MAX_IMAGE_VALUE
            ).astype(np.uint8)
        else:
            raise ValueError(f"unknown map blend mode: {self.map_blend_mode}")

    def create_subpage_update_mask(self, height: int, width: int) -> np.ndarray:
        if self.last_subpage is None:
            return np.full((height, width), 255, dtype=np.uint8)

        rows, cols = np.indices((height, width))
        updated_pixels = (rows + cols) % 2 == self.last_subpage % 2
        return (updated_pixels * 255).astype(np.uint8)

    def projected_image_corners_px(self) -> np.ndarray | None:
        ground_points = self.projected_image_corners_m()
        if ground_points is None:
            return None

        destination_points = np.array(
            [self.world_to_area_pixel(point[0], point[1]) for point in ground_points],
            dtype=np.float32,
        )
        if not np.all(np.isfinite(destination_points)):
            return None
        return destination_points

    def projected_image_corners_m(self) -> np.ndarray | None:
        rays_world = self.camera_corner_rays_world()
        ground_points = []
        for ray in rays_world:
            if ray[2] >= 0.0:
                return None

            scale = (GROUND_ALTITUDE_M - self.last_position_m[2]) / ray[2]
            if scale <= 0.0:
                return None
            ground_points.append(self.last_position_m + scale * ray)

        return np.array(ground_points, dtype=np.float32)

    def camera_corner_rays_world(self) -> np.ndarray:
        half_hfov = np.deg2rad(THERMAL_SENSOR_HORIZONTAL_FOV_DEG) / 2.0
        half_vfov = np.deg2rad(THERMAL_SENSOR_VERTICAL_FOV_DEG) / 2.0
        x = np.tan(half_hfov)
        y = np.tan(half_vfov)
        rays_camera = np.array(
            [
                [-x, -y, 1.0],
                [x, -y, 1.0],
                [x, y, 1.0],
                [-x, y, 1.0],
            ],
            dtype=np.float32,
        )
        rays_camera /= np.linalg.norm(rays_camera, axis=1, keepdims=True)

        camera_to_body = np.array(
            [
                [0.0, 1.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float32,
        )
        body_to_world_level = np.array(
            [
                [0.0, 1.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 0.0, -1.0],
            ],
            dtype=np.float32,
        )

        mount_rotation = Rotation.from_euler(
            "xyz", self.thermal_sensor_rotation_deg, degrees=True
        ).as_matrix()
        attitude_rotation = Rotation.from_euler(
            "xyz", self.last_attitude_deg, degrees=True
        ).as_matrix()
        camera_to_world = (
            body_to_world_level
            @ attitude_rotation
            @ mount_rotation
            @ camera_to_body
        )
        return (camera_to_world @ rays_camera.T).T

    def world_to_area_pixel(self, east: float, north: float) -> tuple[float, float]:
        col = (east - self.area_xmin) / self.area_resolution
        row = (self.area_ymax - north) / self.area_resolution
        return col, row

    def projected_bounds_intersect_area(self, points: np.ndarray) -> bool:
        height, width = self.area_image.shape
        min_col = float(np.min(points[:, 0]))
        max_col = float(np.max(points[:, 0]))
        min_row = float(np.min(points[:, 1]))
        max_row = float(np.max(points[:, 1]))
        return (
            min_col < width
            and max_col >= 0.0
            and min_row < height
            and max_row >= 0.0
        )

    def scale_temperature_image_for_display(self, image: np.ndarray) -> np.ndarray:
        span = MAX_TEMPERATURE_C - MIN_TEMPERATURE_C
        normalized = (image.astype(np.float32) - MIN_TEMPERATURE_C) / span
        normalized = np.clip(normalized, 0.0, 1.0)
        return (normalized * MAX_IMAGE_VALUE).astype(np.uint8)

    def display_images(self) -> bool:
        if self.last_image is not None:
            self.display_scaled_image(WINDOW_NAME, self.last_image, DISPLAY_SCALE)
        self.display_scaled_image(MAP_WINDOW_NAME, self.area_image, DISPLAY_SCALE)
        return cv2.waitKey(1) == ord("q")

    def display_scaled_image(self, window_name: str, image: np.ndarray, scale: int) -> None:
        display_image = self.scale_temperature_image_for_display(image)
        scaled_image = cv2.resize(
            display_image,
            None,
            fx=scale,
            fy=scale,
            interpolation=cv2.INTER_NEAREST,
        )
        cv2.imshow(window_name, scaled_image)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
        return False


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Display grayscale images from mlx90640_stats ZeroMQ messages."
    )
    parser.add_argument(
        "--connect",
        default=DEFAULT_ENDPOINT,
        help=f"ZeroMQ PUB endpoint to connect to, default: {DEFAULT_ENDPOINT}",
    )
    parser.add_argument(
        "--ivy-bus",
        default=None,
        help="Ivy bus address and port. Defaults to the Paparazzi Ivy setting.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    with Mapper(args.connect, args.ivy_bus) as mapper:
        mapper.run()


if __name__ == "__main__":
    main()
