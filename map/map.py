#!/usr/bin/env python3

"""Subscribe to MLX90640 stats frames and display grayscale images."""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from os import getenv, path
from pathlib import Path

import cv2
import ecal.nanobind_core as ecal_core
from ecal.msg.common.core import ReceiveCallbackData
from ecal.msg.proto.core import Subscriber as ProtoSubscriber
import numpy as np
import zmq
from scipy.spatial.transform import Rotation

GENERATED_DIR = Path(__file__).resolve().parent.parent / "generated"
sys.path.append(str(GENERATED_DIR))

from thermal_pb2 import PprzMsg, ThermalData

PPRZ_HOME = getenv("PAPARAZZI_HOME", "/home/fabien/paparazzi")
pprz_python_path = path.join(PPRZ_HOME, "var/lib/python")
sys.path.append(pprz_python_path)

import pprzlink.message as pprz_message
from pprzlink.ivy import IvyMessagesInterface


DEFAULT_ENDPOINT = "tcp://127.0.0.1:5555"
DEFAULT_ECAL_THERMAL_TOPIC = "mlx90640_data"
DEFAULT_ECAL_PPRZ_TOPIC = "pprz_msg"
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
AREA_XMIN = 10
AREA_XMAX = 45
AREA_YMIN = -50
AREA_YMAX = -10

MIN_TEMPERATURE_C = 30
MAX_TEMPERATURE_C = 60
# meters per pixel
AREA_RESOLUTION = 0.3
MAP_BLEND_REPLACE = "replace"
MAP_BLEND_WEIGHTED = "weighted"
MAP_BLEND_MODE = MAP_BLEND_REPLACE
MAP_BLEND_ALPHA = 0.4
DRONE_MARKER_RADIUS_PX = 6
DRONE_HEADING_LENGTH_M = 2.0
DRONE_HEADING_THICKNESS_PX = 2
PROJECTED_IMAGE_OUTLINE_THICKNESS_PX = 2
TEMPERATURE_LABEL_ORIGIN_PX = (8, 22)
DRONE_POSITION_BOUNDS_LABEL_ORIGIN_PX = (8, 44)
THERMAL_PALETTE_RGB = (
    (0.00, (18, 20, 58)),
    (0.18, (31, 87, 151)),
    (0.38, (46, 161, 171)),
    (0.58, (240, 202, 74)),
    (0.78, (226, 103, 51)),
    (1.00, (250, 241, 220)),
)


class Mapper:
    """Receive thermal frames and Paparazzi telemetry for mapping."""

    def __init__(
        self,
        endpoint: str = DEFAULT_ENDPOINT,
        ivy_bus: str | None = None,
        transport: str = "ecal",
        thermal_topic: str = DEFAULT_ECAL_THERMAL_TOPIC,
        pprz_topic: str = DEFAULT_ECAL_PPRZ_TOPIC,
    ):
        self.endpoint = endpoint
        self.transport = transport
        self.thermal_topic = thermal_topic
        self.pprz_topic = pprz_topic
        self._closed = False
        self._owns_ecal = False
        self._latest_frame: dict | None = None
        self._lock = threading.Lock()
        self._thermal_subscriber = None
        self._pprz_subscriber = None
        self._interface = None
        self.socket = None
        self.last_rotorcraft_fp = None
        self.last_position_m: np.ndarray | None = None
        self.last_attitude_deg: np.ndarray | None = None
        self.drone_position_min_east: float | None = None
        self.drone_position_max_east: float | None = None
        self.drone_position_min_north: float | None = None
        self.drone_position_max_north: float | None = None
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

        if self.transport == "ecal":
            self._init_ecal()
        elif self.transport == "zmq":
            self._init_zmq(endpoint, ivy_bus)
        else:
            raise ValueError(f"unknown transport: {self.transport}")

    def _init_ecal(self) -> None:
        self._owns_ecal = not ecal_core.is_initialized()
        if self._owns_ecal:
            ecal_core.initialize("Thermal Map")

        self._thermal_subscriber = ProtoSubscriber(ThermalData, self.thermal_topic)
        self._thermal_subscriber.set_receive_callback(self._on_thermal_receive)
        self._pprz_subscriber = ProtoSubscriber(PprzMsg, self.pprz_topic)
        self._pprz_subscriber.set_receive_callback(self._on_pprz_receive)

    def _init_zmq(self, endpoint: str, ivy_bus: str | None) -> None:
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
                frame = self.receive_frame()
                if frame is not None:
                    image = self.create_grayscale_image(frame)
                    self.last_image = image
                    self.last_subpage = int(frame["subpage"])
                    self.project_last_image_to_area()
                    # print(frame, flush=True)
                if self.display_images():
                    break
                if self.transport == "ecal" and frame is None:
                    time.sleep(0.01)
        finally:
            self.close()

    def close(self) -> None:
        if self._closed:
            return

        self._closed = True
        if self.socket is not None:
            self.socket.close(0)
        if self._interface is not None:
            print("Shutting down ivy interface...")
            self._interface.shutdown()
        if self._thermal_subscriber is not None:
            self._thermal_subscriber.remove_receive_callback()
        if self._pprz_subscriber is not None:
            self._pprz_subscriber.remove_receive_callback()
        if self._owns_ecal and ecal_core.is_initialized():
            ecal_core.finalize()
        cv2.destroyAllWindows()

    def receive_frame(self) -> dict | None:
        if self.transport == "ecal":
            with self._lock:
                latest = self._latest_frame
                self._latest_frame = None
            return latest

        message = self.socket.recv_string()
        return json.loads(message)

    def _on_thermal_receive(
        self,
        pub_id: ecal_core.TopicId,
        data: ReceiveCallbackData[ThermalData],
    ) -> None:
        del pub_id
        with self._lock:
            self._latest_frame = self._thermal_to_frame(data.message)

    def _thermal_to_frame(self, thermal_data: ThermalData) -> dict:
        return {
            "subpage": thermal_data.subpage,
            "width": thermal_data.cols,
            "height": thermal_data.rows,
            "temperatures": list(thermal_data.data),
        }

    def _on_pprz_receive(
        self,
        pub_id: ecal_core.TopicId,
        data: ReceiveCallbackData[PprzMsg],
    ) -> None:
        del pub_id
        pprz_msg = data.message
        if pprz_msg.msg_name.rsplit(".", 1)[-1] != "ROTORCRAFT_FP":
            return

        try:
            msg = pprz_message.PprzMessage(pprz_msg.msg_class, pprz_msg.msg_id)
            msg.binary_to_payload(bytes(pprz_msg.payload))
        except Exception as exc:
            print(f"Could not decode {pprz_msg.msg_name}: {exc}", file=sys.stderr, flush=True)
            return

        self.on_rotorcraft_fp(pprz_msg.src, msg)

    def on_rotorcraft_fp(self, ac_id, msg) -> None:
        self.last_rotorcraft_fp = msg
        self.last_position_m = self.rotorcraft_position_m(msg)
        self.last_attitude_deg = self.rotorcraft_attitude_deg(msg)
        self.update_drone_position_bounds(self.last_position_m)
        print(self.last_position_m)
        #print(f"ROTORCRAFT_FP from {ac_id}: {msg}", flush=True)

    def update_drone_position_bounds(self, position_m: np.ndarray) -> None:
        east = float(position_m[0])
        north = float(position_m[1])
        if self.drone_position_min_east is None:
            self.drone_position_min_east = east
            self.drone_position_max_east = east
            self.drone_position_min_north = north
            self.drone_position_max_north = north
            return

        self.drone_position_min_east = min(self.drone_position_min_east, east)
        self.drone_position_max_east = max(self.drone_position_max_east, east)
        self.drone_position_min_north = min(self.drone_position_min_north, north)
        self.drone_position_max_north = max(self.drone_position_max_north, north)

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
        return temperatures.astype(np.float32)

    def create_area_image(self) -> np.ndarray:
        if self.area_resolution <= 0:
            raise ValueError("area resolution must be positive")

        width = int(np.ceil((self.area_xmax - self.area_xmin) / self.area_resolution))
        height = int(np.ceil((self.area_ymax - self.area_ymin) / self.area_resolution))
        if width <= 0 or height <= 0:
            raise ValueError("area bounds must produce a positive image size")
        return np.zeros((height, width), dtype=np.float32)

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
            )
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

    def temperature_scale_bounds(self, image: np.ndarray) -> tuple[float, float] | None:
        if MIN_TEMPERATURE_C == 0 and MAX_TEMPERATURE_C == 0:
            bounds = self.temperature_data_bounds(image)
            if bounds is None:
                return None
            low, high = bounds
        else:
            low = float(MIN_TEMPERATURE_C)
            high = float(MAX_TEMPERATURE_C)

        if np.isclose(low, high):
            low -= 1.0
            high += 1.0
        return low, high

    @staticmethod
    def temperature_data_bounds(image: np.ndarray) -> tuple[float, float] | None:
        valid_values = image[image > 0]
        if valid_values.size == 0:
            return None
        return float(np.min(valid_values)), float(np.max(valid_values))

    def scale_temperature_image_for_display(
        self,
        image: np.ndarray,
    ) -> tuple[np.ndarray, tuple[float, float] | None]:
        valid_mask = image > 0
        bounds = self.temperature_scale_bounds(image)
        if bounds is None:
            return np.zeros((*image.shape, 3), dtype=np.uint8), None

        low, high = bounds
        normalized = (image.astype(np.float32) - low) / (high - low)
        normalized = np.clip(normalized, 0.0, 1.0)
        return self.apply_thermal_palette(normalized, valid_mask), bounds

    @staticmethod
    def apply_thermal_palette(normalized: np.ndarray, valid_mask: np.ndarray) -> np.ndarray:
        color_image = np.zeros((*normalized.shape, 3), dtype=np.uint8)
        for index in range(len(THERMAL_PALETTE_RGB) - 1):
            left_t, left_rgb = THERMAL_PALETTE_RGB[index]
            right_t, right_rgb = THERMAL_PALETTE_RGB[index + 1]
            segment_mask = (
                valid_mask
                & (normalized >= left_t)
                & (normalized <= right_t)
            )
            if not np.any(segment_mask):
                continue

            local = (normalized[segment_mask] - left_t) / (right_t - left_t)
            left = np.array(left_rgb, dtype=np.float32)
            right = np.array(right_rgb, dtype=np.float32)
            rgb = left + (right - left) * local[:, None]
            color_image[segment_mask] = rgb[:, ::-1].astype(np.uint8)
        return color_image

    def display_images(self) -> bool:
        if self.last_image is not None:
            self.display_scaled_image(WINDOW_NAME, self.last_image, DISPLAY_SCALE)
        self.display_scaled_image(
            MAP_WINDOW_NAME,
            self.area_image,
            DISPLAY_SCALE,
            draw_drone=True,
            draw_temperature_label=True,
        )
        return cv2.waitKey(1) == ord("q")

    def display_scaled_image(
        self,
        window_name: str,
        image: np.ndarray,
        scale: int,
        draw_drone: bool = False,
        draw_temperature_label: bool = False,
    ) -> None:
        display_image, scale_bounds = self.scale_temperature_image_for_display(image)
        scaled_image = cv2.resize(
            display_image,
            None,
            fx=scale,
            fy=scale,
            interpolation=cv2.INTER_NEAREST,
        )
        if draw_drone:
            self.draw_projected_image_shape(scaled_image, scale)
            self.draw_drone_position(scaled_image, scale)
            self.draw_drone_position_bounds_label(scaled_image)
        if draw_temperature_label:
            data_bounds = self.temperature_data_bounds(image)
            if data_bounds is not None:
                self.draw_temperature_label(scaled_image, data_bounds)
        cv2.imshow(window_name, scaled_image)

    def draw_drone_position_bounds_label(self, image: np.ndarray) -> None:
        if self.drone_position_min_east is None:
            return

        text = (
            f"x [{self.drone_position_min_east:.1f} ... "
            f"{self.drone_position_max_east:.1f}]  "
            f"y [{self.drone_position_min_north:.1f} ... "
            f"{self.drone_position_max_north:.1f}]"
        )
        self.draw_label(image, text, DRONE_POSITION_BOUNDS_LABEL_ORIGIN_PX)

    def draw_temperature_label(
        self,
        image: np.ndarray,
        data_bounds: tuple[float, float],
    ) -> None:
        low, high = data_bounds
        text = f"min {low:.1f} C  max {high:.1f} C"
        self.draw_label(image, text, TEMPERATURE_LABEL_ORIGIN_PX)

    @staticmethod
    def draw_label(
        image: np.ndarray,
        text: str,
        origin: tuple[int, int],
    ) -> None:
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        thickness = 1
        text_size, baseline = cv2.getTextSize(text, font, font_scale, thickness)
        top_left = (origin[0] - 4, origin[1] - text_size[1] - 4)
        bottom_right = (origin[0] + text_size[0] + 4, origin[1] + baseline + 4)
        cv2.rectangle(image, top_left, bottom_right, (0, 0, 0), -1)
        cv2.putText(
            image,
            text,
            origin,
            font,
            font_scale,
            (255, 255, 255),
            thickness,
            cv2.LINE_AA,
        )

    def draw_projected_image_shape(self, image: np.ndarray, scale: int) -> None:
        if (
            self.last_image is None
            or self.last_position_m is None
            or self.last_attitude_deg is None
        ):
            return

        corners = self.projected_image_corners_px()
        if corners is None:
            return
        if not self.projected_bounds_intersect_area(corners):
            return

        scaled_corners = np.round(corners * scale).astype(np.int32)
        cv2.polylines(
            image,
            [scaled_corners],
            isClosed=True,
            color=(0, 0, 0),
            thickness=PROJECTED_IMAGE_OUTLINE_THICKNESS_PX + 2,
            lineType=cv2.LINE_AA,
        )
        cv2.polylines(
            image,
            [scaled_corners],
            isClosed=True,
            color=(51, 103, 226),
            thickness=PROJECTED_IMAGE_OUTLINE_THICKNESS_PX,
            lineType=cv2.LINE_AA,
        )

    def draw_drone_position(self, image: np.ndarray, scale: int) -> None:
        if self.last_position_m is None:
            return

        col, row = self.world_to_area_pixel(
            float(self.last_position_m[0]),
            float(self.last_position_m[1]),
        )
        height, width = self.area_image.shape
        if col < 0.0 or col >= width or row < 0.0 or row >= height:
            return

        center = (int(round(col * scale)), int(round(row * scale)))
        self.draw_drone_heading(image, center, scale)
        cv2.circle(image, center, DRONE_MARKER_RADIUS_PX + 2, (0, 0, 0), -1)
        cv2.circle(image, center, DRONE_MARKER_RADIUS_PX, (255, 255, 255), -1)
        cv2.circle(image, center, 2, (51, 103, 226), -1)

    def draw_drone_heading(
        self,
        image: np.ndarray,
        center: tuple[int, int],
        scale: int,
    ) -> None:
        if self.last_attitude_deg is None:
            return

        heading_rad = np.deg2rad(float(self.last_attitude_deg[2]))
        length_px = DRONE_HEADING_LENGTH_M / self.area_resolution * scale
        end = (
            int(round(center[0] + np.sin(heading_rad) * length_px)),
            int(round(center[1] - np.cos(heading_rad) * length_px)),
        )
        cv2.arrowedLine(
            image,
            center,
            end,
            (0, 0, 0),
            DRONE_HEADING_THICKNESS_PX + 2,
            cv2.LINE_AA,
            0,
            0.25,
        )
        cv2.arrowedLine(
            image,
            center,
            end,
            (51, 103, 226),
            DRONE_HEADING_THICKNESS_PX,
            cv2.LINE_AA,
            0,
            0.25,
        )

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
        return False


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Display mapped grayscale images from MLX90640 frames."
    )
    parser.add_argument(
        "--transport",
        choices=("ecal", "zmq"),
        default="ecal",
        help="Transport to use for receiving frames and telemetry",
    )
    parser.add_argument(
        "--connect",
        default=DEFAULT_ENDPOINT,
        help=f"ZeroMQ PUB endpoint to connect to, default: {DEFAULT_ENDPOINT}",
    )
    parser.add_argument(
        "--thermal-topic",
        default=DEFAULT_ECAL_THERMAL_TOPIC,
        help=f"eCAL ThermalData topic, default: {DEFAULT_ECAL_THERMAL_TOPIC}",
    )
    parser.add_argument(
        "--pprz-topic",
        default=DEFAULT_ECAL_PPRZ_TOPIC,
        help=f"eCAL PprzMsg topic, default: {DEFAULT_ECAL_PPRZ_TOPIC}",
    )
    parser.add_argument(
        "--ivy-bus",
        default=None,
        help="Ivy bus address and port for --transport zmq. Defaults to Paparazzi Ivy.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    with Mapper(
        args.connect,
        args.ivy_bus,
        args.transport,
        args.thermal_topic,
        args.pprz_topic,
    ) as mapper:
        mapper.run()


if __name__ == "__main__":
    main()
