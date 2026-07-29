"""Shared helpers for MLX90640 thermal tools."""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation


ROTORCRAFT_FP_POS_SCALE = 0.0039063
ROTORCRAFT_FP_ATTITUDE_SCALE_DEG = 0.0139882
THERMAL_SENSOR_ROTATION_DEG = np.array([0.0, 0.0, -90.0], dtype=np.float32)
THERMAL_SENSOR_HORIZONTAL_FOV_DEG = 55.0
THERMAL_SENSOR_VERTICAL_FOV_DEG = 35.0
QueuedPprzMsg = tuple[int, int, int, int, str, bytes]


def thermal_data_to_frame(thermal_data) -> dict:
    return {
        "subpage": thermal_data.subpage,
        "width": thermal_data.cols,
        "height": thermal_data.rows,
        "temperatures": list(thermal_data.data),
    }


def pprz_message_name(msg_name: str) -> str:
    return msg_name.rsplit(".", 1)[-1]


def pprz_proto_to_queued(pprz_msg) -> QueuedPprzMsg:
    return (
        int(pprz_msg.src),
        int(pprz_msg.dst),
        int(pprz_msg.msg_class),
        int(pprz_msg.msg_id),
        str(pprz_msg.msg_name),
        bytes(pprz_msg.payload),
    )


def rotorcraft_position_m(msg) -> np.ndarray:
    return np.array(
        [
            float(msg["east"]) * ROTORCRAFT_FP_POS_SCALE,
            float(msg["north"]) * ROTORCRAFT_FP_POS_SCALE,
            float(msg["up"]) * ROTORCRAFT_FP_POS_SCALE,
        ],
        dtype=np.float32,
    )


def rotorcraft_attitude_deg(msg) -> np.ndarray:
    return np.array(
        [
            float(msg["phi"]) * ROTORCRAFT_FP_ATTITUDE_SCALE_DEG,
            float(msg["theta"]) * ROTORCRAFT_FP_ATTITUDE_SCALE_DEG,
            float(msg["psi"]) * ROTORCRAFT_FP_ATTITUDE_SCALE_DEG,
        ],
        dtype=np.float32,
    )


class ThermalCameraModel:
    def __init__(
        self,
        sensor_rotation_deg: np.ndarray = THERMAL_SENSOR_ROTATION_DEG,
        horizontal_fov_deg: float = THERMAL_SENSOR_HORIZONTAL_FOV_DEG,
        vertical_fov_deg: float = THERMAL_SENSOR_VERTICAL_FOV_DEG,
    ):
        self.sensor_rotation_deg = sensor_rotation_deg
        self.horizontal_fov_deg = horizontal_fov_deg
        self.vertical_fov_deg = vertical_fov_deg

    def pixel_ray_world(
        self,
        pixel_x: int,
        pixel_y: int,
        width: int,
        height: int,
        attitude_deg: np.ndarray,
    ) -> np.ndarray:
        x_extent, y_extent = self._image_extents()
        x_norm = 0.0 if width <= 1 else (2.0 * pixel_x / (width - 1.0)) - 1.0
        y_norm = 0.0 if height <= 1 else 1.0 - (2.0 * pixel_y / (height - 1.0))
        ray_camera = np.array(
            [x_norm * x_extent, y_norm * y_extent, 1.0],
            dtype=np.float32,
        )
        ray_camera /= np.linalg.norm(ray_camera)
        return self._camera_to_world(attitude_deg) @ ray_camera

    def corner_rays_world(self, attitude_deg: np.ndarray) -> np.ndarray:
        x_extent, y_extent = self._image_extents()
        rays_camera = np.array(
            [
                [-x_extent, y_extent, 1.0],
                [x_extent, y_extent, 1.0],
                [x_extent, -y_extent, 1.0],
                [-x_extent, -y_extent, 1.0],
            ],
            dtype=np.float32,
        )
        rays_camera /= np.linalg.norm(rays_camera, axis=1, keepdims=True)
        return (self._camera_to_world(attitude_deg) @ rays_camera.T).T

    def _image_extents(self) -> tuple[float, float]:
        half_hfov = np.deg2rad(self.horizontal_fov_deg) / 2.0
        half_vfov = np.deg2rad(self.vertical_fov_deg) / 2.0
        return float(np.tan(half_hfov)), float(np.tan(half_vfov))

    def _camera_to_world(self, attitude_deg: np.ndarray) -> np.ndarray:
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
            "xyz",
            self.sensor_rotation_deg,
            degrees=True,
        ).as_matrix()
        attitude_rotation = Rotation.from_euler(
            "xyz",
            attitude_deg,
            degrees=True,
        ).as_matrix()
        return body_to_world_level @ attitude_rotation @ mount_rotation @ camera_to_body


def project_ray_to_ground(
    position_m: np.ndarray,
    ray_world: np.ndarray,
    ground_altitude_m: float,
) -> np.ndarray | None:
    if ray_world[2] >= 0.0:
        return None

    scale = (ground_altitude_m - position_m[2]) / ray_world[2]
    if scale <= 0.0:
        return None
    return position_m + scale * ray_world


def project_pixel_to_ground(
    camera_model: ThermalCameraModel,
    position_m: np.ndarray,
    attitude_deg: np.ndarray,
    pixel_x: int,
    pixel_y: int,
    width: int,
    height: int,
    ground_altitude_m: float,
) -> tuple[float, float, float] | None:
    ray = camera_model.pixel_ray_world(pixel_x, pixel_y, width, height, attitude_deg)
    point = project_ray_to_ground(position_m, ray, ground_altitude_m)
    if point is None:
        return None
    return float(point[0]), float(point[1]), float(point[2])


def projected_image_corner_offsets_m(
    camera_model: ThermalCameraModel,
    position_m: np.ndarray,
    attitude_deg: np.ndarray,
    ground_altitude_m: float,
) -> np.ndarray | None:
    ground_offsets = []
    for ray in camera_model.corner_rays_world(attitude_deg):
        point = project_ray_to_ground(position_m, ray, ground_altitude_m)
        if point is None:
            return None
        ground_offsets.append(point - position_m)

    return np.array(ground_offsets, dtype=np.float32)
