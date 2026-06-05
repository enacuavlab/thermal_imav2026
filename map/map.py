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
IVY_APP_NAME = "ThermalMap"
# in roll, pitch, yaw
THERMAL_SENSOR_ROTATION_DEG = np.array([0.0, 0.0, 0.0], dtype=np.float32)
AREA_XMIN = 0
AREA_XMAX = 1000
AREA_YMIN = 0
AREA_YMAX = 1000
# meters per pixel
AREA_RESOLUTION = 1.0


class Mapper:
    """Receive thermal frames and Paparazzi telemetry for mapping."""

    def __init__(self, endpoint: str = DEFAULT_ENDPOINT, ivy_bus: str | None = None):
        self.endpoint = endpoint
        self._closed = False
        self.last_rotorcraft_fp = None
        self.last_image: np.ndarray | None = None
        self.thermal_sensor_rotation_deg = THERMAL_SENSOR_ROTATION_DEG.copy()
        self.area_xmin = AREA_XMIN
        self.area_xmax = AREA_XMAX
        self.area_ymin = AREA_YMIN
        self.area_ymax = AREA_YMAX
        self.area_resolution = AREA_RESOLUTION
        self.area_image = self.create_area_image()

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
                # print(message, flush=True)
                if self.display_image(image):
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
        print(f"ROTORCRAFT_FP from {ac_id}: {msg}", flush=True)

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

    def display_image(self, image: np.ndarray) -> bool:
        span = MAX_TEMPERATURE_C - MIN_TEMPERATURE_C
        normalized = (image.astype(np.float32) - MIN_TEMPERATURE_C) / span
        normalized = np.clip(normalized, 0.0, 1.0)
        display_image = (normalized * MAX_IMAGE_VALUE).astype(np.uint8)
        scaled_image = cv2.resize(
            display_image,
            None,
            fx=DISPLAY_SCALE,
            fy=DISPLAY_SCALE,
            interpolation=cv2.INTER_NEAREST,
        )
        cv2.imshow(WINDOW_NAME, scaled_image)
        return cv2.waitKey(1) == ord("q")

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
