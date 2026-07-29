#!/usr/bin/env python3

"""Detect thermal hot spots from MLX90640 eCAL frames."""

from __future__ import annotations

import argparse
import faulthandler
import math
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import ecal.nanobind_core as ecal_core
from ecal.msg.common.core import ReceiveCallbackData
from ecal.msg.proto.core import Publisher as ProtoPublisher
from ecal.msg.proto.core import Subscriber as ProtoSubscriber
import numpy as np
from pyproj import Transformer


GENERATED_DIR = Path(__file__).resolve().parent.parent / "generated"
DEFAULT_PPRZLINK_PYTHON = (
    "/home/fabien/paparazzi/sw/ext/pprzlink/lib/v2.0/python/src"
)

sys.path.insert(0, str(Path(DEFAULT_PPRZLINK_PYTHON).expanduser()))
sys.path.append(str(GENERATED_DIR))

from pprzlink.message import PprzMessage
from thermal_pb2 import PprzMsg, ThermalData
from thermal_common import (
    QueuedPprzMsg,
    ThermalCameraModel,
    pprz_message_name,
    pprz_proto_to_queued,
    project_pixel_to_ground,
    rotorcraft_attitude_deg,
    rotorcraft_position_m,
    thermal_data_to_frame,
)


DEFAULT_ECAL_THERMAL_TOPIC = "mlx90640_data"
DEFAULT_ECAL_PPRZ_TOPIC = "pprz_msg"
ECAL_PPRZ_TX_TOPIC = "pprz_msg_tx"
DEFAULT_THRESHOLD_C = 40.0
DEFAULT_MIN_REPORT_INTERVAL_S = 0.5
DEFAULT_GROUND_ALTITUDE_M = 0.0
DEFAULT_CLUSTER_DISTANCE_M = 3.0
SPOT_KINDS = ("hot", "cold")
PPRZ_ALERT_CLASS = "telemetry"
PPRZ_ALERT_MESSAGE = "PAYLOAD_FLOAT"
PPRZ_ALERT_PAYLOAD = "cluster_key,east,north,up,latitude,longitude,median_temperature,member_count"
INS_REF_LATLON_SCALE_DEG = 1e-7
INS_REF_ALT_SCALE = 0.001
GPS_LATLON_SCALE_DEG = 1e-7
GPS_ALT_SCALE_M = 0.001


@dataclass(frozen=True)
class HotSpot:
    sequence: int
    timestamp: float
    temperature: float
    pixel_x: int
    pixel_y: int
    east: float
    north: float
    up: float
    latitude: float = math.nan
    longitude: float = math.nan
    hmsl: float = math.nan


@dataclass(frozen=True)
class SpotCluster:
    sequence: int
    member_count: int
    spot_sequences: tuple[int, ...]
    temperature: float
    east: float
    north: float
    up: float
    latitude: float = math.nan
    longitude: float = math.nan
    hmsl: float = math.nan


class ThermalSpotter:
    """Subscribe to aircraft state and thermal frames, then publish hot spots."""

    def __init__(
        self,
        threshold_c: float,
        min_report_interval_s: float,
        ground_altitude_m: float,
        spot_kind: str = "hot",
        cluster_distance_m: float = DEFAULT_CLUSTER_DISTANCE_M,
        thermal_topic: str = DEFAULT_ECAL_THERMAL_TOPIC,
        pprz_topic: str = DEFAULT_ECAL_PPRZ_TOPIC,
        thermal_map=None,
    ):
        self.threshold_c = threshold_c
        self.min_report_interval_s = min_report_interval_s
        self.ground_altitude_m = ground_altitude_m
        if spot_kind not in SPOT_KINDS:
            raise ValueError(f"unknown spot kind: {spot_kind}")
        if cluster_distance_m < 0.0:
            raise ValueError("cluster distance must be non-negative")
        self.spot_kind = spot_kind
        self.cluster_distance_m = cluster_distance_m
        self.thermal_map = thermal_map
        self._closed = False
        self._owns_ecal = False
        self._latest_frame: dict | None = None
        self._pending_pprz: deque[QueuedPprzMsg] = deque()
        self._lock = threading.Lock()
        self._sequence = 0
        self._last_report_time = 0.0
        self.hot_spots: list[HotSpot] = []
        self.spot_clusters: list[SpotCluster] = []
        self.last_position_m: np.ndarray | None = None
        self.last_attitude_deg: np.ndarray | None = None
        self.ltp_origin_hmsl_m: float | None = None
        self.ltp_to_wgs84: Transformer | None = None
        self.camera_model = ThermalCameraModel()

        self._owns_ecal = not ecal_core.is_initialized()
        if self._owns_ecal:
            ecal_core.initialize("Thermal Spotter")

        self._thermal_subscriber = ProtoSubscriber(ThermalData, thermal_topic)
        self._thermal_subscriber.set_receive_callback(self._on_thermal_receive)
        self._pprz_subscriber = ProtoSubscriber(PprzMsg, pprz_topic)
        self._pprz_subscriber.set_receive_callback(self._on_pprz_receive)
        self._pprz_alert_publisher = ProtoPublisher(PprzMsg, ECAL_PPRZ_TX_TOPIC)

    def close(self) -> None:
        if self._closed:
            return

        self._closed = True
        self._thermal_subscriber.remove_receive_callback()
        self._pprz_subscriber.remove_receive_callback()
        if self.thermal_map is not None:
            self.thermal_map.close()
        if self._owns_ecal and ecal_core.is_initialized():
            ecal_core.finalize()

    def run(self) -> None:
        try:
            while True:
                self.process_pending_pprz_messages()
                frame = self.receive_frame()
                if frame is None:
                    if self.thermal_map is not None and self.thermal_map.display_images():
                        break
                    time.sleep(0.01)
                    continue
                if self.thermal_map is not None:
                    self.thermal_map.process_frame(frame)
                    if self.thermal_map.display_images():
                        break
                self.process_frame(frame)
        finally:
            self.close()

    def receive_frame(self) -> dict | None:
        with self._lock:
            latest = self._latest_frame
            self._latest_frame = None
        return latest

    def receive_pprz_messages(self) -> list[QueuedPprzMsg]:
        with self._lock:
            messages = list(self._pending_pprz)
            self._pending_pprz.clear()
        return messages

    def _on_thermal_receive(
        self,
        pub_id: ecal_core.TopicId,
        data: ReceiveCallbackData[ThermalData],
    ) -> None:
        del pub_id
        with self._lock:
            self._latest_frame = thermal_data_to_frame(data.message)

    def _on_pprz_receive(self, pub_id: ecal_core.TopicId, data: ReceiveCallbackData[PprzMsg]) -> None:
        del pub_id
        with self._lock:
            self._pending_pprz.append(pprz_proto_to_queued(data.message))

    def process_pending_pprz_messages(self) -> None:
        for pprz_msg in self.receive_pprz_messages():
            self.process_pprz_message(pprz_msg)

    def process_pprz_message(self, pprz_msg: QueuedPprzMsg) -> None:
        _, _, msg_class, msg_id, msg_name, payload = pprz_msg
        message_name = pprz_message_name(msg_name)
        if message_name not in {"ROTORCRAFT_FP", "INS_REF", "GPS_INT", "PAYLOAD_COMMAND"}:
            return

        try:
            msg = PprzMessage(msg_class, msg_id)
            msg.binary_to_payload(payload)
        except Exception as exc:
            print(f"Could not decode {msg_name}: {exc}", file=sys.stderr, flush=True)
            return

        if message_name == "ROTORCRAFT_FP":
            self.last_position_m = rotorcraft_position_m(msg)
            self.last_attitude_deg = rotorcraft_attitude_deg(msg)
            if self.thermal_map is not None:
                self.thermal_map.update_aircraft_state(
                    self.last_position_m,
                    self.last_attitude_deg,
                )
        elif message_name == "INS_REF":
            self.on_ins_ref(msg)
        elif message_name == "GPS_INT" and self.thermal_map is not None:
            self.thermal_map.update_gps(
                float(msg["lat"]) * GPS_LATLON_SCALE_DEG,
                float(msg["lon"]) * GPS_LATLON_SCALE_DEG,
                float(msg["hmsl"]) * GPS_ALT_SCALE_M,
            )
        elif message_name == "PAYLOAD_COMMAND":
            self.on_payload_command(msg)

    def on_payload_command(self, msg: PprzMessage) -> None:
        command = msg["command"]
        if len(command) < 2 or int(command[0]) != ord("t"):
            return

        self.threshold_c = float(command[1])
        print(f"Threshold set to {self.threshold_c:.1f} C", flush=True)

    def process_frame(self, frame: dict) -> HotSpot | None:
        spot_pixel = self.detect_spot_pixel(frame, self.spot_kind)
        if spot_pixel is None:
            return None

        temperature, pixel_x, pixel_y = spot_pixel
        if self.spot_kind == "hot" and temperature < self.threshold_c:
            return None
        if self.spot_kind == "cold" and temperature > self.threshold_c:
            return None
        if time.monotonic() - self._last_report_time < self.min_report_interval_s:
            return None

        ground_point = self.project_pixel_to_ground(
            pixel_x,
            pixel_y,
            int(frame["width"]),
            int(frame["height"]),
        )
        if ground_point is None:
            print(
                f"{self.spot_kind} spot past threshold, but aircraft pose cannot project it to ground",
                flush=True,
            )
            return None

        self._last_report_time = time.monotonic()
        self._sequence += 1
        east, north, up = ground_point
        lat, lon = self.ground_point_to_gps(east, north)
        hot_spot = HotSpot(
            sequence=self._sequence,
            timestamp=time.time(),
            temperature=temperature,
            pixel_x=pixel_x,
            pixel_y=pixel_y,
            east=east,
            north=north,
            up=up,
            latitude=lat,
            longitude=lon,
            hmsl=self.ground_hmsl(),
        )
        spot_cluster = self.record_hot_spot(hot_spot)
        if self.thermal_map is not None:
            self.thermal_map.mark_hot_spot(hot_spot)
            self.thermal_map.update_spot_clusters(self.spot_clusters)
        self.publish_spot_cluster(spot_cluster)
        print(
            f"{self.spot_kind} spot #{hot_spot.sequence}: {hot_spot.temperature:.1f} C "
            f"pixel=({hot_spot.pixel_x},{hot_spot.pixel_y}) "
            f"ground=({hot_spot.east:.2f}, {hot_spot.north:.2f}, {hot_spot.up:.2f})",
            flush=True,
        )
        return hot_spot

    @staticmethod
    def detect_hot_pixel(frame: dict) -> tuple[float, int, int] | None:
        return ThermalSpotter.detect_spot_pixel(frame, "hot")

    @staticmethod
    def detect_spot_pixel(
        frame: dict,
        spot_kind: str,
    ) -> tuple[float, int, int] | None:
        width = int(frame["width"])
        height = int(frame["height"])
        temperatures = np.array(frame["temperatures"], dtype=np.float32)
        if width <= 0 or height <= 0 or temperatures.size != width * height:
            return None

        image = temperatures.reshape(height, width)[:, ::-1]
        finite_mask = np.isfinite(image)
        if not np.any(finite_mask):
            return None

        if spot_kind == "hot":
            finite_image = np.where(finite_mask, image, -np.inf)
            flat_index = int(np.argmax(finite_image))
        elif spot_kind == "cold":
            finite_image = np.where(finite_mask, image, np.inf)
            flat_index = int(np.argmin(finite_image))
        else:
            raise ValueError(f"unknown spot kind: {spot_kind}")
        pixel_y, pixel_x = np.unravel_index(flat_index, finite_image.shape)
        return float(finite_image[pixel_y, pixel_x]), int(pixel_x), int(pixel_y)

    def project_pixel_to_ground(
        self,
        pixel_x: int,
        pixel_y: int,
        width: int,
        height: int,
    ) -> tuple[float, float, float] | None:
        if self.last_position_m is None or self.last_attitude_deg is None:
            return None

        return project_pixel_to_ground(
            self.camera_model,
            self.last_position_m,
            self.last_attitude_deg,
            pixel_x,
            pixel_y,
            width,
            height,
            self.ground_altitude_m,
        )

    def ground_point_to_gps(self, east: float, north: float) -> tuple[float, float]:
        if self.ltp_to_wgs84 is None:
            return math.nan, math.nan

        lon, lat, _ = self.ltp_to_wgs84.transform(east, north, self.ground_altitude_m)
        return lat, lon

    def ground_hmsl(self) -> float:
        if self.ltp_origin_hmsl_m is None:
            return math.nan
        return self.ltp_origin_hmsl_m + self.ground_altitude_m

    def on_ins_ref(self, msg: PprzMessage) -> None:
        lat0_deg = float(msg["lat0"]) * INS_REF_LATLON_SCALE_DEG
        lon0_deg = float(msg["lon0"]) * INS_REF_LATLON_SCALE_DEG
        alt0_m = float(msg["alt0"]) * INS_REF_ALT_SCALE
        self.ltp_origin_hmsl_m = float(msg["hmsl0"]) * INS_REF_ALT_SCALE
        self.ltp_to_wgs84 = Transformer.from_pipeline(
            "+proj=pipeline "
            f"+step +inv +proj=topocentric +ellps=WGS84 +lat_0={lat0_deg} "
            f"+lon_0={lon0_deg} +h_0={alt0_m} "
            "+step +inv +proj=cart +ellps=WGS84"
        )

    def record_hot_spot(self, hot_spot: HotSpot) -> SpotCluster:
        self.hot_spots.append(hot_spot)
        self.hot_spots.sort(
            key=lambda spot: spot.temperature,
            reverse=self.spot_kind == "hot",
        )
        self.spot_clusters = self.cluster_spots(
            self.hot_spots,
            self.cluster_distance_m,
        )
        return next(
            cluster
            for cluster in self.spot_clusters
            if hot_spot.sequence in cluster.spot_sequences
        )

    @staticmethod
    def cluster_spots(
        spots: list[HotSpot],
        cluster_distance_m: float,
    ) -> list[SpotCluster]:
        """Group spots connected by horizontal distances within the threshold."""
        if not spots:
            return []

        positions = np.array(
            [(spot.east, spot.north) for spot in spots],
            dtype=np.float64,
        )
        squared_distances = np.sum(
            (positions[:, np.newaxis, :] - positions[np.newaxis, :, :]) ** 2,
            axis=2,
        )
        neighbors = squared_distances <= cluster_distance_m**2
        visited = np.zeros(len(spots), dtype=bool)
        clusters: list[SpotCluster] = []

        for first_index in range(len(spots)):
            if visited[first_index]:
                continue

            member_indices: list[int] = []
            pending = [first_index]
            visited[first_index] = True
            while pending:
                index = pending.pop()
                member_indices.append(index)
                for neighbor_index in np.flatnonzero(neighbors[index]):
                    if not visited[neighbor_index]:
                        visited[neighbor_index] = True
                        pending.append(int(neighbor_index))

            members = [spots[index] for index in member_indices]
            clusters.append(ThermalSpotter.make_spot_cluster(members))

        return sorted(clusters, key=lambda cluster: cluster.sequence)

    @staticmethod
    def make_spot_cluster(members: list[HotSpot]) -> SpotCluster:
        def average(attribute: str) -> float:
            values = [getattr(spot, attribute) for spot in members]
            finite_values = [value for value in values if math.isfinite(value)]
            return math.nan if not finite_values else float(np.mean(finite_values))

        temperatures = [spot.temperature for spot in members]

        return SpotCluster(
            sequence=min(spot.sequence for spot in members),
            member_count=len(members),
            spot_sequences=tuple(sorted(spot.sequence for spot in members)),
            temperature=float(np.median(temperatures)),
            east=average("east"),
            north=average("north"),
            up=average("up"),
            latitude=average("latitude"),
            longitude=average("longitude"),
            hmsl=average("hmsl"),
        )

    def publish_spot_cluster(self, spot_cluster: SpotCluster) -> None:
        self._pprz_alert_publisher.send(self.spot_cluster_to_pprz_msg(spot_cluster))

    def spot_cluster_to_pprz_msg(self, spot_cluster: SpotCluster) -> PprzMsg:
        # Alert schema for telemetry.PAYLOAD_FLOAT:
        # cluster_key, east, north, up, latitude, longitude,
        # median_temperature, member_count
        pprz_message = PprzMessage(PPRZ_ALERT_CLASS, PPRZ_ALERT_MESSAGE)
        pprz_message["values"] = [
            float(spot_cluster.sequence),
            spot_cluster.east,
            spot_cluster.north,
            spot_cluster.up,
            spot_cluster.latitude,
            spot_cluster.longitude,
            spot_cluster.temperature,
            float(spot_cluster.member_count),
        ]
        return PprzMsg(
            src=0,
            dst=0xFF,
            msg_class=pprz_message.class_id,
            msg_id=pprz_message.msg_id,
            msg_name=f"{pprz_message.msg_class}.{pprz_message.name}",
            payload=pprz_message.payload_to_binary(),
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Detect and publish MLX90640 hot spots from eCAL topics.")
    parser.add_argument("--threshold", type=float, default=DEFAULT_THRESHOLD_C, help=f"spot threshold in deg C, default: {DEFAULT_THRESHOLD_C}")
    parser.add_argument("--spot-kind", choices=SPOT_KINDS, default="hot", help="detect temperatures at or above (hot), or at or below (cold), the threshold")
    parser.add_argument("--cluster-distance", type=float, default=DEFAULT_CLUSTER_DISTANCE_M, help="maximum horizontal distance in meters between clustered spots, default: %(default)s")
    parser.add_argument("--min-report-interval", type=float, default=DEFAULT_MIN_REPORT_INTERVAL_S, help="minimum seconds between reports, default: %(default)s")
    parser.add_argument("--ground-altitude", type=float, default=DEFAULT_GROUND_ALTITUDE_M, help=f"ground altitude in local ENU meters, default: {DEFAULT_GROUND_ALTITUDE_M}")
    parser.add_argument("--thermal-topic", default=DEFAULT_ECAL_THERMAL_TOPIC, help=f"eCAL ThermalData topic, default: {DEFAULT_ECAL_THERMAL_TOPIC}")
    parser.add_argument("--pprz-topic", default=DEFAULT_ECAL_PPRZ_TOPIC, help=f"eCAL PprzMsg topic, default: {DEFAULT_ECAL_PPRZ_TOPIC}")
    parser.add_argument("--map", action="store_true", help="show the optional thermal map")
    parser.add_argument("--geotiff", type=Path, help="georeferenced satellite or orthophoto background for --map")
    parser.add_argument("--heat-alpha", type=float, default=0.55, help="thermal overlay alpha over a GeoTIFF, default: %(default)s")
    return parser.parse_args()


def main(enable_map: bool = False) -> None:
    faulthandler.enable()
    args = parse_args()

    thermal_map = None
    if args.map or enable_map:
        from map import ThermalMap

        try:
            thermal_map = ThermalMap(
                ground_altitude_m=args.ground_altitude,
                geotiff_path=args.geotiff,
                heat_alpha=args.heat_alpha,
            )
        except RuntimeError as exc:
            print(exc, file=sys.stderr)
            raise SystemExit(1) from exc

    print(f"Publishing PPRZ alerts as {PPRZ_ALERT_CLASS}.{PPRZ_ALERT_MESSAGE} values=[{PPRZ_ALERT_PAYLOAD}]", flush=True)
    spotter = ThermalSpotter(
        threshold_c=args.threshold,
        min_report_interval_s=args.min_report_interval,
        ground_altitude_m=args.ground_altitude,
        spot_kind=args.spot_kind,
        cluster_distance_m=args.cluster_distance,
        thermal_topic=args.thermal_topic,
        pprz_topic=args.pprz_topic,
        thermal_map=thermal_map,
    )
    try:
        spotter.run()
    except KeyboardInterrupt:
        print("\nStopped", file=sys.stderr)


if __name__ == "__main__":
    main()
