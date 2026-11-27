#!/usr/bin/env python3

"""PyQt5 viewer for MLX90640 frame messages."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import threading
import time
from typing import Protocol

from PyQt5 import QtCore, QtGui, QtWidgets
import zmq

GENERATED_DIR = Path(__file__).resolve().parent.parent / "generated"
sys.path.append(str(GENERATED_DIR))

try:
    import ecal.nanobind_core as ecal_core
    from ecal.msg.common.core import ReceiveCallbackData
    from ecal.msg.proto.core import Subscriber as ProtoSubscriber
except ImportError as error:
    ecal_core = None
    print(f"eCAL unavailable ({error}); using ZeroMQ only.", file=sys.stderr)
else:
    try:
        from thermal_pb2 import ThermalData
    except ImportError as error:
        ecal_core = None
        print(
            f"thermal_pb2 unavailable ({error}); using ZeroMQ only. "
            "Generate the Python protobuf module by running "
            "'cd proto && bash generate_proto.sh' from the project root.",
            file=sys.stderr,
        )


DEFAULT_ENDPOINT = "tcp://127.0.0.1:5555"
DEFAULT_ECAL_TOPIC = "mlx90640_data"
DEFAULT_WIDTH = 32
DEFAULT_HEIGHT = 24


class FrameSubscriber(Protocol):
    label: str

    def close(self) -> None:
        ...

    def receive_latest(self) -> dict | None:
        ...


class ZmqFrameSubscriber:
    def __init__(self, endpoint: str):
        self.label = f"ZMQ SUB {endpoint}"
        self.context = zmq.Context.instance()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.connect(endpoint)

    def close(self) -> None:
        self.socket.close(0)

    def receive_latest(self) -> dict | None:
        latest = None
        while True:
            try:
                latest = json.loads(self.socket.recv_string(flags=zmq.NOBLOCK))
            except zmq.Again:
                return latest


class EcalFrameSubscriber:
    def __init__(self, topic: str):
        self.label = f"eCAL SUB {topic}"
        self.frame = 0
        self._lock = threading.Lock()
        self._latest_message: dict | None = None
        self._closed = False
        self._owns_ecal = not ecal_core.is_initialized()
        if self._owns_ecal:
            ecal_core.initialize("MLX90640 Frame Viewer")
        self.subscriber = ProtoSubscriber(ThermalData, topic)
        self.subscriber.set_receive_callback(self._on_receive)

    def close(self) -> None:
        if self._closed:
            return

        self._closed = True
        self.subscriber.remove_receive_callback()
        if self._owns_ecal and ecal_core.is_initialized():
            ecal_core.finalize()

    def receive_latest(self) -> dict | None:
        with self._lock:
            latest = self._latest_message
            self._latest_message = None
        return latest

    def _on_receive(self, pub_id: ecal_core.TopicId, data: ReceiveCallbackData[ThermalData]) -> None:
        del pub_id
        with self._lock:
            self._latest_message = self._to_message(data.message)

    def _to_message(self, thermal_data) -> dict:
        temperatures = list(thermal_data.data)
        finite = [value for value in temperatures if math.isfinite(value)]
        self.frame += 1

        if finite:
            minimum = min(finite)
            maximum = max(finite)
            average = sum(finite) / len(finite)
        else:
            minimum = math.nan
            maximum = math.nan
            average = math.nan

        return {
            "frame": self.frame,
            "subpage": thermal_data.subpage,
            "min": minimum,
            "avg": average,
            "max": maximum,
            "count": len(finite),
            "width": thermal_data.cols or DEFAULT_WIDTH,
            "height": thermal_data.rows or DEFAULT_HEIGHT,
            "temperatures": temperatures,
        }


class StatTile(QtWidgets.QFrame):
    def __init__(self, title: str, suffix: str = ""):
        super().__init__()
        self.suffix = suffix
        self.setObjectName("statTile")

        self.title_label = QtWidgets.QLabel(title)
        self.title_label.setObjectName("tileTitle")
        self.value_label = QtWidgets.QLabel("--")
        self.value_label.setObjectName("tileValue")

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(12, 8, 12, 10)
        layout.setSpacing(2)
        layout.addWidget(self.title_label)
        layout.addWidget(self.value_label)

    def set_value(self, value) -> None:
        if isinstance(value, float):
            text = f"{value:.2f}{self.suffix}" if math.isfinite(value) else "--"
        elif value is None:
            text = "--"
        else:
            text = f"{value}{self.suffix}"
        self.value_label.setText(text)


class ThermalFrame(QtWidgets.QWidget):
    def __init__(self, min_temp: float | None = None, max_temp: float | None = None):
        super().__init__()
        self.min_temp = min_temp
        self.max_temp = max_temp
        self.width_px = DEFAULT_WIDTH
        self.height_px = DEFAULT_HEIGHT
        self.values: list[float | None] = []
        self.minimum = math.nan
        self.maximum = math.nan
        self.setMinimumSize(560, 420)

    def set_frame(self, width: int, height: int, values: list) -> None:
        if width <= 0 or height <= 0 or len(values) != width * height:
            return
        self.width_px = width
        self.height_px = height
        self.values = [float(value) if value is not None else None for value in values]
        finite = [value for value in self.values if value is not None and math.isfinite(value)]
        if finite:
            self.minimum = min(finite)
            self.maximum = max(finite)
        else:
            self.minimum = math.nan
            self.maximum = math.nan
        self.update()

    def paintEvent(self, event) -> None:
        del event
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing, False)
        painter.fillRect(self.rect(), QtGui.QColor("#0d1117"))

        frame_rect = self._frame_rect()
        if not self.values:
            painter.setPen(QtGui.QColor("#8b949e"))
            painter.drawText(frame_rect, QtCore.Qt.AlignCenter, "Waiting for frame")
            return

        image = QtGui.QImage(self.width_px, self.height_px, QtGui.QImage.Format_RGB32)
        low, high = self._scale_limits()

        for row in range(self.height_px):
            for col in range(self.width_px):
                value = self.values[row * self.width_px + col]
                color = self._color_for(value, low, high)
                image.setPixelColor(self.width_px - 1 - col, row, color)

        painter.drawImage(frame_rect, image.scaled(
            frame_rect.size(),
            QtCore.Qt.IgnoreAspectRatio,
            QtCore.Qt.FastTransformation,
        ))

        painter.setPen(QtGui.QPen(QtGui.QColor("#30363d"), 1))
        painter.drawRect(frame_rect)
        self._draw_scale(painter, frame_rect, low, high)

    def _scale_limits(self) -> tuple[float, float]:
        low = self.min_temp if self.min_temp is not None else self.minimum
        high = self.max_temp if self.max_temp is not None else self.maximum
        if not math.isfinite(low):
            low = high - 1.0 if math.isfinite(high) else 0.0
        if not math.isfinite(high):
            high = low + 1.0
        if self.min_temp is None or self.max_temp is None:
            if low >= high or math.isclose(low, high):
                if self.min_temp is None and self.max_temp is None:
                    low -= 1.0
                    high += 1.0
                elif self.min_temp is None:
                    low = high - 1.0
                else:
                    high = low + 1.0
        return low, high

    def _frame_rect(self) -> QtCore.QRect:
        outer = self.rect().adjusted(12, 12, -78, -12)
        target_ratio = self.width_px / self.height_px
        current_ratio = outer.width() / max(outer.height(), 1)
        if current_ratio > target_ratio:
            width = int(outer.height() * target_ratio)
            x = outer.left() + (outer.width() - width) // 2
            return QtCore.QRect(x, outer.top(), width, outer.height())
        height = int(outer.width() / target_ratio)
        y = outer.top() + (outer.height() - height) // 2
        return QtCore.QRect(outer.left(), y, outer.width(), height)

    def _draw_scale(self, painter: QtGui.QPainter, frame_rect: QtCore.QRect, low: float, high: float) -> None:
        scale = QtCore.QRect(frame_rect.right() + 18, frame_rect.top(), 22, frame_rect.height())
        for y in range(scale.height()):
            t = 1.0 - y / max(scale.height() - 1, 1)
            painter.setPen(self._palette(t))
            painter.drawLine(scale.left(), scale.top() + y, scale.right(), scale.top() + y)

        painter.setPen(QtGui.QColor("#c9d1d9"))
        painter.drawText(scale.right() + 8, scale.top() + 8, f"{high:.1f} C")
        painter.drawText(scale.right() + 8, scale.bottom(), f"{low:.1f} C")
        painter.setPen(QtGui.QPen(QtGui.QColor("#30363d"), 1))
        painter.drawRect(scale)

    def _color_for(self, value: float | None, low: float, high: float) -> QtGui.QColor:
        if value is None or not math.isfinite(value):
            return QtGui.QColor("#111111")
        t = (value - low) / (high - low)
        return self._palette(max(0.0, min(1.0, t)))

    @staticmethod
    def _palette(t: float) -> QtGui.QColor:
        stops = [
            (0.00, (18, 20, 58)),
            (0.18, (31, 87, 151)),
            (0.38, (46, 161, 171)),
            (0.58, (240, 202, 74)),
            (0.78, (226, 103, 51)),
            (1.00, (250, 241, 220)),
        ]
        for index in range(len(stops) - 1):
            left_t, left = stops[index]
            right_t, right = stops[index + 1]
            if t <= right_t:
                local = (t - left_t) / (right_t - left_t)
                rgb = [
                    int(left[channel] + (right[channel] - left[channel]) * local)
                    for channel in range(3)
                ]
                return QtGui.QColor(*rgb)
        return QtGui.QColor(*stops[-1][1])


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, subscriber: FrameSubscriber, min_temp: float | None = None,
                 max_temp: float | None = None):
        super().__init__()
        self.subscriber = subscriber
        self.last_message_time = None

        self.setWindowTitle("MLX90640 Frame Viewer")
        self.resize(900, 620)

        self.status_label = QtWidgets.QLabel(self.subscriber.label)
        self.status_label.setObjectName("statusLabel")

        self.min_tile = StatTile("Min", " C")
        self.avg_tile = StatTile("Average", " C")
        self.max_tile = StatTile("Max", " C")
        self.frame_tile = StatTile("Frame")
        self.count_tile = StatTile("Pixels")
        self.age_tile = StatTile("Age", " s")
        self.frame_widget = ThermalFrame(min_temp, max_temp)

        tile_row = QtWidgets.QHBoxLayout()
        for tile in (
            self.min_tile,
            self.avg_tile,
            self.max_tile,
            self.frame_tile,
            self.count_tile,
            self.age_tile,
        ):
            tile_row.addWidget(tile)

        central = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(central)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(12)
        layout.addWidget(self.status_label)
        layout.addLayout(tile_row)
        layout.addWidget(self.frame_widget, 1)
        self.setCentralWidget(central)

        self.poll_timer = QtCore.QTimer(self)
        self.poll_timer.timeout.connect(self.poll_messages)
        self.poll_timer.start(15)

        self.age_timer = QtCore.QTimer(self)
        self.age_timer.timeout.connect(self.update_age)
        self.age_timer.start(250)

    def closeEvent(self, event) -> None:
        self.subscriber.close()
        super().closeEvent(event)

    def poll_messages(self) -> None:
        try:
            message = self.subscriber.receive_latest()
        except (json.JSONDecodeError, UnicodeDecodeError, zmq.ZMQError, RuntimeError) as error:
            self.status_label.setText(f"Receive error: {error}")
            return

        if message is None:
            return

        self.last_message_time = time.monotonic()
        self.status_label.setText("Receiving")
        self.apply_message(message)

    def apply_message(self, message: dict) -> None:
        minimum = float(message.get("min", math.nan))
        average = float(message.get("avg", math.nan))
        maximum = float(message.get("max", math.nan))

        self.min_tile.set_value(minimum)
        self.avg_tile.set_value(average)
        self.max_tile.set_value(maximum)
        self.frame_tile.set_value(message.get("frame"))
        self.count_tile.set_value(message.get("count"))
        self.age_tile.set_value(0.0)

        self.frame_widget.set_frame(
            int(message.get("width", DEFAULT_WIDTH)),
            int(message.get("height", DEFAULT_HEIGHT)),
            message.get("temperatures", []),
        )

    def update_age(self) -> None:
        if self.last_message_time is None:
            self.age_tile.set_value(None)
            return

        age = time.monotonic() - self.last_message_time
        self.age_tile.set_value(age)
        if age > 3.0:
            self.status_label.setText(f"No message for {age:.1f}s")


def parse_args(argv: list[str] | None = None,
               parser: argparse.ArgumentParser | None = None) -> argparse.Namespace:
    if parser is None:
        parser = argparse.ArgumentParser(description="PyQt5 viewer for MLX90640 frames")
    parser.add_argument(
        "--transport",
        choices=("zmq", "ecal"),
        default="zmq",
        help="Transport to use for receiving frames",
    )
    parser.add_argument("--connect", default=DEFAULT_ENDPOINT, help="ZeroMQ endpoint to connect to")
    parser.add_argument("--topic", default=DEFAULT_ECAL_TOPIC, help="eCAL topic to subscribe to")
    parser.add_argument("--min-temp", type=float, help="Color scale minimum in Celsius (default: frame minimum)")
    parser.add_argument("--max-temp", type=float, help="Color scale maximum in Celsius (default: frame maximum)")
    args = parser.parse_args(argv)
    for name in ("min_temp", "max_temp"):
        value = getattr(args, name)
        if value is not None and not math.isfinite(value):
            parser.error(f"--{name.replace('_', '-')} must be finite")
    if args.min_temp is not None and args.max_temp is not None and args.min_temp >= args.max_temp:
        parser.error("--min-temp must be less than --max-temp")
    return args


def make_subscriber(args: argparse.Namespace) -> FrameSubscriber:
    if args.transport == "ecal" and ecal_core is not None:
        return EcalFrameSubscriber(args.topic)
    return ZmqFrameSubscriber(args.connect)


def apply_style(app: QtWidgets.QApplication) -> None:
    app.setStyleSheet(
        """
        QMainWindow, QWidget {
            background: #0d1117;
            color: #e6edf3;
            font-family: Inter, DejaVu Sans, Arial, sans-serif;
            font-size: 13px;
        }
        #statusLabel {
            color: #8b949e;
            padding: 2px;
        }
        #statTile {
            background: #161b22;
            border: 1px solid #30363d;
            border-radius: 6px;
        }
        #tileTitle {
            color: #8b949e;
            font-size: 12px;
        }
        #tileValue {
            color: #f0f6fc;
            font-size: 22px;
            font-weight: 600;
        }
        """
    )


def main() -> int:
    args = parse_args()
    try:
        subscriber = make_subscriber(args)
    except RuntimeError as error:
        print(error, file=sys.stderr)
        return 1

    app = QtWidgets.QApplication(sys.argv)
    apply_style(app)
    window = MainWindow(subscriber, args.min_temp, args.max_temp)
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
