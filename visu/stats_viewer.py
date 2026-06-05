#!/usr/bin/env python3

"""PyQt5 viewer for MLX90640 ZeroMQ frame messages."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time

from PyQt5 import QtCore, QtGui, QtWidgets
import zmq


DEFAULT_ENDPOINT = "tcp://127.0.0.1:5555"
DEFAULT_WIDTH = 32
DEFAULT_HEIGHT = 24


class ZmqFrameSubscriber:
    def __init__(self, endpoint: str):
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
    def __init__(self):
        super().__init__()
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
        low = self.minimum
        high = self.maximum
        if not math.isfinite(low) or not math.isfinite(high):
            low, high = 0.0, 1.0
        if math.isclose(low, high):
            low -= 1.0
            high += 1.0

        for row in range(self.height_px):
            for col in range(self.width_px):
                value = self.values[row * self.width_px + col]
                color = self._color_for(value, low, high)
                image.setPixelColor(col, row, color)

        painter.drawImage(frame_rect, image.scaled(
            frame_rect.size(),
            QtCore.Qt.IgnoreAspectRatio,
            QtCore.Qt.FastTransformation,
        ))

        painter.setPen(QtGui.QPen(QtGui.QColor("#30363d"), 1))
        painter.drawRect(frame_rect)
        self._draw_scale(painter, frame_rect, low, high)

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
    def __init__(self, endpoint: str):
        super().__init__()
        self.subscriber = ZmqFrameSubscriber(endpoint)
        self.last_message_time = None

        self.setWindowTitle("MLX90640 Frame Viewer")
        self.resize(900, 620)

        self.status_label = QtWidgets.QLabel(f"SUB {endpoint}")
        self.status_label.setObjectName("statusLabel")

        self.min_tile = StatTile("Min", " C")
        self.avg_tile = StatTile("Average", " C")
        self.max_tile = StatTile("Max", " C")
        self.frame_tile = StatTile("Frame")
        self.count_tile = StatTile("Pixels")
        self.age_tile = StatTile("Age", " s")
        self.frame_widget = ThermalFrame()

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
        except (json.JSONDecodeError, UnicodeDecodeError, zmq.ZMQError) as error:
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
        print(message)

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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="PyQt5 viewer for MLX90640 ZeroMQ frames")
    parser.add_argument("--connect", default=DEFAULT_ENDPOINT, help="ZeroMQ endpoint to connect to")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    app = QtWidgets.QApplication(sys.argv)
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

    window = MainWindow(args.connect)
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
