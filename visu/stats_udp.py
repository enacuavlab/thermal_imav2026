#!/usr/bin/env python3

"""Stream thermal images as JPEG datagrams to the PprzGCS video viewer."""

from __future__ import annotations

import argparse
import math
import signal
import socket
import sys

from PyQt5 import QtCore, QtGui
import zmq

import stats_viewer


MAX_DATAGRAM_SIZE = 65507  # Maximum IPv4 UDP payload, including the complete JPEG.


def encode_frame(image: QtGui.QImage, quality: int) -> bytes:
    """Keep a complete JPEG in one datagram; PprzGCS does no reassembly."""
    while True:
        for current_quality in range(quality, 0, -5):
            data = QtCore.QByteArray()
            buffer = QtCore.QBuffer(data)
            buffer.open(QtCore.QIODevice.WriteOnly)
            if not image.save(buffer, "JPEG", current_quality):
                raise RuntimeError("Qt JPEG encoder unavailable")
            buffer.close()
            if data.size() <= MAX_DATAGRAM_SIZE:
                return bytes(data)
        # Very large or detailed frames may need a smaller image as well.
        if image.width() == 1 and image.height() == 1:
            raise RuntimeError("Cannot fit JPEG in a UDP datagram")
        image = image.scaled(
            max(1, image.width() * 3 // 4),
            max(1, image.height() * 3 // 4),
            QtCore.Qt.KeepAspectRatio,
            QtCore.Qt.SmoothTransformation,
        )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1", help="Destination host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=5600, help="Destination UDP port (default: 5600)")
    parser.add_argument("--fps", type=float, default=10, help="Output frames per second (default: 10)")
    parser.add_argument("--quality", type=int, default=85, help="Maximum JPEG quality, 1-100 (default: 85)")
    parser.add_argument("--scale", type=int, default=20, help="Pixel enlargement factor (default: 20, giving 640x480 for 32x24 input)")
    args = stats_viewer.parse_args(argv, parser)
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    if not math.isfinite(args.fps) or not 0.1 <= args.fps <= 1000:
        parser.error("--fps must be between 0.1 and 1000")
    if not 1 <= args.quality <= 100:
        parser.error("--quality must be between 1 and 100")
    if not 1 <= args.scale <= 100:
        parser.error("--scale must be between 1 and 100")
    return args


def main() -> int:
    args = parse_args()
    app = QtCore.QCoreApplication([sys.argv[0]])
    try:
        destination = (socket.gethostbyname(args.host), args.port)
        subscriber = stats_viewer.make_subscriber(args)
    except (OSError, RuntimeError) as error:
        print(error, file=sys.stderr)
        return 1

    frame = stats_viewer.ThermalImage(args.min_temp, args.max_temp)
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    exit_code = 0
    jpeg = None

    def send_frame() -> None:
        nonlocal exit_code, jpeg
        try:
            message = subscriber.receive_latest()
            if message is not None:
                frame.set_frame(
                    int(message.get("width", stats_viewer.DEFAULT_WIDTH)),
                    int(message.get("height", stats_viewer.DEFAULT_HEIGHT)),
                    message.get("temperatures", []),
                )
                if frame.values:
                    image = frame.to_image().scaled(
                        frame.width_px * args.scale, frame.height_px * args.scale,
                        QtCore.Qt.IgnoreAspectRatio, QtCore.Qt.FastTransformation,
                    )
                    jpeg = encode_frame(image, args.quality)
            if jpeg is not None:
                udp.sendto(jpeg, destination)
        except (ValueError, TypeError, UnicodeDecodeError, zmq.ZMQError) as error:
            print(f"Invalid thermal frame: {error}", file=sys.stderr)
        except (OSError, RuntimeError) as error:
            print(f"UDP video error: {error}", file=sys.stderr)
            exit_code = 1
            app.quit()

    timer = QtCore.QTimer()
    timer.timeout.connect(send_frame)
    timer.start(max(1, round(1000 / args.fps)))
    previous_handlers = {
        signum: signal.signal(signum, lambda *_: app.quit())
        for signum in (signal.SIGINT, signal.SIGTERM)
    }
    print(f"Sending JPEG video to udp://{args.host}:{args.port} at {args.fps:g} fps", flush=True)
    try:
        app.exec_()
    finally:
        timer.stop()
        subscriber.close()
        udp.close()
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
