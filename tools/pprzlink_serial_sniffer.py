#!/usr/bin/env python3

"""Print transparent PPRZLink packets received from a serial device."""

from __future__ import annotations

import argparse
import sys
from datetime import datetime, timedelta, timezone
from pathlib import Path
import subprocess


GENERATED_DIR = Path(__file__).resolve().parent.parent / "generated"
DEFAULT_PPRZLINK_PYTHON = (
    "/home/pprz/pprzlink/lib/v2.0/python/src"
)
DEFAULT_ECAL_TOPIC = "pprz_msg"

sys.path.insert(0, str(Path(DEFAULT_PPRZLINK_PYTHON).expanduser()))
sys.path.append(str(GENERATED_DIR))

import ecal.nanobind_core as ecal_core
import serial
from ecal.msg.proto.core import Publisher as ProtoPublisher
from pprzlink import messages_xml_map
from pprzlink.message import PprzMessage
from pprzlink.pprz_transport import PprzTransport
from thermal_pb2 import PprzMsg


ecal_rec: subprocess.Popen | None = None
GPS_WEEK = timedelta(days=7)
TIME_SYNC_THRESHOLD = timedelta(seconds=60)
TIME_SYNC_RETRY_INTERVAL = timedelta(seconds=60)
last_time_sync_attempt: datetime | None = None


def resolve_message_name(class_id: int, msg_id: int) -> str:
    try:
        class_name = messages_xml_map.get_class_name(class_id)
        msg_name = messages_xml_map.get_msg_name(class_name, msg_id)
    except ValueError:
        return f"class:{class_id}/msg:{msg_id}"

    return f"{class_name}.{msg_name}"


class EcalPprzPublisher:
    def __init__(self, topic: str):
        self._owns_ecal = not ecal_core.is_initialized()
        if self._owns_ecal:
            ecal_core.initialize("PPRZLink Serial Sniffer")
        self._publisher = ProtoPublisher(PprzMsg, topic)

    def close(self) -> None:
        if self._owns_ecal and ecal_core.is_initialized():
            ecal_core.finalize()

    def publish(
        self,
        source: int,
        destination: int,
        class_id: int,
        msg_id: int,
        message_name: str,
        payload: bytes,
    ) -> None:
        msg = PprzMsg(
            src=source,
            dst=destination,
            msg_class=class_id,
            msg_id=msg_id,
            msg_name=message_name,
            payload=payload,
        )
        self._publisher.send(msg)


def parse_packet(buffer: bytes) -> tuple[int, int, int, int, str, bytes]:
    if len(buffer) < 4:
        raise ValueError(f"PPRZ payload too short: {len(buffer)} byte(s)")

    source = buffer[0]
    destination = buffer[1]
    class_id = buffer[2] & 0x0F
    msg_id = buffer[3]
    payload = bytes(buffer[4:])
    message_name = resolve_message_name(class_id, msg_id)
    return source, destination, class_id, msg_id, message_name, payload


def format_payload(payload: bytes) -> str:
    return payload.hex(" ")


def infer_datetime_from_gps_tow(
    tow_ms: int,
    now: datetime | None = None,
) -> datetime:
    if now is None:
        now = datetime.now(timezone.utc)

    days_since_gps_week_start = (now.weekday() + 1) % 7
    week_start = (now - timedelta(days=days_since_gps_week_start)).replace(
        hour=0,
        minute=0,
        second=0,
        microsecond=0,
    )
    current_week_date = week_start + timedelta(milliseconds=tow_ms)
    candidates = (
        current_week_date - GPS_WEEK,
        current_week_date,
        current_week_date + GPS_WEEK,
    )
    return min(candidates, key=lambda candidate: abs(candidate - now))


def maybe_set_system_time_from_gps(gps_datetime: datetime, now: datetime) -> None:
    global last_time_sync_attempt

    drift = abs(gps_datetime - now)
    gps_datetime_str = gps_datetime.isoformat().replace("+00:00", "Z")
    if drift <= TIME_SYNC_THRESHOLD:
        return

    if (
        last_time_sync_attempt is not None
        and now - last_time_sync_attempt < TIME_SYNC_RETRY_INTERVAL
    ):
        print(
            f"GPS time sync skipped: date={gps_datetime_str} "
            f"drift={drift.total_seconds():.3f}s retry_interval="
            f"{TIME_SYNC_RETRY_INTERVAL.total_seconds():.0f}s",
            flush=True,
        )
        return

    last_time_sync_attempt = now
    epoch_seconds = int(gps_datetime.timestamp())
    print(
        f"GPS time sync correcting system time: date={gps_datetime_str} "
        f"drift={drift.total_seconds():.3f}s",
        flush=True,
    )
    try:
        subprocess.run(
            ["sudo", "date", "-u", "-s", f"@{epoch_seconds}"],
            check=False,
        )
    except OSError as exc:
        print(f"GPS time sync failed to run sudo date: {exc}", file=sys.stderr, flush=True)


def start_ecal_recording() -> None:
    global ecal_rec

    if ecal_rec is not None and ecal_rec.poll() is None:
        print("ecal_rec is already recording")
        return

    print("start recording")
    ecal_rec = subprocess.Popen(["ecal_rec", "-r", "--activate", "-d", "/home/pprz/sd/logs"])


def stop_ecal_recording() -> None:
    global ecal_rec

    if ecal_rec is None:
        return

    if ecal_rec.poll() is None:
        print("stop recording")
        ecal_rec.terminate()
        try:
            ecal_rec.wait(timeout=5)
        except subprocess.TimeoutExpired:
            ecal_rec.kill()
            ecal_rec.wait()

    ecal_rec = None


def on_payload_command(
    source: int,
    destination: int,
    pprz_message: PprzMessage,
) -> None:
    # PAYLOAD_COMMAND needs domain-specific handling here.
    if chr(pprz_message.command[0]) == 'r':
        if pprz_message.command[1] == 1:
            start_ecal_recording()
        elif pprz_message.command[1] == 0:
            stop_ecal_recording()
    elif chr(pprz_message.command[0]) == 's':
        if pprz_message.command[1] == 0:
            print("shutdown")
            stop_ecal_recording()
            subprocess.run(["sudo", "shutdown", "now"], check=False)
    else:
        print(pprz_message)


def on_gps_int(pprz_message: PprzMessage) -> None:
    now = datetime.now(timezone.utc)
    gps_datetime = infer_datetime_from_gps_tow(pprz_message.tow, now)
    gps_datetime_str = gps_datetime.isoformat().replace("+00:00", "Z")
    #print(f"GPS_INT tow={pprz_message.tow} date={gps_datetime_str}", flush=True)
    maybe_set_system_time_from_gps(gps_datetime, now)


def decode_pprz_message(class_id: int, msg_id: int, payload: bytes) -> PprzMessage:
    pprz_message = PprzMessage(class_id, msg_id)
    pprz_message.binary_to_payload(payload)
    return pprz_message


def dispatch_message(
    source: int,
    destination: int,
    class_id: int,
    msg_id: int,
    message_name: str,
    payload: bytes,
) -> None:
    short_message_name = message_name.rsplit(".", 1)[-1]
    if short_message_name in {"PAYLOAD_COMMAND", "GPS_INT"}:
        try:
            pprz_message = decode_pprz_message(class_id, msg_id, payload)
        except Exception as exc:
            print(f"Could not decode {message_name}: {exc}", file=sys.stderr, flush=True)
            return

    if short_message_name == "GPS_INT":
        on_gps_int(pprz_message)
    elif short_message_name == "PAYLOAD_COMMAND":
        on_payload_command(
            source,
            destination,
            pprz_message,
        )


def run(args: argparse.Namespace) -> None:
    if args.messages:
        messages_xml_map.parse_messages(args.messages)

    transport = PprzTransport()
    ecal_publisher = EcalPprzPublisher(args.ecal_topic)

    try:
        with serial.Serial(args.device, args.baudrate, timeout=args.timeout) as port:
            print(
                f"Listening on {args.device} at {args.baudrate} baud, "
                f"publishing eCAL topic {args.ecal_topic}",
                flush=True,
            )
            while True:
                byte = port.read(1)
                if not byte:
                    continue

                if not transport.parse_byte(byte):
                    continue

                try:
                    source, destination, class_id, msg_id, message_name, payload = parse_packet(
                        bytes(transport.get_buffer())
                    )
                except ValueError as exc:
                    print(f"Skipping packet: {exc}", file=sys.stderr, flush=True)
                    continue

                dispatch_message(
                    source,
                    destination,
                    class_id,
                    msg_id,
                    message_name,
                    payload,
                )
                ecal_publisher.publish(
                    source,
                    destination,
                    class_id,
                    msg_id,
                    message_name,
                    payload,
                )
                if args.verbose:
                    print(
                        f"source={source} destination={destination} "
                        f"message={message_name} payload={format_payload(payload)}",
                        flush=True,
                    )
    finally:
        stop_ecal_recording()
        ecal_publisher.close()


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Open a serial device, read transparent PPRZLink v2 packets, "
            "and print source, destination, message name, and binary payload."
        )
    )
    parser.add_argument("device", help="serial device, e.g. /dev/ttyUSB0")
    parser.add_argument(
        "-b",
        "--baudrate",
        type=int,
        default=115200,
        help="serial baudrate (default: %(default)s)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=1.0,
        help="serial read timeout in seconds (default: %(default)s)",
    )
    parser.add_argument(
        "--messages",
        type=Path,
        help="optional messages.xml path for message-name lookup",
    )
    parser.add_argument(
        "--ecal-topic",
        default=DEFAULT_ECAL_TOPIC,
        help="eCAL topic for PprzMsg publishing (default: %(default)s)",
    )
    parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="print received messages to stdout",
    )
    args = parser.parse_args()

    try:
        run(args)
    except KeyboardInterrupt:
        print("\nStopped", file=sys.stderr)


if __name__ == "__main__":
    main()
