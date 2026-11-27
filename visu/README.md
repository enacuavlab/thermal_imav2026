# MLX90640 Frame Viewer

PyQt5 viewer for the frame messages published by `mlx90640_stats`.

Start the publisher:

```bash
./build/mlx90640_stats --device /dev/i2c-17 --publish tcp://*:5555
```

Start the viewer:

```bash
python3 visu/stats_viewer.py --connect tcp://127.0.0.1:5555
```

Or receive the protobuf frames published on eCAL:

```bash
python3 visu/stats_viewer.py --transport ecal --topic mlx90640_data
```

To send the same viewer image (thermal frame, color scale, and statistics) to
the PprzGCS video panel, install `visu/requirements.txt` and run:

```bash
python3 visu/stats_udp.py --connect tcp://127.0.0.1:5555
```

The sender runs offscreen by default and sends complete JPEGs to
`127.0.0.1:5600`, one per UDP datagram. In PprzGCS Devices settings, add a stream
with source `udp://:5600` and select it in the Video panel. If no cameras are
configured, the panel's default source already uses that port. A layout can
also configure it explicitly:

```xml
<widget name="video"><configure source="udp://:5600" width="900"/></widget>
```

For eCAL input, a fixed temperature scale, or a different port:

```bash
python3 visu/stats_udp.py --transport ecal --topic mlx90640_data \
  --min-temp 15 --max-temp 40 --port 5602
```

Use `udp://:5602` in PprzGCS for that example. The same eCAL availability and
ZeroMQ fallback behavior as `stats_viewer.py` applies.
`--preview` also opens the local viewer; `--fps` (default 10), `--quality`
(default 85), and `--host` (default `127.0.0.1`) control output. JPEG quality
is reduced automatically if necessary to fit the 65,507-byte UDP payload limit;
exceptionally large images are also scaled down. The sender continues streaming
the last frame with its age when input stops, and shows “Waiting for frame”
before the first input arrives. Stop with Ctrl+C.

Expected message shape:

```json
{
  "frame": 0,
  "subpage": 1,
  "min": 21.123,
  "avg": 23.456,
  "max": 28.789,
  "count": 768,
  "width": 32,
  "height": 24,
  "temperatures": [23.1, 23.2]
}
```
