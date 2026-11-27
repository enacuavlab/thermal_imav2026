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

To send only the thermal image, using the viewer's palette and horizontal flip, to
the PprzGCS video panel, install `visu/requirements.txt` and run:

```bash
python3 visu/stats_udp.py --connect tcp://127.0.0.1:5555
```

The sender is a CLI app with no window or display-server requirement. It sends
only thermal pixels, without statistics, text, borders, or a color scale, as JPEGs to
`127.0.0.1:5600`, one per UDP datagram. In PprzGCS Devices settings, add a stream
with source `udp://:5600` and select it in the Video panel. If no cameras are
configured, the panel's default source already uses that port. A layout can
also configure it explicitly:

```xml
<widget name="video"><configure source="udp://:5600" width="640"/></widget>
```

For eCAL input, a fixed temperature scale, or a different port:

```bash
python3 visu/stats_udp.py --transport ecal --topic mlx90640_data \
  --min-temp 15 --max-temp 40 --port 5602
```

Use `udp://:5602` in PprzGCS for that example. The same eCAL availability and
ZeroMQ fallback behavior as `stats_viewer.py` applies.
`--scale` (default 20) enlarges sensor pixels without smoothing, producing
640×480 images for 32×24 input; use `--scale 1` for native resolution.
The GCS video panel displays frames as received. Use `--rotate 90`,
`--rotate 180`, or `--rotate 270` to rotate clockwise in the sender
(default: `--rotate 0`). Rotation follows the viewer's horizontal flip;
90° and 270° swap the output width and height. For example:

```bash
python3 visu/stats_udp.py --rotate 90 --port 5600
```

`--fps` (default 10), `--quality`
(default 85), and `--host` (default `127.0.0.1`) control output. JPEG quality
is reduced automatically if necessary to fit the 65,507-byte UDP payload limit;
exceptionally large images are also scaled down. The sender continues streaming
the last image when input stops, and waits for the first thermal frame before
sending video. Stop with Ctrl+C.

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
