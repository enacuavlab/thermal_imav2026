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
