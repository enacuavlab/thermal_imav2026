# Thermal MLX90640

Small MLX90640 thermal camera project with:

- a C/C++ reader using the Melexis driver API
- ZeroMQ and eCAL publishing of full 32x24 temperature frames
- a PyQt5 viewer for live visualization

## Wifi

SSID: OctIR
password: same as in voliere
address: 192.168.10.1/24
No DHCP, so set a manual address in the /24 range.



## Allow passwordless shutdown

sudo cp pprzlink-shutdown /etc/sudoers.d/


## Build

Dependencies:

- CMake
- CLI11
- libzmq
- eCAL
- pkg-config

```bash
cmake -S . -B build
cmake --build build
```

## Run Publisher

```bash
./build/mlx90640_stats --device /dev/i2c-17 --publish tcp://*:5555
```

Useful options:

```bash
./build/mlx90640_stats --frames 10
./build/mlx90640_stats --refresh-mode 3
./build/mlx90640_stats --address 0x33
```

## Run Viewer

```bash
pip install -r visu/requirements.txt
python3 visu/stats_viewer.py --connect tcp://127.0.0.1:5555
```

To receive the protobuf frames over eCAL instead:

```bash
python3 visu/stats_viewer.py --transport ecal --topic mlx90640_data
```

The viewer receives JSON frames over ZeroMQ or `ThermalData` protobuf frames over eCAL.

## Thermal Spotter

`sw/thermal_spotter.py` is the main processing application. It combines thermal
frames with Paparazzi telemetry, projects detected temperature extrema onto the
ground, clusters nearby detections, and publishes the affected cluster.

Run hot-spot detection with the defaults:

```bash
python3 sw/thermal_spotter.py --threshold 40
```

For cold-spot detection, use a threshold as an upper bound:

```bash
python3 sw/thermal_spotter.py --spot-kind cold --threshold 40
```

Useful options:

```bash
python3 sw/thermal_spotter.py \
  --map \
  --geotiff path/to/background.tif \
  --cluster-distance 3 \
  --ground-altitude 0
```

`--spot-kind hot` selects the maximum finite pixel and reports it when the
temperature is at least the threshold. `--spot-kind cold` selects the minimum
finite pixel and reports it when the temperature is at most the threshold.

### Clusters and Map

Spots are grouped using horizontal ground distance. A cluster contains every
spot connected by one or more links no greater than `--cluster-distance`; the
default is 3 m. The cluster position is the ENU barycenter of its members and
its temperature is the median of their temperatures.

With `--map`, the OpenCV map shows individual spots as magenta crosshairs and
clusters as green stars. Use the clickable `Spots` and `Clusters` checkboxes in
the upper-right corner of the map window to show or hide either layer.

### eCAL Topics and PPRZLink

| Direction | Topic | Message |
| --- | --- | --- |
| Input | `mlx90640_data` | `ThermalData` frames |
| Input | `pprz_msg` | Paparazzi `PprzMsg` telemetry and commands |
| Output | `pprz_msg_tx` | Paparazzi `PprzMsg` cluster reports |

The spotter consumes `ROTORCRAFT_FP` for aircraft pose, `INS_REF` for local
ENU-to-WGS84 conversion, `GPS_INT` for GeoTIFF map positioning, and
`PAYLOAD_COMMAND` for runtime commands. Send `PAYLOAD_COMMAND.command` as
`['t', threshold]` to update the threshold in degrees Celsius; the command
array is encoded as `uint8[]`, so `'t'` is ASCII byte `0x74`.

Each accepted detection emits one `telemetry.PAYLOAD_FLOAT` message for its
updated cluster, addressed to broadcast ID `0xFF`. Its float values are:

```text
cluster_key, east, north, up, latitude, longitude, median_temperature, member_count
```

The `cluster_key` is the lowest spot sequence number in the cluster. It is a
deterministic key for the current cluster composition, not the consecutive
number shown on the map.

## Serial PPRZLink Bridge

`sw/pprzlink_serial_sniffer.py` bridges transparent PPRZLink v2 packets and
eCAL in both directions:

```bash
python3 sw/pprzlink_serial_sniffer.py /dev/ttyS3 --baudrate 115200
```

Packets received over serial are published to `pprz_msg`. `PprzMsg` messages
published to `pprz_msg_tx` are encoded as transparent PPRZLink v2 packets and
written to serial. Override either topic with `--rx-topic` or `--tx-topic`.




