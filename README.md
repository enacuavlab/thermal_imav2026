# Thermal MLX90640

Small MLX90640 thermal camera project with:

- a C/C++ reader using the Melexis driver API
- ZeroMQ and eCAL publishing of full 32x24 temperature frames
- a PyQt5 viewer for live visualization

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
