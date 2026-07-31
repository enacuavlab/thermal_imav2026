
# Installation

## Flasher la radxa zero 3W avec eeprom

Installer Rkdeveloptool :
https://docs.radxa.com/en/som/cm/cm3/low-level-dev/rkdeveloptool?host-os=debian

Appuyer sur le bouton "bootrom" en connectant l'USB au PC, puis:

```
sudo rkdeveloptool db rk356x_spl_loader_ddr1056_v1.12.109_no_check_todly.bin
sudo rkdeveloptool wl 0 radxa-zero3_bookworm_pprz.img
```



## Wifi
Show connections:

`nmcli connection show`


Activate "octir" connection:

`sudo nmcli connection up octir`


# Overlays

sudo rsetup
Overlays -> Install 3rd party overlay -> i2c400k.dtbo

Overlays -> Manage ->
Enable I2C4-M0
Enable UART3-M0


git clone https://github.com/paparazzi/pprzlink.git
clone thermal

python3 -m venv termenv
echo "source /home/pprz/termenv/bin/activate" >> ~/.bashrc

## Install eCAL

`cd sd/ecal`
`sudo dpkg -i build/_deploy/ecal_6.1.1_arm64.deb`
`pip install dist/eclipse_ecal-6.2.0a1.dev50+g7a8ee406d.d20260616-cp311-cp311-linux_aarch64.whl`


## thermal

pip install -r requirements.txt

./install_service.sh

Add to the fstab:
`/dev/mmcblk1p1  /home/pprz/sd  ext4  defaults,noatime  0  2`


## Allow passwordless shutdown

sudo cp pprzlink-shutdown /etc/sudoers.d/

# Build MLX driver

sudo apt install cmake libprotobuf-dev protobuf-compiler libcli11-dev libzmq3-dev
makdir build && cd build
cmake ..
make -j4
