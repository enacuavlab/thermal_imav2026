
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
Overlays -> Install 3rd party overlay -> i2c400k

Overlays -> Manage ->
Enable I2C4-M0
Enable UART3-M0


git clone https://github.com/paparazzi/pprzlink.git
clone thermal

python3 -m venv thermenv
echo "source /home/pprz/thermenv/bin/activate" >> .bashrc


build/_deploy/ecal_6.1.1_arm64.deb

dist/eclipse_ecal-6.2.0a1.dev50+g7a8ee406d.d20260616-cp311-cp311-linux_aarch64.whl


## thermal

pip install -r requirements.txt

./install_service.sh

Add to the fstab:
`/dev/mmcblk1p1  /home/pprz/sd  ext4  defaults,noatime  0  2`
