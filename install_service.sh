#!/bin/bash

mkdir -p ~/.config/systemd/user/

rm ~/.config/systemd/user/pprz_*

ln -s ~/thermal/pprz_thermal.service ~/.config/systemd/user/pprz_thermal.service
ln -s ~/thermal/pprz_telemetry_sniffer.service ~/.config/systemd/user/pprz_telemetry_sniffer.service

sudo systemctl enable systemd-networkd-wait-online.service
systemctl --user enable pprz_thermal.service
systemctl --user enable pprz_telemetry_sniffer.service

systemctl --user daemon-reload

# start user.default target at startup
loginctl enable-linger $(whoami)

