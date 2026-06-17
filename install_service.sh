#!/bin/bash

mkdir -p ~/.config/systemd/user/

rm ~/.config/systemd/user/pprz_*

ln -s ~/thermal/pprz_thermal.service ~/.config/systemd/user/pprz_thermal.service

sudo systemctl enable systemd-networkd-wait-online.service
systemctl --user enable pprz_thermal.service

systemctl --user daemon-reload

# start user.default target at startup
loginctl enable-linger $(whoami)

