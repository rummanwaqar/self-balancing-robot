#!/usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
sudo socat -d -d pty,raw,echo=0,link=/dev/ttyUSB3,user=root,group=dialout,mode=660 pty,raw,echo=0,link=/dev/ttyUSB4,user=root,group=dialout,mode=660 &
sleep 10s
echo 'Outputing /dev/ttyUSB4'
python $DIR/ble_robot.py
