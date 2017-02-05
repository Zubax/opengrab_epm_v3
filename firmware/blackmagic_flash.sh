#!/bin/bash
#
# Copyright (c) 2015 Zubax Robotics, zubax.com
# Distributed under the MIT License, available in the file LICENSE.
# Author: Pavel Kirienko <pavel.kirienko@zubax.com>
#

BM_DEV=$(readlink -f /dev/serial/by-id/usb-Black_Sphere_Technologies_Black_Magic_Probe_*-if00)
PORT=${1:-$BM_DEV}
echo "Using port $PORT"

elf=build/firmware.elf

arm-none-eabi-size $elf || exit 1

tmpfile=fwupload.tempfile
cat > $tmpfile <<EOF
exec-file $elf
target extended-remote $PORT
mon swdp_scan
attach 1
load
kill
EOF

arm-none-eabi-gdb --batch -x $tmpfile
rm $tmpfile
