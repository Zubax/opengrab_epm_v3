# NicaDrone OpenGrab EPM688 V3.3 firmware update

OpenGrab Electropermanent Magnet



## Building the firmeware on Debian-based system


Clone the repo:

cd ..../home/git (or any directory you like)

git clone https://github.com/Zubax/opengrab_epm.git

todo: something is missing here

cd opengrab_epm

make

cd build

ls

..

firmware.bin
firmware.elf
firmware.hex

## Flashing firmware over serial in Windows based system

Close J4 force the LPC start a bootloader

Close J3 select to serial bootloader
alternatively open J3 to start the CAN bootloader

Connect the EPM UART TX and RX to an FTDI cable

Power up the board

Get the NXP Flash Magic serial flasher
http://www.flashmagictool.com/

Baud Rate: 115200

Interface: ISP

Oscilator: 12Mhz

File: firmware.hex

Start!


## Configuring CAN node ID

configurable hardpoint ID range is 0 to 7, inclusive
it is assigned by the lowest 3 bits of the DIP switch
the highest bit enables fixed node ID
if the highest bit is LOW, the device will perform dynamic node ID allocation
if the highest bit is HIGH, the device will use node ID that is (hardpoint ID + 100)
try that
therefore if the highest bit is 1, you don't need the dynamic node ID allocation server

## Something exploded?

oops sorry
Hardware:
Andreas@NicaDrone.com
Software:
Pavel@Zubax.com

## Firmware license

Copyright (C) 2015  Zubax Robotics <info@zubax.com>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
