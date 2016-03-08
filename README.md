OpenGrab EPM v3
===============

OpenGrab EPM v3 is an electropermanent magnet, combining the advantages of electro and permanent magnets.
It has been developed by [NicaDrone](http://nicadrone.com) in collaboration with [Zubax Robotics](http://zubax.com).

Useful links:

* [**PRODUCT DOCUMENTATION**](http://docs.zubax.com/opengrab_epm_v3)
* [HARDWARE SOURCES](https://tools.upverter.com/eda/#designId=1dada3422c772add)
* [SUPPORT FORUM](http://productforums.zubax.com)

## Building the firmware

### Prerequisites

* Python 2.7+ or Python 3.2+
* ARM GCC Embedded 4.9+

### Building

After cloning this repository, execute the following:

```bash
git submodule update --init --recursive
cd firmware
make -j8
```

The build outputs will be available in the directory `build/`.

## Flashing the firmware

### Via UART

#### Connecting USB-UART adapter

FTDI cable is the recommended USB-UART adapter.
5 V and 3.3 V FTDI cables work fine, 3.3 V is recommended.
Use the following pinout when connecting:

Wire    | Signal
--------|--------
Orange  | RXD
Yellow  | TXD
Black   | GND

#### Flashing with NXP Flash Magic

Get the flashing tool from <http://www.flashmagictool.com/>.
Start the tool, then configure it as follows:

Parameter       | Value
----------------|------------------------------------
Baud rate       | 115200
Interface       | ISP
Oscilator       | 12 MHz
File            | Use the `.hex` file in the build output directory**

1. Connect the USB-UART adapter.
2. Close J3 to select serial bootloader.
3. Close J4 to force the LPC to start the bootloader.
4. Power up the board.
5. Run the flashing tool.

** If needed the .hex file can be created from .bin:
```bash
arm-none-eabi-objcopy -I binary -O ihex firmware.bin out.hex
```

### Via DroneCode Probe

[DroneCode Probe](https://docs.zubax.com/dronecode_probe) is a generic JTAG / SWD + UART console adapter
compatible with most ARM Cortex based designs and in particular with hardware maintained by the
[DroneCode project](http://dronecode.org).

In order to flash the board using this tool, simply connect the debugger and execute the script
`blackmagic_flash.sh`.

### Via [8devices USB2CAN](http://www.8devices.com/usb2can) and embedded CAN bootloader

Note to Windows users: In order to use USB2CAN in a VM the drivers have to be installed on the host and driver
signature enforcement has to be disabled in some versions of Windows.

Register the CAN interface with SocketCAN:

```bash
sudo ip link set can0 up type can bitrate 100000 sample-point 0.875
```

Uploading binary using LPC11 embedded CAN bootloader:

```bash
tools/drwatson/lpc11c00_can_bootloader.py can0 firmware/build/firmware.bin
```

## Licenses

### Firmware

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

### Hardware

The hardware sources are distributed under the terms of
[CC BY-SA 4.0](https://creativecommons.org/licenses/by-sa/4.0/).
