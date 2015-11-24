OpenGrab EPM 688 V3 Firmware
============================

This directory contains sources of the official firmware for OpenGrab EPM 688 V3 (electropermanent magnet).

Useful links:

* Link to NicaDrone.
* Link to the Zubax documentation portal.
* Some other link.

## Building & Running

### Prerequisites

* Python 2.7+ or Python 3.2+
* ARM GCC Embedded 4.9+

### Building

After cloning this repository, execute the following:

```bash
git submodule update --init --recursive
cd firmware
make
```

### Flashing

#### Connecting FTDI cable:

5V and 3V3 FTDI cables work fine, 3V3 is recommended
Using this as reference https://tools.upverter.com/eda/#designId=1dada3422c772add,tool=pcb

Connect Orange to RXD
Connect Yellow to TXD 
Connect black to GND 

#### Flashing with NXP Flash Magic serial flasher

Connect FTDI cable 

Close J3 select to serial bootloader 

Close J4 force the LPC start a bootloader

Power up the board 

Get the NXP Flash Magic serial flasher
http://www.flashmagictool.com/

Baud Rate: 115200

Interface: ISP

Oscilator: 12Mhz

File: firmware.hex

Start!

#### Black Magic Probe	//should that not be Drone Code Probe?

Use the script `blackmagic_flash.sh`.

#### Something else

You know what to do.
