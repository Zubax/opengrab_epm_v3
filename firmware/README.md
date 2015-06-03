OpenGrab EPM 688 V3 Firmware
============================

This directory contains sources of the official firmware for OpenGrab EPM 688 V3 (electropermanent magnet).

Useful links:

* Link to NicaDrone.
* Link to the Zubax documentation portal.
* Some other link.

## Hardware configuration

The following table summarizes the use of GPIO pins (special-purpose pins, like CAN PHY, are not listed).
Refer to the schematic for details.

| LPC11C24 pin | Type      | Function                           |
| ------------ | --------- | ---------------------------------- |
|   PIO0_11    | AIN       | Vin_ADC
|   PIO1_0     | DOUT      | CTRL_1
|   PIO1_1     | DOUT      | CTRL_3
|   PIO1_5     | DIN       | DIP_1
|   PIO1_6     | DIN       | DIP_2
|   PIO1_7     | DIN       | DIP_3
|   PIO2_0     | DOUT      | Status_LED
|   PIO2_6     | DOUT      | CAN_LED
|   PIO2_7     | DOUT      | CTRL_2
|   PIO2_8     | DOUT      | CTRL_4
|   PIO2_10    | DIN       | PWM
|   PIO3_0     | DOUT      | SW_L
|   PIO3_1     | DOUT      | SW_H
|   PIO3_3     | DIN       | DIP_4

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

If you're using Black Magic Probe, use the script `blackmagic_flash.sh`.

If you're using something else, you know what to do.
