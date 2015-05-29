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

If you're using Black Magic Probe, use the script `blackmagic_flash.sh`.

If you're using something else, you know what to do.
