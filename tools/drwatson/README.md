# Drwatson

Drwatson is a production testing utility.
It requires Ubuntu-based system and Python 3.4+.

## Setup Drwatson

First make sure that all submodules are checked out: `git submodule update --init --recursive`.

Then execute the setup script as superuser, and you're ready to go:

```bash
sudo ./setup.sh
sudo ./drwatson_epm_v3.py --iface=can0
```

## Uploading binary using LPC11 embedded CAN bootloader

Drwatson can upload firmware automatically, but if you want to do it yourself, here's how:

```bash
cd drwatson
./lpc11c00_can_bootloader.py can0 firmware.bin
```
