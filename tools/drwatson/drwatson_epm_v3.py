#!/usr/bin/env python3
#
# Copyright (C) 2015 Zubax Robotics, <info@zubax.com>
# Author: Pavel Kirienko <pavel.kirienko@zubax.com>
#

import drwatson
import lpc11c00_can_bootloader
from drwatson import imperative, info, input, CLIWaitCursor
from contextlib import closing

PRODUCT_NAME = 'com.zubax.opengrab_epm_v3'
SIGNATURE_OFFSET = 32768 - 128


args = drwatson.init('''OpenGrab EPM v3 production testing application.
This application requires superuser priveleges to function correctly.''',
                     dict(dest='iface', help='CAN interface name, e.g. "can0"'),
                     require_root=True)


api = drwatson.make_api_context_with_user_provided_credentials()

with CLIWaitCursor():
    firmware_base = drwatson.download('https://files.zubax.com/products/com.zubax.opengrab_epm_v3/firmware.bin')
    assert len(firmware_base) <= SIGNATURE_OFFSET, 'Firmware too large'

    drwatson.execute_shell_command('ip link set %s up type can bitrate 100000 sample-point 0.875',
                                   args.iface, ignore_failure=True)


def process_one_device():
    drwatson.execute_shell_command('ifconfig %s down && ifconfig %s up', args.iface, args.iface)

    with closing(lpc11c00_can_bootloader.BootloaderInterface(args.iface)) as bli:
        input('\n'.join(['1. Set PIO0_3 low, PIO0_1 low',
                         '2. Connect the device to CAN bus',
                         '3. Press ENTER']))

        with CLIWaitCursor():
            info('Reading unique ID...')
            unique_id = bli.read_unique_id()

            info('Requesting signature for unique ID %s', ' '.join(['%02x' % x for x in unique_id]))
            signature = api.generate_signature(unique_id, PRODUCT_NAME).signature

            info('Signature has been generated successfully [%d bytes], patching the firware...', len(signature))
            firmware_with_signature = firmware_base.ljust(SIGNATURE_OFFSET, b'\xFF') + signature

            info('Flashing the firmware [%d bytes]...', len(firmware_with_signature))
            bli.unlock()
            bli.load_firmware(firmware_with_signature)

        input('Set PIO0_3 high, PIO0_1 high, then press enter')

        info('Starting the firmware...')
        bli.reset()

    input('\n'.join(['1. Make sure that LED indicators are blinking',
                     '2. Test button operation',
                     '3. Press ENTER']))

    imperative('Done! Go get a cookie.')


drwatson.run(process_one_device)
