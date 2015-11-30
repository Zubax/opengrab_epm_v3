#!/usr/bin/env python3
#
# Copyright (C) 2015 Zubax Robotics, <info@zubax.com>
# Author: Pavel Kirienko <pavel.kirienko@zubax.com>
#

from drwatson import init, run, make_api_context_with_user_provided_credentials, execute_shell_command,\
    info, error, input, CLIWaitCursor, download, abort
import lpc11c00_can_bootloader
from contextlib import closing

PRODUCT_NAME = 'com.zubax.opengrab_epm_v3'
SIGNATURE_OFFSET = 32768 - 128


args = init('''OpenGrab EPM v3 production testing application.
This application requires superuser priveleges to function correctly.
Usage instructions:
    1. Connect CAN adapter to this computer.
    2. Start this application and follow its instructions.''',
            dict(dest='iface', help='CAN interface name, e.g. "can0"'),
            require_root=True)


api = make_api_context_with_user_provided_credentials()

with CLIWaitCursor():
    firmware_base = download('https://files.zubax.com/products/com.zubax.opengrab_epm_v3/firmware.bin')
    assert len(firmware_base) <= SIGNATURE_OFFSET, 'Firmware too large'

    execute_shell_command('ifconfig %s down && ip link set %s up type can bitrate %d sample-point 0.875',
                          args.iface, args.iface, lpc11c00_can_bootloader.CAN_BITRATE, ignore_failure=True)


def process_one_device():
    execute_shell_command('ifconfig %s down && ifconfig %s up', args.iface, args.iface)

    with closing(lpc11c00_can_bootloader.BootloaderInterface(args.iface)) as bli:
        input('\n'.join(['1. Set PIO0_3 low, PIO0_1 low (J4 closed, J3 open)',
                         '2. Connect the device to CAN bus',
                         '3. Press ENTER']))

        with CLIWaitCursor():
            info('Reading unique ID...')
            unique_id = bli.read_unique_id()

            info('Requesting signature for unique ID %s', ' '.join(['%02x' % x for x in unique_id]))
            gensign_response = api.generate_signature(unique_id, PRODUCT_NAME)

            info('Signature has been generated successfully [%s], patching the firmware...',
                 ['existing', 'NEW'][gensign_response.new])
            firmware_with_signature = firmware_base.ljust(SIGNATURE_OFFSET, b'\xFF') + gensign_response.signature

            while True:
                try:
                    info('Flashing the firmware [%d bytes]...', len(firmware_with_signature))
                    bli.unlock()
                    bli.load_firmware(firmware_with_signature)
                except Exception as ex:
                    error('Flashing failed: %r', ex)
                    if not input('Try harder?', yes_no=True):
                        abort('Flashing failed')
                else:
                    break

        input('Set PIO0_1 high (J4 open), then press ENTER')

        info('Starting the firmware...')
        bli.reset()

    input('\n'.join(['1. Make sure that LED indicators are blinking',
                     '2. Test button operation',
                     '3. Press ENTER']))


run(process_one_device)
