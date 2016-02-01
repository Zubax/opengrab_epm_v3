#!/usr/bin/env python3
#
# Copyright (C) 2015 Zubax Robotics, <info@zubax.com>
# Author: Pavel Kirienko <pavel.kirienko@zubax.com>
#

from drwatson import init, run, make_api_context_with_user_provided_credentials, execute_shell_command,\
    info, error, input, CLIWaitCursor, download, abort
from drwatson import lpc11c00_can_bootloader as bootloader
from contextlib import closing

PRODUCT_NAME = 'com.zubax.opengrab_epm_v3'
SIGNATURE_OFFSET = 32768 - 128
FIRMWARE_URL = 'https://files.zubax.com/products/com.zubax.opengrab_epm_v3/firmware.bin'


args = init('''OpenGrab EPM v3 production testing application.
This application requires superuser priveleges to function correctly.
Usage instructions:
    1. Connect CAN adapter to this computer.
    2. Start this application and follow its instructions.''',
            lambda p: p.add_argument('iface', help='CAN interface name, e.g. "can0"'),
            lambda p: p.add_argument('--firmware', '-f', help='location of the firmware file', default=FIRMWARE_URL),
            lambda p: p.add_argument('--only-sign', help='skip testing, only install signature',
                                     action='store_true'),
            require_root=True)

api = make_api_context_with_user_provided_credentials()

with CLIWaitCursor():
    firmware_base = download(args.firmware)
    assert 0 < len(firmware_base) <= SIGNATURE_OFFSET, 'Firmware size is incorrect'

    execute_shell_command('ifconfig %s down && ip link set %s up type can bitrate %d sample-point 0.875',
                          args.iface, args.iface, bootloader.CAN_BITRATE, ignore_failure=True)


def load_and_start_firmware(bootloader_interface, firmware_image):
    while True:
        try:
            info('Flashing the firmware [%d bytes]...', len(firmware_image))
            bootloader_interface.unlock()
            bootloader_interface.load_firmware(firmware_image)
        except Exception as ex:
            error('Flashing failed: %r', ex)
            if not input('Try harder?', yes_no=True):
                abort('Flashing failed')
        else:
            input('Set PIO0_1 high (J4 open), then press ENTER')
            info('Starting the firmware...')
            bootloader_interface.reset()
            break


def process_one_device():
    execute_shell_command('ifconfig %s down && ifconfig %s up', args.iface, args.iface)

    if not args.only_sign:
        # Flashing firmware without signature
        with closing(bootloader.BootloaderInterface(args.iface)) as bli:
            input('\n'.join(['1. Set PIO0_3 low, PIO0_1 low (J4 closed, J3 open)',
                             '2. Power on the device and connect it to CAN bus',
                             '3. Press ENTER']))

            with CLIWaitCursor():
                load_and_start_firmware(bli, firmware_base)

        # Testing the device
        input('\n'.join(['1. Make sure that LED indicators are blinking',
                         '2. Test button operation',
                         '3. Press ENTER']))

    # Installing signature
    with closing(bootloader.BootloaderInterface(args.iface)) as bli:
        info("Now we're going to sign the device")
        input('\n'.join(['1. Set PIO0_3 low, PIO0_1 low (J4 closed, J3 open)',
                         '2. Reset the device (e.g. cycle power)',
                         '3. Press ENTER']))

        with CLIWaitCursor():
            info('Reading unique ID...')
            unique_id = bli.read_unique_id()

            info('Requesting signature for unique ID %s', ' '.join(['%02x' % x for x in unique_id]))
            gensign_response = api.generate_signature(unique_id, PRODUCT_NAME)

            info('Signature has been generated successfully [%s], patching the firmware...',
                 ['existing', 'NEW'][gensign_response.new])
            firmware_with_signature = firmware_base.ljust(SIGNATURE_OFFSET, b'\xFF') + gensign_response.signature

            load_and_start_firmware(bli, firmware_with_signature)


run(process_one_device)
