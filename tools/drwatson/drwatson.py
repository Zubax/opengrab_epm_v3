#!/usr/bin/env python3
#
# Copyright (C) 2015 Zubax Robotics, <info@zubax.com>
# Author: Pavel Kirienko <pavel.kirienko@zubax.com>
#

import sys
import logging

logging.basicConfig(stream=sys.stderr, level=logging.DEBUG, format='%(asctime)s %(levelname)s %(name)s: %(message)s')
logging.getLogger('urllib3.connectionpool').setLevel('WARNING')

import lpc11c00_can_bootloader
import drwatson_core
from drwatson_core import print_imperative, print_error, print_info, request_input

PRODUCT_NAME = 'com.zubax.opengrab_epm_v3'
SIGNATURE_OFFSET = 32768 - 128

if len(sys.argv) <= 1:
    drwatson_core.fatal('Required arguments: <can-iface>', file=sys.stderr)

drwatson_core.show_color_legend()

firmware_base = drwatson_core.download('https://files.zubax.com/products/com.zubax.opengrab_epm_v3/firmware.bin')
assert len(firmware_base) <= SIGNATURE_OFFSET, 'Firmware too large'

api = drwatson_core.make_api_context_with_user_provided_credentials()

bli = lpc11c00_can_bootloader.BootloaderInterface(sys.argv[1])

def process_one_device():
    request_input('\n'.join(['1. Set PIO0_3 low, PIO0_1 low',
                             '2. Connect the device to CAN bus',
                             '3. Press ENTER']))

    print_info('Reading unique ID...')
    unique_id = bli.read_unique_id()

    print_info('Requesting signature for unique ID %s', ' '.join(['%02x' % x for x in unique_id]))
    signature = api.generate_signature(unique_id, PRODUCT_NAME).signature

    print_info('Signature has been generated successfully [%d bytes], patching the firware...', len(signature))
    firmware_with_signature = firmware_base.ljust(SIGNATURE_OFFSET, b'\xFF') + signature

    print_info('Flashing the firmware [%d bytes]...', len(firmware_with_signature))
    bli.unlock()
    bli.load_firmware(firmware_with_signature)

    request_input('Set PIO0_3 high, PIO0_1 high, then press enter')

    print_info('Starting the firmware...')
    bli.reset()

    request_input('\n'.join(['1. Make sure that LED indicators are blinking',
                             '2. Test button operation',
                             '3. Press ENTER']))

    print_imperative('Done! Go get a cookie.')


drwatson_core.run(process_one_device)
