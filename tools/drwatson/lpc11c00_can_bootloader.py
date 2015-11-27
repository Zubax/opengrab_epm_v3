#!/usr/bin/env python3
#
# Copyright (C) 2015 Zubax Robotics, <info@zubax.com>
# Author: Pavel Kirienko <pavel.kirienko@zubax.com>
#

import struct
import sys
import time
import itertools
import logging
import can
import warnings
from contextlib import closing
from enum import IntEnum


logger = logging.getLogger(__name__)


def group_in_chunks(l, n):
    for i in range(0, len(l), n):
        yield l[i:i + n]


def mark_last_iteration(iterable):
    it = iter(iterable)
    last = next(it)
    for val in it:
        yield last, False
        last = val
    yield last, True


class SDOClientCommandSpecifier(IntEnum):
    segment_download = 0
    initiate_download = 1
    initiate_upload = 2
    segment_upload = 3
    abort = 4
    block_upload = 5
    block_download = 6


class SDOServerCommandSpecifier(IntEnum):
    segment_upload = 0
    segment_download = 1
    initiate_upload = 2
    initiate_download = 3
    abort = 4
    block_download = 5
    block_upload = 6


class CANOpenException(Exception):
    pass


class BootloaderInterface:
    TIMEOUT = 1
    LONG_TIMEOUT = 30
    NODE_ID = 0x7D
    TX_OBJ_ID = 0x600
    RX_OBJ_ID = 0x580
    UNLOCK_CODE = 23130
    RAM_BUFFER_BASE_ADDRESS = 0x10000200
    FLASH_SECTOR_SIZE = 4096
    MAX_FLASH_SECTOR_NUM = 7
    FLASH_OFFSET = 0

    def __init__(self, iface_name):
        self.bus = can.Bus(iface_name, self.TIMEOUT)

    @staticmethod
    def _serialize_sdo_payload(payload, format=None):  # @ReservedAssignment
        if payload is None:
            payload = bytes()

        if not isinstance(payload, bytes):
            format = {  # @ReservedAssignment
                'u32': 'I',
                'u16': 'H',
                'u8': 'B',
            }[format]
            payload = struct.pack('<' + format, payload)

        assert len(payload) <= 4
        return payload

    @staticmethod
    def _make_throwing_response_validator(response_command_specifier):
        def validator(f):
            command_byte = f['data'][0]
            command_spec = command_byte >> 5
            if command_spec == SDOServerCommandSpecifier.abort:
                abort_code = struct.unpack('<I', f['data'][4:])[0]
                raise CANOpenException('Abort %08x %r' % (abort_code, f['data']))

            if command_spec != response_command_specifier:
                raise CANOpenException('Unexpected command specifier %d in response %r' % (command_spec, f['data']))

            return True

        return validator

    def _send_and_wait_response(self, payload, response_validator, timeout=None):
        # Sending the request
        can_id = self.TX_OBJ_ID + self.NODE_ID
        self.bus.send_std(can_id, payload)

        # Waiting for the response
        timeout = timeout or self.TIMEOUT
        deadline = time.time() + timeout
        while time.time() < deadline:
            f = self.bus.receive(timeout)
            if f['id'] != self.RX_OBJ_ID + self.NODE_ID:
                continue

            if response_validator(f):
                return f

        raise CANOpenException('Response timeout')

    def _sdo_request(self, index, subindex, command_byte, resp_command_specifier, payload=None, timeout=None):
        payload = payload or bytes()

        logger.debug('SDO request cmd=0x%02x idx=0x%04x subidx=0x%02x payload=%s',
                     command_byte, index, subindex, ','.join('%02x' % x for x in payload))

        # Constructing the payload
        payload = bytes([command_byte, index & 0xFF, index >> 8, subindex]) + payload
        payload = payload.ljust(8, b'\x00')

        def response_validator(f):
            self._make_throwing_response_validator(resp_command_specifier)(f)

            resp_index = f['data'][1] | (f['data'][2] << 8)
            resp_subindex = f['data'][3]
            if (resp_index != index) or (resp_subindex != subindex):
                raise CANOpenException('Unexpected index/subindex in response %r' % f['data'])

            return True

        f = self._send_and_wait_response(payload, response_validator, timeout=timeout)
        resp_command = f['data'][0]
        resp_payload = f['data'][4:]
        logger.debug('SDO response cmd=%02x payload=%s', resp_command, ','.join('%02x' % x for x in resp_payload))
        return resp_command, resp_payload

    def _download_expedited(self, index, subindex, payload=None, payload_format=None, timeout=None):
        payload = self._serialize_sdo_payload(payload, payload_format)
        command_spec = SDOClientCommandSpecifier.initiate_download
        bytes_without_data = 4 - len(payload)
        command_byte = (command_spec << 5) | (bytes_without_data << 2) | 0b11
        response_command_specifier = SDOServerCommandSpecifier.initiate_download
        return self._sdo_request(index, subindex, command_byte, response_command_specifier, payload, timeout=timeout)

    def _upload_expedited(self, index, subindex):
        command_byte = SDOClientCommandSpecifier.initiate_upload << 5
        response_command_specifier = SDOServerCommandSpecifier.initiate_upload
        return self._sdo_request(index, subindex, command_byte, response_command_specifier)

    def _download_segmented(self, index, subindex, data):
        assert isinstance(data, bytes)

        # SDO download initiate
        command_spec = SDOClientCommandSpecifier.initiate_download
        command_byte = (command_spec << 5) | 1                      # With size, 4 bytes

        req_payload = self._serialize_sdo_payload(len(data), 'u32')

        self._sdo_request(index, subindex, command_byte, SDOServerCommandSpecifier.initiate_download, req_payload)

        # Downloading the segments
        toggle = False
        for ch, last in mark_last_iteration(group_in_chunks(data, 7)):
            command_spec = SDOClientCommandSpecifier.segment_download
            n = 7 - len(ch)
            c = last
            command_byte = (command_spec << 5) | (toggle << 4) | (n << 1) | (c << 0)

            seg_payload = bytes([command_byte]) + ch
            seg_payload = seg_payload.ljust(8, b'\x00')

            resp_command_specifier = SDOServerCommandSpecifier.segment_download

            def validator(f):
                self._make_throwing_response_validator(resp_command_specifier)(f)
                if bool(f['data'][0] & (1 << 4)) != toggle:
                    raise Exception('Download toggle mismatch')
                return True

            self._send_and_wait_response(seg_payload, validator)

            toggle = not toggle

    def _upload_segmented(self, index, subindex):
        # SDO upload initiate
        command_spec = SDOClientCommandSpecifier.initiate_upload
        command_byte = (command_spec << 5)

        cmd, payload = self._sdo_request(index, subindex, command_byte, SDOServerCommandSpecifier.initiate_upload)
        # LPC11C00 returns cmd = 64

        # Uploading the segments
        output = bytes()
        toggle = False
        while True:
            command_spec = SDOClientCommandSpecifier.segment_upload
            command_byte = (command_spec << 5) | (toggle << 4)

            seg_payload = bytes([command_byte])
            seg_payload = seg_payload.ljust(8, b'\x00')

            resp_command_specifier = SDOServerCommandSpecifier.segment_upload

            def validator(f):
                self._make_throwing_response_validator(resp_command_specifier)(f)
                if bool(f['data'][0] & (1 << 4)) != toggle:
                    raise Exception('Upload toggle mismatch')
                return True

            f = self._send_and_wait_response(seg_payload, validator)
            toggle = not toggle

            cmd, payload = f['data'][0], f['data'][1:]

            n = (cmd >> 1) & 0b111
            c = bool(cmd & 1)

            output += payload[:(7 - n)]

            if c:
                break

        return output

    def unlock(self):
        self._download_expedited(0x5000, 0, self.UNLOCK_CODE, 'u16')

    def _erase_sectors_and_prepare_for_write(self, start_sector, end_sector):
        logger.debug('Erasing and preparing for write sectors %d to %d', start_sector, end_sector)

        payload = start_sector | (end_sector << 8)
        payload_format = 'u16'
        assert payload <= 0xFFFF

        self._download_expedited(0x5020, 0, payload, payload_format, timeout=self.LONG_TIMEOUT)  # Prepare for write
        self._download_expedited(0x5030, 0, payload, payload_format, timeout=self.LONG_TIMEOUT)  # Erase
        self._download_expedited(0x5040, 1, payload, payload_format, timeout=self.LONG_TIMEOUT)  # Blank check
        self._download_expedited(0x5020, 0, payload, payload_format, timeout=self.LONG_TIMEOUT)  # Prepare for write

    def _load_block(self, offset, data, fill_byte=None):
        raw_size = len(data)
        fill_byte = b'\xFF' if fill_byte is None else fill_byte
        for x in (256, 512, 1024, 4096):
            if raw_size <= x:
                data = data.ljust(x, fill_byte)
                break

        logger.debug('Loading %d (%d) bytes at offset %08x', len(data), raw_size, offset)

        # Setting the RAM write address
        self._download_expedited(0x5015, 0, self.RAM_BUFFER_BASE_ADDRESS, 'u32')

        # Loading into RAM
        self._download_segmented(0x1F50, 1, data)

        # Copying RAM to flash
        self._download_expedited(0x5050, 1, offset, 'u32')                                # Flash address
        self._download_expedited(0x5050, 2, self.RAM_BUFFER_BASE_ADDRESS, 'u32')          # RAM address
        self._download_expedited(0x5050, 3, len(data), 'u16', timeout=self.LONG_TIMEOUT)  # Size, starts the operation

        # Comparing flash with RAM
        self._download_expedited(0x5060, 1, offset, 'u32')                                # Flash address
        self._download_expedited(0x5060, 2, self.RAM_BUFFER_BASE_ADDRESS, 'u32')          # RAM address
        self._download_expedited(0x5060, 3, len(data), 'u16', timeout=self.LONG_TIMEOUT)  # Size, starts the operation

    def read(self, offset, length):
        length = (length + 3) & ~0b11

        if offset < 0x200:
            warnings.warn('Reading memory addresses under 0x200 is probably pointless, '
                          'because it is remapped to the bootloader\'s vector table.')

        self._download_expedited(0x5010, 0, offset, 'u32')
        self._download_expedited(0x5011, 0, length, 'u32')

        return self._upload_segmented(0x1F50, 1)

    def load_firmware(self, firmware_file, offset=None):
        # Loading the image
        if isinstance(firmware_file, str):
            with open(firmware_file, 'rb') as f:
                firmware = f.read()
        elif isinstance(firmware_file, bytes):
            firmware = firmware_file
        else:
            firmware = firmware_file.read()

        # Computing sectors
        offset = offset or self.FLASH_OFFSET
        size = len(firmware)

        start_sector = offset // self.FLASH_SECTOR_SIZE
        end_sector = (offset + size) // self.FLASH_SECTOR_SIZE
        for x in (start_sector, end_sector):
            assert 0 <= x <= self.MAX_FLASH_SECTOR_NUM

        # Patching the firmware - see 26.3.3 of the user manual
        if offset == 0:
            checksum_offset = 0x1C
            pre = firmware[:checksum_offset]
            post = firmware[(checksum_offset + 4):]

            checksum = 0
            for vector in group_in_chunks(pre, 4):
                checksum += struct.unpack('<I', vector)[0]
            checksum = (~checksum + 1) & 0xFFFFFFFF
            print(checksum)

            firmware = pre + struct.pack('<I', checksum) + post

            assert size == len(firmware)

        # Erasing and preparing the sectors for write
        logger.info('Firmware offset %08x, size %d, sectors %d to %d',
                    offset, size, start_sector, end_sector)

        self._erase_sectors_and_prepare_for_write(start_sector, end_sector)

        # Loading the image at the specified offset
        image_offset = 0
        load_offset = offset
        for ch in group_in_chunks(firmware, self.FLASH_SECTOR_SIZE):
            self._load_block(load_offset, ch)
            image_offset += len(ch)
            load_offset += len(ch)

    def read_unique_id(self):
        out = itertools.chain(*[self._upload_expedited(0x5100, x + 1)[1] for x in range(4)])
        return bytes(out)

    def reset(self):
        # See https://www.lpcware.com/content/forum/invoking-can-bootloader-via-iap-on-lpc11cxx-#comment-1119294
        data = bytes([0x01, 0x4a, 0x02, 0x4b, 0xda, 0x60, 0xc0, 0x46, 0x04, 0x00, 0xfa, 0x05, 0x00, 0xed, 0x00, 0xe0])
        data = data.ljust(256, b'\x00')

        address = self.RAM_BUFFER_BASE_ADDRESS

        logger.debug('Loading and executing %d bytes at %08x', len(data), address)

        # Setting the RAM write address
        self._download_expedited(0x5015, 0, address, 'u32')

        # Loading into RAM
        self._download_segmented(0x1F50, 1, data)

        # Executing the code - this will reset the MCU
        self._download_expedited(0x5070, 1, address, 'u32')
        self._download_expedited(0x1F51, 1, 1, 'u8')

    def close(self):
        self.bus.close()


if __name__ == '__main__':
    logging.basicConfig(stream=sys.stderr, level=logging.DEBUG, format='%(asctime)s %(levelname)s: %(message)s')

    if len(sys.argv) <= 1:
        print('Usage:', sys.argv[0], '<iface-name>', '[fw-file-name]', file=sys.stderr)
        exit(1)

    iface_name = sys.argv[1]
    file_name = sys.argv[2] if len(sys.argv) > 2 else None

    with closing(BootloaderInterface(iface_name)) as bli:
        bli.unlock()
        print('Unique ID', list(bli.read_unique_id()))

        if file_name:
            bli.load_firmware(file_name)

        bli.reset()
