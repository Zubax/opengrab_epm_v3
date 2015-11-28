#
# Copyright (C) 2015 Zubax Robotics, <info@zubax.com>
# Author: Pavel Kirienko <pavel.kirienko@zubax.com>
#
# CAN bus wrapper
#

import socket
import struct
import sys
import select


class TimeoutException(TimeoutError):
    pass


class Bus:
    FORMAT = '=IB3x8s'
    IO_SIZE = 16
    STD_ID_MASK = 0x000007FF
    EXT_ID_MASK = 0x1FFFFFFF
    EXT_ID_FLAG = 1 << 31
    MAX_LEN = 8

    @staticmethod
    def _parse_frame(frame):
        id, can_dlc, data = struct.unpack(Bus.FORMAT, frame)  # @ReservedAssignment
        ext = bool(id & Bus.EXT_ID_FLAG)
        return {
            'id': id & Bus.EXT_ID_MASK,
            'data': data[:can_dlc],
            'ext': ext
        }

    @staticmethod
    def _make_frame(id, data, ext):  # @ReservedAssignment
        if isinstance(data, str):
            data = bytes(data, 'utf8')

        if not isinstance(data, bytes):
            data = bytes(data)

        assert id & (Bus.EXT_ID_MASK if ext else Bus.STD_ID_MASK) == id
        assert len(data) <= Bus.MAX_LEN

        if ext:
            id |= Bus.EXT_ID_FLAG

        can_dlc = len(data)
        data = data.ljust(Bus.MAX_LEN, b'\x00')
        return struct.pack(Bus.FORMAT, id, can_dlc, data)

    def __init__(self, iface_name, default_timeout=None):
        self.default_timeout = default_timeout

        self.socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.socket.bind((iface_name,))

        self.poll = select.poll()
        self.poll.register(self.socket.fileno())

    def _resolve_timeout_ms(self, timeout):
        if timeout is not None:
            return timeout * 1000

        if self.default_timeout is not None:
            return self.default_timeout * 1000

    def receive(self, timeout=None):
        self.poll.modify(self.socket.fileno(), select.POLLIN | select.POLLPRI)
        out = self.poll.poll(self._resolve_timeout_ms(timeout))
        if not len(out):
            raise TimeoutException('CAN bus read timeout')

        frame, (_iface, _id_size) = self.socket.recvfrom(self.IO_SIZE)
        return self._parse_frame(frame)

    def _send_impl(self, encoded_frame, timeout):
        self.poll.modify(self.socket.fileno(), select.POLLOUT)
        out = self.poll.poll(self._resolve_timeout_ms(timeout))
        if not len(out):
            raise TimeoutException('CAN bus write timeout')

        self.socket.send(encoded_frame)

    def send_std(self, id, data, timeout=None):  # @ReservedAssignment
        self._send_impl(self._make_frame(id, data, False), timeout)

    def send_ext(self, id, data, timeout=None):  # @ReservedAssignment
        self._send_impl(self._make_frame(id, data, True), timeout)

    def close(self):
        self.socket.close()
