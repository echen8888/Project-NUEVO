"""
Developed by Toby Chen, con mucho amor <3
Author Email: pc.toby.chen@gmail.com
Update Date: May 20, 2026
License: MIT
"""

import ctypes
import struct

from .utils import (
    CRC16_BYTES2IGNORE,
    FRAME_HEADER_MAGIC_NUM,
    FRAME_HEADER_SIZE,
    TLV_FLAG_CRC16,
    TLV_HEADER_SIZE,
    crc16_ccitt,
)


class Encoder:
    def __init__(self, deviceId, bufferSize=1024, crc=True):
        if not 0 <= int(deviceId) <= 0xFF:
            raise ValueError("deviceId must fit in uint8")

        self.deviceId = int(deviceId)
        self.bufferSize = bufferSize
        self.buffer = bytearray(bufferSize)
        self.crc = bool(crc)
        self.frameNum = 0
        self.reset()

    def addPacket(self, tlv_type, length, value):
        tlv_type = int(tlv_type)
        length = int(length)

        if not 0 <= tlv_type <= 0xFF:
            raise ValueError("tlv_type must fit in uint8")
        if not 0 <= length <= 0xFF:
            raise ValueError("tlv length must fit in uint8")

        if value is None:
            payload = b""
        elif isinstance(value, (bytes, bytearray, memoryview)):
            payload = bytes(value)
        else:
            payload = ctypes.string_at(ctypes.addressof(value), length)

        if len(payload) != length:
            raise ValueError("payload length does not match tlv length")

        required = TLV_HEADER_SIZE + length
        if self.bufferIndex + required > self.bufferSize:
            raise BufferError("TLV frame buffer overflow")

        self.buffer[self.bufferIndex] = tlv_type
        self.buffer[self.bufferIndex + 1] = length
        self.bufferIndex += TLV_HEADER_SIZE

        if length:
            self.buffer[self.bufferIndex:self.bufferIndex + length] = payload
            self.bufferIndex += length

        self.numTlvs += 1

    def wrapupBuffer(self, deviceId=None, frameNum=None):
        if deviceId is None:
            deviceId = self.deviceId
        deviceId = int(deviceId)
        if not 0 <= deviceId <= 0xFF:
            raise ValueError("deviceId must fit in uint8")

        if frameNum is None:
            frameNum = self.frameNum
        frameNum = int(frameNum) & 0xFF

        total_len = self.bufferIndex
        flags = TLV_FLAG_CRC16 if self.crc else 0

        header = struct.pack(
            "<4sHHBBBB",
            FRAME_HEADER_MAGIC_NUM,
            total_len,
            0,
            deviceId,
            frameNum,
            self.numTlvs,
            flags,
        )
        self.buffer[:FRAME_HEADER_SIZE] = header

        checksum = 0
        if self.crc:
            checksum = crc16_ccitt(self.buffer[CRC16_BYTES2IGNORE:total_len])
            struct.pack_into("<H", self.buffer, 6, checksum)

        self._last_header = {
            "deviceId": deviceId,
            "frameNum": frameNum,
            "numTlvs": self.numTlvs,
            "flags": flags,
            "checksum": checksum,
            "numTotalBytes": total_len,
        }
        return total_len, self.buffer

    def reset(self):
        self.bufferIndex = FRAME_HEADER_SIZE
        self.numTlvs = 0
        self._last_header = None
        self.frameNum = (self.frameNum + 1) & 0xFF
