"""
Developed by Toby Chen, con mucho amor <3
Author Email: pc.toby.chen@gmail.com
Update Date: May 20, 2026
License: MIT
"""

import struct
from enum import Enum

from .utils import (
    CRC16_BYTES2IGNORE,
    FRAME_HEADER_MAGIC_NUM,
    FRAME_HEADER_SIZE,
    TLV_FLAG_CRC16,
    TLV_HEADER_SIZE,
    FrameHeader,
    crc16_ccitt,
)


class FrameDecodeState(Enum):
    Init = 0
    MagicNum = 1
    NumTotalBytes = 2
    WaitFullFrame = 3


class DecodeErrorCode(Enum):
    NoError = 0
    CrcError = 1
    TotalPacketLenError = 2
    BufferOutOfIndex = 3
    UnpackFrameHeaderError = 4
    TlvError = 5
    TlvLenError = 6


class TlvDecodeDescriptor:
    def __init__(self, bufferLen=1024):
        self.decodeState = FrameDecodeState.Init
        self.errorCode = DecodeErrorCode.NoError
        self.ofst = 0
        self.frameHeader = FrameHeader()
        self.buffer = bytearray(bufferLen)
        self.bufferIndex = 0
        self.expectedFrameLength = 0


class Decoder:
    def __init__(self, callback, crc=True, bufferLen=1024):
        self.callback = callback
        self.descriptor = TlvDecodeDescriptor(bufferLen=bufferLen)
        self.crc = bool(crc)

    def decode(self, bytes2decode):
        for packet in bytes(bytes2decode):
            self.decodePacket(packet)

    def decodePacket(self, packet: int):
        packet &= 0xFF
        d = self.descriptor

        if d.decodeState == FrameDecodeState.Init:
            if packet == FRAME_HEADER_MAGIC_NUM[0]:
                d.buffer[0] = packet
                d.bufferIndex = 1
                d.ofst = 1
                d.expectedFrameLength = 0
                d.decodeState = FrameDecodeState.MagicNum
            return

        if d.decodeState == FrameDecodeState.MagicNum:
            if packet == FRAME_HEADER_MAGIC_NUM[d.ofst]:
                d.buffer[d.bufferIndex] = packet
                d.bufferIndex += 1
                d.ofst += 1
                if d.ofst >= len(FRAME_HEADER_MAGIC_NUM):
                    d.ofst = 0
                    d.expectedFrameLength = 0
                    d.decodeState = FrameDecodeState.NumTotalBytes
            else:
                self.resetDescriptor()
                if packet == FRAME_HEADER_MAGIC_NUM[0]:
                    d.buffer[0] = packet
                    d.bufferIndex = 1
                    d.ofst = 1
                    d.decodeState = FrameDecodeState.MagicNum
            return

        if d.decodeState == FrameDecodeState.NumTotalBytes:
            d.buffer[d.bufferIndex] = packet
            d.bufferIndex += 1
            d.expectedFrameLength |= (packet << (8 * d.ofst))
            d.ofst += 1

            if d.ofst >= 2:
                d.ofst = 0
                if (
                    d.expectedFrameLength > len(d.buffer)
                    or d.expectedFrameLength < FRAME_HEADER_SIZE
                    or d.expectedFrameLength < d.bufferIndex
                ):
                    d.errorCode = DecodeErrorCode.TotalPacketLenError
                    self.callback(d.errorCode, d.frameHeader, [])
                    self.resetDescriptor()
                else:
                    d.decodeState = FrameDecodeState.WaitFullFrame
            return

        if d.decodeState == FrameDecodeState.WaitFullFrame:
            if d.bufferIndex >= len(d.buffer):
                d.errorCode = DecodeErrorCode.BufferOutOfIndex
                self.callback(d.errorCode, d.frameHeader, [])
                self.resetDescriptor()
                return

            d.buffer[d.bufferIndex] = packet
            d.bufferIndex += 1
            if d.bufferIndex >= d.expectedFrameLength:
                tlvs = self.parseFrame()
                self.callback(d.errorCode, d.frameHeader, tlvs)
                self.resetDescriptor()

    def parseFrame(self):
        d = self.descriptor
        d.errorCode = DecodeErrorCode.NoError
        frameData = bytes(d.buffer[:d.bufferIndex])
        tlvList = []

        if len(frameData) < FRAME_HEADER_SIZE:
            d.errorCode = DecodeErrorCode.UnpackFrameHeaderError
            return tlvList

        try:
            magicNum, numTotalBytes, checksum, deviceId, frameNum, numTlvs, flags = struct.unpack(
                "<4sHHBBBB", frameData[:FRAME_HEADER_SIZE]
            )
        except struct.error:
            d.errorCode = DecodeErrorCode.UnpackFrameHeaderError
            return tlvList

        if magicNum != FRAME_HEADER_MAGIC_NUM:
            d.errorCode = DecodeErrorCode.UnpackFrameHeaderError
            return tlvList

        d.frameHeader.magicNum[:] = magicNum
        d.frameHeader.numTotalBytes = numTotalBytes
        d.frameHeader.checksum = checksum
        d.frameHeader.deviceId = deviceId
        d.frameHeader.frameNum = frameNum
        d.frameHeader.numTlvs = numTlvs
        d.frameHeader.flags = flags

        if numTotalBytes != len(frameData):
            d.errorCode = DecodeErrorCode.TotalPacketLenError
            return tlvList

        frame_has_crc = bool(flags & TLV_FLAG_CRC16)
        if frame_has_crc != self.crc:
            d.errorCode = DecodeErrorCode.CrcError
            return tlvList

        if frame_has_crc:
            if crc16_ccitt(frameData[CRC16_BYTES2IGNORE:]) != checksum:
                d.errorCode = DecodeErrorCode.CrcError
                return tlvList

        ofst = FRAME_HEADER_SIZE
        for _ in range(numTlvs):
            if ofst + TLV_HEADER_SIZE > len(frameData):
                d.errorCode = DecodeErrorCode.TlvError
                return []

            tlvType, tlvLen = struct.unpack("<BB", frameData[ofst:ofst + TLV_HEADER_SIZE])
            ofst += TLV_HEADER_SIZE

            if ofst + tlvLen > len(frameData):
                d.errorCode = DecodeErrorCode.TlvLenError
                return []

            tlvList.append((tlvType, tlvLen, frameData[ofst:ofst + tlvLen]))
            ofst += tlvLen

        if ofst != len(frameData):
            d.errorCode = DecodeErrorCode.TlvLenError
            return []

        return tlvList

    def resetDescriptor(self):
        d = self.descriptor
        d.decodeState = FrameDecodeState.Init
        d.errorCode = DecodeErrorCode.NoError
        d.ofst = 0
        d.frameHeader = FrameHeader()
        d.bufferIndex = 0
        d.expectedFrameLength = 0
