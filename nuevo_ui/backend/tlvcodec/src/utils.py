import ctypes

__all__ = [
    "FRAME_HEADER_MAGIC_NUM",
    "FRAME_HEADER_SIZE",
    "TLV_HEADER_SIZE",
    "TLV_FLAG_CRC16",
    "CRC16_BYTES2IGNORE",
    "FrameHeader",
    "TlvHeader",
    "crc16_ccitt",
]

FRAME_HEADER_MAGIC_NUM = bytes((0xAA, 0x55, 0x5A, 0xA5))
FRAME_HEADER_SIZE = 12
TLV_HEADER_SIZE = 2
TLV_FLAG_CRC16 = 0x01
CRC16_BYTES2IGNORE = 8


class FrameHeader(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("magicNum", ctypes.c_uint8 * 4),
        ("numTotalBytes", ctypes.c_uint16),
        ("checksum", ctypes.c_uint16),
        ("deviceId", ctypes.c_uint8),
        ("frameNum", ctypes.c_uint8),
        ("numTlvs", ctypes.c_uint8),
        ("flags", ctypes.c_uint8),
    ]


class TlvHeader(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("tlvType", ctypes.c_uint8),
        ("tlvLen", ctypes.c_uint8),
    ]


def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc
