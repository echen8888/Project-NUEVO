import ctypes
import struct

from tlvcodec import Decoder, DecodeErrorCode, Encoder
from tlvcodec.src.utils import CRC16_BYTES2IGNORE, FRAME_HEADER_MAGIC_NUM, TLV_FLAG_CRC16, crc16_ccitt


def main():
    encoder = Encoder(deviceId=1, bufferSize=128, crc=True)
    encoder.addPacket(0x10, ctypes.sizeof(ctypes.c_uint32), ctypes.c_uint32(0x12345678))
    encoder.addPacket(0x40, ctypes.sizeof(ctypes.c_uint8), ctypes.c_uint8(0xAB))
    length, buffer = encoder.wrapupBuffer()
    frame = bytes(buffer[:length])

    magic, total_len, checksum, device_id, frame_num, num_tlvs, flags = struct.unpack(
        "<4sHHBBBB", frame[:12]
    )
    assert magic == FRAME_HEADER_MAGIC_NUM
    assert total_len == 21
    assert length == 21
    assert checksum == crc16_ccitt(frame[CRC16_BYTES2IGNORE:])
    assert device_id == 1
    assert frame_num == 1
    assert num_tlvs == 2
    assert flags == TLV_FLAG_CRC16

    decoded = []

    def callback(error_code, frame_header, tlvs):
        decoded.append((error_code, frame_header, tlvs))

    decoder = Decoder(callback=callback, crc=True, bufferLen=128)
    decoder.decode(frame)

    assert len(decoded) == 1
    error_code, frame_header, tlvs = decoded[0]
    assert error_code == DecodeErrorCode.NoError
    assert frame_header.numTotalBytes == 21
    assert frame_header.deviceId == 1
    assert frame_header.frameNum == 1
    assert frame_header.numTlvs == 2
    assert tlvs == [
        (0x10, 4, b"\x78\x56\x34\x12"),
        (0x40, 1, b"\xAB"),
    ]

    corrupted = bytearray(frame)
    corrupted[-1] ^= 0x01
    decoded.clear()
    decoder.decode(corrupted)
    assert len(decoded) == 1
    assert decoded[0][0] == DecodeErrorCode.CrcError

    print("PASS: python tlv codec")


if __name__ == "__main__":
    main()
