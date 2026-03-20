# TLV Codec (Python)

Python encoder/decoder for the compact NUEVO TLV serial protocol.
Matches the C implementation in `firmware/arduino/src/lib/tlvcodec.c`.

## Wire format

- Frame header: 12 bytes
- TLV header: 2 bytes
- Type IDs: `uint8`
- Length field: `uint8`
- Checksum: CRC16-CCITT
- Multi-TLV frames: supported and expected

## Usage

```python
import ctypes

from tlvcodec import Decoder, Encoder, DecodeErrorCode

encoder = Encoder(deviceId=2, crc=True)
encoder.addPacket(1, 4, (ctypes.c_uint8 * 4)(1, 2, 3, 4))
encoder.addPacket(2, 0, None)
length, buffer = encoder.wrapupBuffer()

def callback(error_code, frame_header, tlv_list):
    if error_code != DecodeErrorCode.NoError:
        return
    for tlv_type, tlv_len, tlv_data in tlv_list:
        handle(tlv_type, tlv_len, tlv_data)

decoder = Decoder(callback=callback, crc=True)
decoder.decode(buffer[:length])
```

## Location

This package lives at `nuevo_ui/backend/tlvcodec/` and is used by `nuevo_bridge`.
