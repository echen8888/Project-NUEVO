#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../../arduino/src/lib/tlvcodec.h"

static int g_callback_count = 0;
static enum DecodeErrorCode g_last_error = NoError;

static void decode_callback(enum DecodeErrorCode *error,
                            const struct FrameHeader *frameHeader,
                            struct TlvHeader *tlvHeaders,
                            uint8_t **tlvData)
{
    g_callback_count++;
    g_last_error = *error;

    if (*error != NoError) {
        return;
    }

    assert(frameHeader != NULL);
    assert(frameHeader->deviceId == 1U);
    assert(frameHeader->numTlvs == 2U);
    assert(frameHeader->numTotalBytes == 21U);

    assert(tlvHeaders[0].tlvType == 0x10U);
    assert(tlvHeaders[0].tlvLen == 4U);
    assert(memcmp(tlvData[0], "\x78\x56\x34\x12", 4U) == 0);

    assert(tlvHeaders[1].tlvType == 0x40U);
    assert(tlvHeaders[1].tlvLen == 1U);
    assert(tlvData[1][0] == 0xABU);
}

int main(void)
{
    struct TlvEncodeDescriptor encoder;
    struct TlvDecodeDescriptor decoder;
    uint32_t value32 = 0x12345678U;
    uint8_t value8 = 0xABU;

    initEncodeDescriptor(&encoder, 128U, 1U, true);
    addTlvPacket(&encoder, 0x10U, 4U, &value32);
    addTlvPacket(&encoder, 0x40U, 1U, &value8);

    int length = wrapupBuffer(&encoder);
    assert(length == 21);
    assert(encoder.buffer[0] == 0xAAU);
    assert(encoder.buffer[1] == 0x55U);
    assert(encoder.buffer[2] == 0x5AU);
    assert(encoder.buffer[3] == 0xA5U);
    assert(encoder.buffer[4] == 21U);
    assert(encoder.buffer[5] == 0U);
    assert(encoder.buffer[8] == 1U);
    assert(encoder.buffer[9] == 0U);
    assert(encoder.buffer[10] == 2U);
    assert(encoder.buffer[11] == TLV_FLAG_CRC16);

    initDecodeDescriptor(&decoder, 128U, true, decode_callback);
    decode(&decoder, encoder.buffer, (size_t)length);
    assert(g_callback_count == 1);
    assert(g_last_error == NoError);

    encoder.buffer[length - 1] ^= 0x01U;
    g_callback_count = 0;
    g_last_error = NoError;
    decode(&decoder, encoder.buffer, (size_t)length);
    assert(g_callback_count == 1);
    assert(g_last_error == CrcError);

    releaseEncodeDescriptor(&encoder);
    releaseDecodeDescriptor(&decoder);

    puts("PASS: c tlv codec");
    return 0;
}
