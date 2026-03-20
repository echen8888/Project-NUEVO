/*
 * Developed by Toby Chen, con mucho amor <3
 * Author Email: pc.toby.chen@gmail.com
 * Update Date: May 20, 2026
 * License: MIT
 */

#ifndef TLV_CODEC_H
#define TLV_CODEC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FRAME_HEADER_MAGIC_NUM_LEN 4U
#define TLV_FLAG_CRC16 0x01U
#define CRC16_BYTES2IGNORE 8U

#pragma pack(push, 1)

struct FrameHeader
{
    uint8_t magicNum[FRAME_HEADER_MAGIC_NUM_LEN];
    uint16_t numTotalBytes;
    uint16_t checksum;
    uint8_t deviceId;
    uint8_t frameNum;
    uint8_t numTlvs;
    uint8_t flags;
};

struct TlvHeader
{
    uint8_t tlvType;
    uint8_t tlvLen;
};

#pragma pack(pop)

struct TlvEncodeDescriptor
{
    uint8_t *buffer;
    size_t bufferSize;
    size_t bufferIndex;
    bool crc;
    struct FrameHeader frameHeader;
    struct TlvHeader tlvHeader;
};

void initEncodeDescriptor(struct TlvEncodeDescriptor *descriptor,
                          size_t bufferSize,
                          uint8_t deviceId,
                          bool crc);
void releaseEncodeDescriptor(struct TlvEncodeDescriptor *descriptor);
void addTlvPacket(struct TlvEncodeDescriptor *descriptor,
                  uint8_t tlvType,
                  uint8_t tlvLen,
                  const void *dataAddr);
int wrapupBuffer(struct TlvEncodeDescriptor *descriptor);
void resetDescriptor(struct TlvEncodeDescriptor *descriptor);

enum FrameDecodeState
{
    Init = 0,
    MagicNum = 1,
    TotalPacketLen = 2,
    WaitFullFrame = 3
};

enum DecodeErrorCode
{
    NoError = 0,
    CrcError = 1,
    TotalPacketLenError = 2,
    BufferOutOfIndex = 3,
    UnpackFrameHeaderError = 4,
    TlvError = 5,
    TlvLenError = 6
};

struct TlvDecodeDescriptor
{
    uint8_t *buffer;
    size_t bufferSize;
    size_t bufferIndex;
    bool crc;
    enum FrameDecodeState decodeState;
    enum DecodeErrorCode errorCode;
    size_t ofst;
    struct FrameHeader frameHeader;

    void (*callback)(enum DecodeErrorCode *error,
                     const struct FrameHeader *frameHeader,
                     struct TlvHeader *tlvHeaders,
                     uint8_t **tlvData);

    struct TlvHeader *tlvHeaders;
    uint8_t **tlvData;
};

void initDecodeDescriptor(struct TlvDecodeDescriptor *descriptor,
                          size_t bufferSize,
                          bool crc,
                          void (*callback)(enum DecodeErrorCode *error,
                                           const struct FrameHeader *frameHeader,
                                           struct TlvHeader *tlvHeaders,
                                           uint8_t **tlvData));
void releaseDecodeDescriptor(struct TlvDecodeDescriptor *descriptor);
void decode(struct TlvDecodeDescriptor *descriptor, const uint8_t *data, size_t dataLen);
void decodePacket(struct TlvDecodeDescriptor *descriptor, const uint8_t *data);
void parseFrame(struct TlvDecodeDescriptor *descriptor);
void resetDecodeDescriptor(struct TlvDecodeDescriptor *descriptor);

extern uint8_t FRAME_HEADER_MAGIC_NUM[FRAME_HEADER_MAGIC_NUM_LEN];

#define TLVCODEC_TLV_SLOTS_FOR_FRAME_BYTES(frameBytes) \
    (((frameBytes) > sizeof(struct FrameHeader)) ? (((frameBytes) - sizeof(struct FrameHeader)) / sizeof(struct TlvHeader)) : 0U)

uint16_t CRC16(const uint8_t *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif // TLV_CODEC_H
