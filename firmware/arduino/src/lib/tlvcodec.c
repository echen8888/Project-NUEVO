/*
 * Developed by Toby Chen, con mucho amor <3
 * Author Email: pc.toby.chen@gmail.com
 * Update Date: May 20, 2026
 * License: MIT
 */

#include "tlvcodec.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

uint8_t FRAME_HEADER_MAGIC_NUM[FRAME_HEADER_MAGIC_NUM_LEN] = {0xAA, 0x55, 0x5A, 0xA5};

static void writeLe16(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)((value >> 8) & 0xFFU);
}

static uint16_t readLe16(const uint8_t *src)
{
    return (uint16_t)((uint16_t)src[0] | ((uint16_t)src[1] << 8));
}

static void serializeFrameHeader(uint8_t *dst, const struct FrameHeader *header)
{
    memcpy(dst, header->magicNum, FRAME_HEADER_MAGIC_NUM_LEN);
    writeLe16(dst + 4U, header->numTotalBytes);
    writeLe16(dst + 6U, header->checksum);
    dst[8] = header->deviceId;
    dst[9] = header->frameNum;
    dst[10] = header->numTlvs;
    dst[11] = header->flags;
}

static void deserializeFrameHeader(struct FrameHeader *header, const uint8_t *src)
{
    memcpy(header->magicNum, src, FRAME_HEADER_MAGIC_NUM_LEN);
    header->numTotalBytes = readLe16(src + 4U);
    header->checksum = readLe16(src + 6U);
    header->deviceId = src[8];
    header->frameNum = src[9];
    header->numTlvs = src[10];
    header->flags = src[11];
}

static void reportDecodeError(struct TlvDecodeDescriptor *descriptor,
                              enum DecodeErrorCode error)
{
    descriptor->errorCode = error;
    if (descriptor->callback != NULL) {
        descriptor->callback(&descriptor->errorCode, &descriptor->frameHeader, NULL, NULL);
    }
}

void initEncodeDescriptor(struct TlvEncodeDescriptor *descriptor,
                          size_t bufferSize,
                          uint8_t deviceId,
                          bool crc)
{
    descriptor->buffer = (uint8_t *)malloc(bufferSize);
    if (descriptor->buffer == NULL) {
        fprintf(stderr, "Failed to allocate memory for buffer\n");
        exit(EXIT_FAILURE);
    }

    descriptor->bufferSize = bufferSize;
    descriptor->bufferIndex = sizeof(struct FrameHeader);
    descriptor->crc = crc;
    memset(&descriptor->frameHeader, 0, sizeof(descriptor->frameHeader));
    memcpy(descriptor->frameHeader.magicNum, FRAME_HEADER_MAGIC_NUM, FRAME_HEADER_MAGIC_NUM_LEN);
    descriptor->frameHeader.deviceId = deviceId;
    descriptor->frameHeader.flags = crc ? TLV_FLAG_CRC16 : 0U;
}

void releaseEncodeDescriptor(struct TlvEncodeDescriptor *descriptor)
{
    if (descriptor->buffer != NULL) {
        free(descriptor->buffer);
        descriptor->buffer = NULL;
    }
}

void addTlvPacket(struct TlvEncodeDescriptor *descriptor,
                  uint8_t tlvType,
                  uint8_t tlvLen,
                  const void *dataAddr)
{
    if (descriptor->bufferIndex + sizeof(struct TlvHeader) + (size_t)tlvLen > descriptor->bufferSize) {
        fprintf(stderr, "Buffer overflow\n");
        exit(EXIT_FAILURE);
    }

    descriptor->tlvHeader.tlvType = tlvType;
    descriptor->tlvHeader.tlvLen = tlvLen;
    descriptor->buffer[descriptor->bufferIndex++] = descriptor->tlvHeader.tlvType;
    descriptor->buffer[descriptor->bufferIndex++] = descriptor->tlvHeader.tlvLen;

    if (tlvLen > 0U && dataAddr != NULL) {
        memcpy(descriptor->buffer + descriptor->bufferIndex, dataAddr, tlvLen);
        descriptor->bufferIndex += tlvLen;
    }

    descriptor->frameHeader.numTlvs = (uint8_t)(descriptor->frameHeader.numTlvs + 1U);
}

int wrapupBuffer(struct TlvEncodeDescriptor *descriptor)
{
    descriptor->frameHeader.numTotalBytes = (uint16_t)descriptor->bufferIndex;
    descriptor->frameHeader.checksum = 0U;
    descriptor->frameHeader.flags = descriptor->crc ? TLV_FLAG_CRC16 : 0U;

    serializeFrameHeader(descriptor->buffer, &descriptor->frameHeader);

    if (descriptor->crc) {
        descriptor->frameHeader.checksum =
            CRC16(descriptor->buffer + CRC16_BYTES2IGNORE,
                  descriptor->bufferIndex - CRC16_BYTES2IGNORE);
        writeLe16(descriptor->buffer + 6U, descriptor->frameHeader.checksum);
    }

    return (int)descriptor->frameHeader.numTotalBytes;
}

void resetDescriptor(struct TlvEncodeDescriptor *descriptor)
{
    descriptor->bufferIndex = sizeof(struct FrameHeader);
    descriptor->frameHeader.frameNum = (uint8_t)(descriptor->frameHeader.frameNum + 1U);
    descriptor->frameHeader.numTlvs = 0U;
    descriptor->frameHeader.checksum = 0U;
    descriptor->frameHeader.flags = descriptor->crc ? TLV_FLAG_CRC16 : 0U;
}

void initDecodeDescriptor(struct TlvDecodeDescriptor *descriptor,
                          size_t bufferSize,
                          bool crc,
                          void (*callback)(enum DecodeErrorCode *error,
                                           const struct FrameHeader *frameHeader,
                                           struct TlvHeader *tlvHeaders,
                                           uint8_t **tlvData))
{
    size_t tlvSlots = TLVCODEC_TLV_SLOTS_FOR_FRAME_BYTES(bufferSize);

    descriptor->buffer = (uint8_t *)malloc(bufferSize);
    if (descriptor->buffer == NULL) {
        fprintf(stderr, "Failed to allocate memory for buffer\n");
        exit(EXIT_FAILURE);
    }

    descriptor->tlvHeaders = NULL;
    descriptor->tlvData = NULL;
    if (tlvSlots > 0U) {
        descriptor->tlvHeaders = (struct TlvHeader *)malloc(sizeof(struct TlvHeader) * tlvSlots);
        if (descriptor->tlvHeaders == NULL) {
            fprintf(stderr, "Failed to allocate memory for tlv headers\n");
            free(descriptor->buffer);
            exit(EXIT_FAILURE);
        }
        descriptor->tlvData = (uint8_t **)malloc(sizeof(uint8_t *) * tlvSlots);
        if (descriptor->tlvData == NULL) {
            fprintf(stderr, "Failed to allocate memory for tlv data\n");
            free(descriptor->buffer);
            free(descriptor->tlvHeaders);
            exit(EXIT_FAILURE);
        }
    }

    descriptor->bufferSize = bufferSize;
    descriptor->bufferIndex = 0U;
    descriptor->crc = crc;
    descriptor->decodeState = Init;
    descriptor->errorCode = NoError;
    descriptor->ofst = 0U;
    memset(&descriptor->frameHeader, 0, sizeof(descriptor->frameHeader));
    descriptor->callback = callback;
}

void releaseDecodeDescriptor(struct TlvDecodeDescriptor *descriptor)
{
    if (descriptor->buffer != NULL) {
        free(descriptor->buffer);
        descriptor->buffer = NULL;
    }
    if (descriptor->tlvHeaders != NULL) {
        free(descriptor->tlvHeaders);
        descriptor->tlvHeaders = NULL;
    }
    if (descriptor->tlvData != NULL) {
        free(descriptor->tlvData);
        descriptor->tlvData = NULL;
    }
}

void decode(struct TlvDecodeDescriptor *descriptor, const uint8_t *data, size_t dataLen)
{
    for (size_t i = 0; i < dataLen; i++) {
        decodePacket(descriptor, &data[i]);
    }
}

void decodePacket(struct TlvDecodeDescriptor *descriptor, const uint8_t *data)
{
    switch (descriptor->decodeState) {
    case Init:
        if (*data == FRAME_HEADER_MAGIC_NUM[0]) {
            descriptor->bufferIndex = 0U;
            descriptor->buffer[descriptor->bufferIndex++] = *data;
            descriptor->ofst = 1U;
            descriptor->frameHeader.numTotalBytes = 0U;
            descriptor->decodeState = MagicNum;
        }
        break;

    case MagicNum:
        if (*data == FRAME_HEADER_MAGIC_NUM[descriptor->ofst]) {
            descriptor->buffer[descriptor->bufferIndex++] = *data;
            descriptor->ofst++;
            if (descriptor->ofst >= FRAME_HEADER_MAGIC_NUM_LEN) {
                descriptor->ofst = 0U;
                descriptor->frameHeader.numTotalBytes = 0U;
                descriptor->decodeState = TotalPacketLen;
            }
        } else {
            resetDecodeDescriptor(descriptor);
            if (*data == FRAME_HEADER_MAGIC_NUM[0]) {
                descriptor->buffer[descriptor->bufferIndex++] = *data;
                descriptor->ofst = 1U;
                descriptor->decodeState = MagicNum;
            }
        }
        break;

    case TotalPacketLen:
        descriptor->buffer[descriptor->bufferIndex++] = *data;
        descriptor->frameHeader.numTotalBytes |= (uint16_t)((uint16_t)(*data) << (descriptor->ofst * 8U));
        descriptor->ofst++;

        if (descriptor->ofst >= 2U) {
            descriptor->ofst = 0U;
            if (descriptor->frameHeader.numTotalBytes > descriptor->bufferSize ||
                descriptor->frameHeader.numTotalBytes < sizeof(struct FrameHeader) ||
                descriptor->frameHeader.numTotalBytes < descriptor->bufferIndex) {
                reportDecodeError(descriptor, TotalPacketLenError);
                resetDecodeDescriptor(descriptor);
            } else {
                descriptor->decodeState = WaitFullFrame;
            }
        }
        break;

    case WaitFullFrame:
        if (descriptor->bufferIndex >= descriptor->bufferSize) {
            reportDecodeError(descriptor, BufferOutOfIndex);
            resetDecodeDescriptor(descriptor);
            break;
        }

        descriptor->buffer[descriptor->bufferIndex++] = *data;
        if (descriptor->bufferIndex >= descriptor->frameHeader.numTotalBytes) {
            descriptor->ofst = 0U;
            parseFrame(descriptor);
            resetDecodeDescriptor(descriptor);
        }
        break;

    default:
        resetDecodeDescriptor(descriptor);
        break;
    }
}

void parseFrame(struct TlvDecodeDescriptor *descriptor)
{
    size_t packetTlvSlots;
    bool frameHasCrc;

    if (descriptor->bufferIndex < sizeof(struct FrameHeader)) {
        reportDecodeError(descriptor, UnpackFrameHeaderError);
        return;
    }

    deserializeFrameHeader(&descriptor->frameHeader, descriptor->buffer);
    descriptor->ofst = sizeof(struct FrameHeader);

    if (memcmp(descriptor->frameHeader.magicNum, FRAME_HEADER_MAGIC_NUM, FRAME_HEADER_MAGIC_NUM_LEN) != 0) {
        reportDecodeError(descriptor, UnpackFrameHeaderError);
        return;
    }

    if (descriptor->frameHeader.numTotalBytes != descriptor->bufferIndex) {
        reportDecodeError(descriptor, TotalPacketLenError);
        return;
    }

    packetTlvSlots = TLVCODEC_TLV_SLOTS_FOR_FRAME_BYTES(descriptor->frameHeader.numTotalBytes);
    if (descriptor->frameHeader.numTlvs > packetTlvSlots) {
        reportDecodeError(descriptor, TlvError);
        return;
    }

    frameHasCrc = (descriptor->frameHeader.flags & TLV_FLAG_CRC16) != 0U;
    if (descriptor->crc != frameHasCrc) {
        reportDecodeError(descriptor, CrcError);
        return;
    }

    if (frameHasCrc) {
        uint16_t validCrc =
            CRC16(descriptor->buffer + CRC16_BYTES2IGNORE,
                  descriptor->frameHeader.numTotalBytes - CRC16_BYTES2IGNORE);
        if (descriptor->frameHeader.checksum != validCrc) {
            reportDecodeError(descriptor, CrcError);
            return;
        }
    }

    for (uint8_t i = 0; i < descriptor->frameHeader.numTlvs; ++i) {
        if (descriptor->ofst + sizeof(struct TlvHeader) > descriptor->frameHeader.numTotalBytes) {
            reportDecodeError(descriptor, TlvError);
            return;
        }

        descriptor->tlvHeaders[i].tlvType = descriptor->buffer[descriptor->ofst++];
        descriptor->tlvHeaders[i].tlvLen = descriptor->buffer[descriptor->ofst++];

        if (descriptor->ofst + descriptor->tlvHeaders[i].tlvLen > descriptor->frameHeader.numTotalBytes) {
            reportDecodeError(descriptor, TlvLenError);
            return;
        }

        descriptor->tlvData[i] = descriptor->buffer + descriptor->ofst;
        descriptor->ofst += descriptor->tlvHeaders[i].tlvLen;
    }

    if (descriptor->ofst != descriptor->frameHeader.numTotalBytes) {
        reportDecodeError(descriptor, TlvLenError);
        return;
    }

    descriptor->errorCode = NoError;
    if (descriptor->callback != NULL) {
        descriptor->callback(&descriptor->errorCode,
                             &descriptor->frameHeader,
                             descriptor->tlvHeaders,
                             descriptor->tlvData);
    }
}

void resetDecodeDescriptor(struct TlvDecodeDescriptor *descriptor)
{
    descriptor->bufferIndex = 0U;
    descriptor->decodeState = Init;
    descriptor->errorCode = NoError;
    descriptor->ofst = 0U;
    memset(&descriptor->frameHeader, 0, sizeof(descriptor->frameHeader));
}

uint16_t CRC16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU;

    for (size_t i = 0; i < length; ++i) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (uint8_t bit = 0; bit < 8U; ++bit) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}
