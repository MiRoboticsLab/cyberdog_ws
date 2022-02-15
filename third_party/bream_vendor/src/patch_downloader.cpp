/*******************************************************************************
* Copyright 2017 Broadcom Limited -- http://www.broadcom.com
* This program is the proprietary software of Broadcom Limited and/or
* its licensors, and may only be used, duplicated, modified or distributed
* pursuant to the terms and conditions of a separate, written license
* agreement executed between you and Broadcom (an "Authorized License").
* Except as set forth in an Authorized License, Broadcom grants no license
* (express or implied), right to use, or waiver of any kind with respect to
* the Software, and Broadcom expressly reserves all rights in and to the
* Software and all intellectual property rights therein. IF YOU HAVE NO
* AUTHORIZED LICENSE, THEN YOU HAVE NO RIGHT TO USE THIS SOFTWARE IN ANY
* WAY, AND SHOULD IMMEDIATELY NOTIFY BROADCOM AND DISCONTINUE ALL USE OF
* THE SOFTWARE.
* ---------------------------------------------------------------------------*/
/** @file patch_donwnloader.cpp
*============================================================================*/

#include "patch_downloader.h"
#include <string.h>
#include <iostream>
#ifdef LD2_NO_FILESYSTEM
// #include "bream_patch_4775B1.h"
#include "bream_patch_4776A0.h"
#endif

// Macros used to decode Bream version
#define MAJOR_VERSION(ver) (((ver) & 0xFFFF0000) >> 16)
#define MINOR_VERSION(ver) ((ver) & 0x0000FFFF)

typedef union BinaryBuffer {
  uint8_t byte;
  uint32_t byte32;
  uint8_t bytes[4];
} BinaryBuffer;

static int uTurnOnCount = 0;
static bool _TurnAsicOn()
{
  if (uTurnOnCount) {
    return true;
  }

  LD2_LOG("Set nStdby on\n");
  // Power off and on
  LD2OS_setGpio(LODI2_GPIO_NSTDBY, 0);
  LD2OS_delay(1);
  LD2OS_setGpio(LODI2_GPIO_NSTDBY, 1);
  LD2OS_delay(15);

  uTurnOnCount++;
  LD2OS_emptySerialBuffer();
  return true;
}

static void _TurnAsicOff(void)
{
  if (!--uTurnOnCount) {
    uTurnOnCount = 0;
    LD2OS_setGpio(LODI2_GPIO_NSTDBY, 0);
    LD2OS_delay(1);
  }
}

void _WriteToAsic(const unsigned char * pucBuf, uint32_t ulLen)
{
  LD2_ASSERT(pucBuf);
  LD2_ASSERT(ulLen > 0);

  LD2OS_writeToSerial(pucBuf, ulLen);
}

static void Progress(unsigned long current, unsigned long total, PatchDownloadProgressCb cb)
{
  static int prevPercent = 0;
  int percent = (int)((current / (float)total) * 100);
  if (prevPercent != percent && percent % 10 == 0) {
    if (cb) {
      cb(percent);
    } else {
      LD2_LOG("%d%% completed\n", percent);
    }
  }
  prevPercent = percent;
}

bool Bream_LoadPatch(const char * patch, PatchDownloadProgressCb cb, char * tty)
{
  static const char aucPatchSignature[12] =
  {0x42, 0x52, 0x45, 0x41, 0x4d, 0x20, 0x50, 0x41, 0x54, 0x43, 0x48, 0x20};
  static const unsigned char BREAM_BLOCK_TYPE_TRANSMIT = 0x01;
  static const unsigned char BREAM_BLOCK_TYPE_RECEIVE = 0x02;
  static const unsigned char BREAM_BLOCK_TYPE_STRING = 0x03;
  static const unsigned char BREAM_BLOCK_TYPE_DELAY = 0x04;

  if (uTurnOnCount) {
    _TurnAsicOff();
  }
  _TurnAsicOn();

  const int MAX_LENGTH = 4096;
  char rxBuffer[MAX_LENGTH];
  uint32_t iRead = 0;

  BinaryBuffer * pBinBuf = NULL;
  unsigned long ulNumberOfDataBlocks = 0;

#ifndef LD2_NO_FILESYSTEM
  char buffer[MAX_LENGTH];
  pBinBuf = (BinaryBuffer *)buffer;
  void * fd = LD2OS_openFile(patch, LD2OS_O_RDONLY);
  if (!fd) {
    LD2_LOG("Fail to open Bream Patch File : %s\n", patch);
    std::cout<<"Fail to open Bream Patch File"<<std::endl;
    return false;
  }
  // 1. read header

  int read = LD2OS_readFile(fd, buffer, 12);
  if (memcmp(buffer, aucPatchSignature, 12) != 0) {

    LD2_LOG("Bream Signature is not matched\n");
    
    LD2OS_closeFile(fd);

    return false;
  }

  read = LD2OS_readFile(fd, (char *)pBinBuf, 4);
  LD2_LOG(
    "Version : %u.%u\n",
    MAJOR_VERSION(pBinBuf->byte32),
    MINOR_VERSION(pBinBuf->byte32));

  read = LD2OS_readFile(fd, (char *)pBinBuf, 4);
  ulNumberOfDataBlocks = pBinBuf->byte32;
  LD2_LOG("number of data blocks : 0x%08x\n", pBinBuf->byte32);
  // 2. read blocks
  for (unsigned long i = 0; i < ulNumberOfDataBlocks; i++) {
    unsigned char ucType = 0;
    uint32_t ulLength = 0;
    unsigned int uiDelay = 0;
    read = LD2OS_readFile(fd, (char *)pBinBuf, 1);

    ucType = pBinBuf->byte;
    read = LD2OS_readFile(fd, (char *)pBinBuf, 3);
    pBinBuf->bytes[3] = 0;
    ulLength = pBinBuf->byte32;
    LD2_ASSERT(ulLength <= MAX_LENGTH);
    unsigned char * p = (unsigned char *)rxBuffer;
    uint32_t remain = 0;

    switch (ucType) {
      case BREAM_BLOCK_TYPE_TRANSMIT:

        read = LD2OS_readFile(fd, (char *)pBinBuf, ulLength);
        _WriteToAsic((const unsigned char *)pBinBuf, ulLength);
        break;
      case BREAM_BLOCK_TYPE_RECEIVE:
        read = LD2OS_readFile(fd, (char *)pBinBuf, ulLength);
        remain = ulLength;
        do {
          iRead = 0;
          LD2OS_readFromSerial(p, &iRead, remain, -1);
          p += iRead;  remain -= iRead;
          LD2_ASSERT((char *)p <= (rxBuffer + sizeof(rxBuffer)));
        } while (remain > 0);

        if (memcmp(pBinBuf, rxBuffer, ulLength) != 0) {
          LD2_LOG("Receive Packet is not matched.\n");
          LD2OS_dump((const uint8_t *)rxBuffer, ulLength);
          LD2OS_closeFile(fd);
          LD2OS_close();
          std::cout<<"over return false "<<std::endl;

          return false;
        }
        break;
      case BREAM_BLOCK_TYPE_STRING:
        read = LD2OS_readFile(fd, (char *)pBinBuf, ulLength);
        pBinBuf->bytes[ulLength] = 0;
        LD2_LOG(" String:[%s]\n", (char *)pBinBuf);
        break;
      case BREAM_BLOCK_TYPE_DELAY:
        LD2_ASSERT(ulLength == 4);
        LD2OS_readFile(fd, (char *)&uiDelay, ulLength);
        LD2_LOG(" Delay:[%d]\n", uiDelay);
        LD2OS_delay(uiDelay);
        break;
      default:
        LD2_ASSERT(0);
    }
    Progress(i, ulNumberOfDataBlocks, cb);
  }
  LD2OS_closeFile(fd);
#else
  LD2_LOG("Read patch from array for no filesystem platform, patch size %d\n", bream_patch_len);

  // 1. read header
  uint32_t patch_read_offset = 0;
  if (memcmp(&bream_patch[patch_read_offset], aucPatchSignature, 12) != 0) {
    LD2_LOG("Bream Signature is not matched\n");
    return false;
  }
  patch_read_offset += 12;

  pBinBuf = (BinaryBuffer *)&bream_patch[patch_read_offset];
  LD2_LOG("version : 0x%08x\n", pBinBuf->byte32);
  patch_read_offset += 4;

  pBinBuf = (BinaryBuffer *)&bream_patch[patch_read_offset];
  ulNumberOfDataBlocks = pBinBuf->byte32;
  LD2_LOG("number of data blocks : 0x%08x\n", pBinBuf->byte32);
  patch_read_offset += 4;

  // 2. read blocks
  for (unsigned long i = 0; i < ulNumberOfDataBlocks; i++) {
    unsigned char ucType = 0;
    unsigned long ulLength = 0;
    unsigned int uiDelay = 0;

    pBinBuf = (BinaryBuffer *)&bream_patch[patch_read_offset];
    ucType = pBinBuf->byte;
    ulLength = (pBinBuf->bytes[1]) + (pBinBuf->bytes[2] << 8) + (pBinBuf->bytes[3] << 16);
    patch_read_offset += 4;

    unsigned char * p = (unsigned char *)rxBuffer;
    unsigned long remain = 0;
    switch (ucType) {
      case BREAM_BLOCK_TYPE_TRANSMIT:
        _WriteToAsic(&bream_patch[patch_read_offset], ulLength);
        patch_read_offset += ulLength;
        break;
      case BREAM_BLOCK_TYPE_RECEIVE:
        remain = ulLength;
        do {
          iRead = 0;
          LD2OS_readFromSerial(p, &iRead, remain, -1);
          p += iRead;  remain -= iRead;
        } while (remain > 0);

        if (memcmp(&bream_patch[patch_read_offset], rxBuffer, ulLength) != 0) {
          LD2_LOG("Receive Packet is not matched.\n");
          LD2OS_dump((const uint8_t *)rxBuffer, ulLength);
          LD2OS_dump((const uint8_t *)&bream_patch[patch_read_offset], ulLength);
          LD2OS_delay(3000);
          return false;
        }
        patch_read_offset += ulLength;
        break;
      case BREAM_BLOCK_TYPE_STRING:
        memcpy(p, &bream_patch[patch_read_offset], ulLength);
        p[ulLength] = 0;
        LD2_LOG(" String:[%s]\n", p);
        patch_read_offset += ulLength;
        break;
      case BREAM_BLOCK_TYPE_DELAY:
        LD2_ASSERT(ulLength == 4);
        pBinBuf = (BinaryBuffer *)&bream_patch[patch_read_offset];
        uiDelay = pBinBuf->bytes[0] + (pBinBuf->bytes[1] << 8) + (pBinBuf->bytes[2] << 16) +
          (pBinBuf->bytes[3] << 27);
        LD2_LOG(" Delay:[%d]\n", uiDelay);
        LD2OS_delay(uiDelay);
        patch_read_offset += ulLength;
        break;
      default:
        LD2_ASSERT(0);
    }
    Progress(i, ulNumberOfDataBlocks, cb);
  }
#endif
  Progress(ulNumberOfDataBlocks, ulNumberOfDataBlocks, cb);
  return true;
}
