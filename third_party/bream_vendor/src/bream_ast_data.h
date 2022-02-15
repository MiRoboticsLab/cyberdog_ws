/*******************************************************************************
* Copyright 2019 Broadcom Limited -- http://www.broadcom.com
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
/** @file bream_ast_data.h
*============================================================================*/

#ifndef BREAM_AST_DATA_H
#define BREAM_AST_DATA_H

#include <string.h>

#include "lodi2_os_glue.h"
#include "bream.h"
#include "bream_handler.h"

struct BreamAstData
{
  BreamAstData(const char * path, U2 _clsid)
    : clsid(_clsid), fp(nullptr), size(0), bytesRead(0)
  {
    fp = LD2OS_openFile(path, LD2OS_O_RDONLY);
    if (fp == nullptr) {
      LD2_LOG("%s() failed to open %s\n", __FUNCTION__, path);
    } else {
      LD2OSFileInfo finfo {};
      LD2OS_statFile(path, &finfo);
      size = (int)finfo.fsize;
      strncpy(fname, path, strlen(path));
    }
  }
#ifndef BREAM_V2
  bool CheckCrc(unsigned char * buf, U2 len)
  {
    unsigned char chkA = 0, chkB = 0;
    for (int i = 2; i < (len - 2); i++) {
      chkA = chkA + buf[i];
      chkB = chkB + chkA;
    }
    return chkA == buf[len - 2] && buf[len - 1] == chkB;
  }

  int ReadOnePacket()
  {
    if (fp == nullptr) {
      return 0;
    }
    int offset = 0;
    if (LD2OS_readFile(fp, (void *)buff, 6) <= 0) {
      LD2_LOG(
        "%s() failed to read file %s!\n",
        __FUNCTION__, fname);
      return -1;
    }

    offset += 6;

#define SYNC_CHAR1  0xB5
#define SYNC_CHAR2  0x62
    if (buff[0] != SYNC_CHAR1 || buff[1] != SYNC_CHAR2) {
      LD2_LOG(
        "%s() Sync byte mismatch! 0X%02X, 0x%02X expected, "
        "but 0x%02X, 0x%02X\n", __FUNCTION__, SYNC_CHAR1, SYNC_CHAR2, buff[0], buff[1]);
      return -1;
    }
    U2 len = buff[5] << 8 | buff[4];
    if (LD2OS_readFile(fp, buff + offset, len + 2) <= 0) {
      LD2_LOG(
        "%s() failed to read file %s!\n",
        __FUNCTION__, fname);
      return -1;
    }
    offset += len + 2;
    if (!CheckCrc(buff, offset)) {
      LD2_LOG("%s() CRC failed!\n", __FUNCTION__);
      return -1;
    }
    bytesRead += offset;
    if (bytesRead >= size) {
      LD2OS_closeFile(fp);
      fp = nullptr;
    }
    return offset;
  }
#else
  bool CheckCrc(unsigned char * buf, U2 len)
  {
    uint16_t c = 0;
    for (int i = 4; i < len; i++) {
      c = BreamHandler::DataCRC(c, buf[i]);
    }
    return c == 0;
  }

  int ReadOnePacket()
  {
    if (fp == nullptr) {
      return 0;
    }
    int offset = 0;
    if (LD2OS_readFile(fp, (void *)buff, 4) <= 0) {
      LD2_LOG(
        "%s() failed to read file %s!\n",
        __FUNCTION__, fname);
      return -1;
    }

    offset += 4;

#define SYNC_CHAR1  0xBC
#define SYNC_CHAR2  0xB2
    if (buff[0] != SYNC_CHAR1 || buff[1] != SYNC_CHAR2) {
      LD2_LOG(
        "%s() Sync byte mismatch! 0X%02X, 0x%02X expected, "
        "but 0x%02X, 0x%02X\n", __FUNCTION__, SYNC_CHAR1, SYNC_CHAR2, buff[0], buff[1]);
      return -1;
    }
    U2 len = buff[2] << 3 | buff[3] >> 5;
    uint8_t c = BreamHandler::LengthCRC(len);
    if (c != (buff[3] & 0x1F)) {
      LD2_LOG(
        "%s() data length invalid in file %s!\n",
        __FUNCTION__, fname);
      return -1;
    }

    if (LD2OS_readFile(fp, buff + offset, len + 2) <= 0) {
      LD2_LOG(
        "%s() failed to read file %s!\n",
        __FUNCTION__, fname);
      return -1;
    }
    offset += len + 2;
    if (!CheckCrc(buff, offset)) {
      LD2_LOG("%s() CRC failed!\n", __FUNCTION__);
      return -1;
    }
    bytesRead += offset;
    if (bytesRead >= size) {
      LD2OS_closeFile(fp);
      fp = nullptr;
    }
    return offset;
  }

#endif

  virtual int Send() = 0;
  virtual ~BreamAstData()
  {
    if (fp != nullptr) {
      LD2OS_closeFile(fp);
    }
  }
  U2 clsid;
  void * fp;
  int size;
  int bytesRead;
  char fname[256];
  U1 buff[LD2BRM_MAX_PACKET_SIZE];
};
#endif
