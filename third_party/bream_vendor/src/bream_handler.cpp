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
/** @file bream_handler.cpp
*============================================================================*/

#include <stdio.h>
#include <string.h>
#include "lodi2_os_glue.h"
#include "bream.h"
#include "bream_endian.h"
#include "bream_handler.h"

#ifndef BREAM_V2
#define SYNC_CHAR1  0xB5
#define SYNC_CHAR2  0x62
#define LD2BRM_HEADER_SIZE      4
#else
#define SYNC_CHAR1  0xBC
#define SYNC_CHAR2  0xB2
#define LD2BRM_HEADER_SIZE      2
#endif

#define LD2BRM_PREAMBLE_SIZE    2
#define LD2BRM_CHKSUM_SIZE      2

enum
{
  WAIT_FOR_SYNC_CHAR1 = 0,
  WAIT_FOR_SYNC_CHAR2,
  WAIT_FOR_HEADER,
  WAIT_FOR_MESSAGE_COMPLETE,
};

BreamHandler & BreamHandler::GetInstance(void)
{
  static BreamHandler s_instance;
  return s_instance;
}

BreamHandler::BreamHandler()
: m_cbLD2BRM_ACK_ACK_Output(NULL),
  m_cbLD2BRM_ACK_NACK_Output(NULL),
  m_cbLD2BRM_Stp_ESRC_Get(NULL),
  m_cbLD2BRM_Stp_GNSS_Get(NULL),
  m_cbLD2BRM_Stp_NAVX5_Get(NULL),
  m_cbLD2BRM_StpLteGet(NULL),
  m_cbLD2BRM_StpBiasGet(NULL),
  m_cbLD2BRM_StpLnaGet(NULL),
  m_cbLD2BRM_StpFctTestGet(NULL),
  m_cbLD2BRM_Wng_DEBUG_Output(NULL),
  m_cbD2BRM_WNG_ERROR_Output(NULL),
  m_cbLD2BRM_Wng_NOTICE_Output(NULL),
  m_cbLD2BRM_Wng_TEST_Output(NULL),
  m_cbLD2BRM_Wng_WARNING_Output(NULL),
  m_cbLD2BRM_LOG_BATCH_Polled(NULL),
  m_cbLD2BRM_LOG_FINDTIME_Output(NULL),
  m_cbLD2BRM_LOG_INFO_Output(NULL),
  m_cbLD2BRM_LOG_RETRIEVEPOSEXTRA_Output(NULL),
  m_cbLD2BRM_LOG_RETRIEVEPOS_Output(NULL),
  m_cbLD2BRM_LOG_RETRIEVESTRING_Output(NULL),
  m_cbLD2BRM_Ast_ACK_Output(NULL),
  m_cbLD2BRM_Ast_DBD_Output(NULL),
  m_cbLD2BRM_Sts_BATCH_Polled(NULL),
  m_cbLD2BRM_Sts_GNSS_Polled(NULL),
  m_cbLD2BRM_Sts_HW_Polled(NULL),
  m_cbLD2BRM_Sts_IO_Polled(NULL),
  m_cbLD2BRM_Sts_RXR_Output(NULL),
  m_cbLD2BRM_Sts_VER_Polled(NULL),
  m_cbLD2BRM_Pvt_CLOCK_Polled(NULL),
  m_cbLD2BRM_Pvt_DOP_Polled(NULL),
  m_cbLD2BRM_Pvt_EOE_Periodic(NULL),
  m_cbLD2BRM_Pvt_GEOFENCE_Polled(NULL),
  m_cbLD2BRM_Pvt_ODO_Polled(NULL),
  m_cbLD2BRM_Pvt_ORB_Polled(NULL),
  m_cbLD2BRM_Pvt_PVT_Polled(NULL),
  m_cbLD2BRM_Pvt_SAT_Polled(NULL),
  m_cbLD2BRM_Pvt_STATUS_Polled(NULL),
  m_cbLD2BRM_Pvt_SVINFO_Polled(NULL),
  m_cbLD2BRM_Pvt_VELNED_Polled(NULL),
  m_cbLD2BRM_PvtCbeeStatus(NULL),
  m_cbLD2BRM_AscSubframesPolled(NULL),
  m_cbLD2BRM_AscMeasPolled(NULL),
  m_cbLD2BRM_AscAgcPolled(NULL),
  m_cbLD2BRM_RawData(NULL),
  m_uiParserState(WAIT_FOR_SYNC_CHAR1),
  m_uiNmeaParseState(-1),
  m_uiRxLen(0),
  m_uiPayloadLen(0),
  m_ulPrevRxGarbageBytes(0),
  m_ulRxGarbageBytes(0),
  m_uiRxNmeaLen(0),
  m_uiRxDumpLen(0),
  m_fdDump(NULL),
  m_bPacketDump(false)
{
  m_acRawPacketDumpPath[0] = 0;
}

BreamHandler::~BreamHandler()
{
  if (m_fdDump != NULL) {
    LD2OS_closeFile(m_fdDump);
  }
}

void BreamHandler::ResetState()
{
  m_uiParserState = WAIT_FOR_SYNC_CHAR1;
  m_uiNmeaParseState = -1;
  m_uiRxLen = 0;
  m_uiPayloadLen = 0;
  m_ulPrevRxGarbageBytes = 0;
  m_ulRxGarbageBytes = 0;
  m_uiRxNmeaLen = 0;
  m_uiRxDumpLen = 0;
}

uint16_t BreamHandler::DataCRC(uint16_t c, uint8_t d)
{
  // Polynomial: x^16 +x^8 +x^4 +x^3 +x +1 (0x1011b)
  // (Hamming distance 4 up to 28642 bits, and 6 up to 99 bits).
  uint8_t s;
  s = d ^ (c >> 8);
  c = (c ^ s) << 8;
  uint16_t t = s ^ (s << 1);
  c = c ^ t ^ (t << 3);
  return c;
}

uint8_t BreamHandler::LengthCRC(uint16_t data)
{
  // Polynomial: x^5 +x^3 +x +1   (0x2B)
  // (Hamming distance 4 up to 10 bits)
  const uint8_t table[] =
  {0x00, 0x0B, 0x16, 0x1D, 0x07, 0x0C, 0x11, 0x1A,
    0x0E, 0x05, 0x18, 0x13, 0x09, 0x02, 0x1F, 0x14,
    0x1C, 0x17, 0x0A, 0x01, 0x1B, 0x10, 0x0D, 0x06,
    0x12, 0x19, 0x04, 0x0F, 0x15, 0x1E, 0x03, 0x08};

  uint8_t C = table[(data >> 10) & 0x01];
  C = table[C ^ ((data >> 5) & 0x1F)];
  C = table[C ^ (data & 0x1F)];
  return C;
}

void BreamHandler::DumpRawPacket(uint8_t * buf, uint16_t size, bool bwrite)
{
  if (!m_bPacketDump || size == 0) {return;}

  if (m_fdDump == NULL && m_acRawPacketDumpPath[0] != 0) {
    m_fdDump = LD2OS_openFile(m_acRawPacketDumpPath, LD2OS_O_WRONLY);
    if (m_fdDump == NULL) {
      LD2_LOG("failed to open file %s\n", m_acRawPacketDumpPath);
    }
  }

  const int CHAR_PER_LINE = 32;
  char buff[4096];
  int len = 0;
  if (m_fdDump) {
    len = snprintf(
      buff, sizeof(buff) - 1, "[%8u] %s(%03d): ",
      (unsigned int)LD2OS_getTime(), bwrite ? "T" : "R", size);
  } else {
    len = snprintf(buff, sizeof(buff) - 1, "%s(%03d): ", bwrite ? "T" : "R", size);
  }
  int space = len;
  char cbuf[CHAR_PER_LINE + 2];
  int clen = 0;
  uint16_t i;
  for (i = 0; i < size; i++) {
    if (i > 0 && i % CHAR_PER_LINE == 0) {
      len += snprintf(
        buff + len, sizeof(buff) - len - 1, " %.*s",
        CHAR_PER_LINE, cbuf);
      clen = 0;
      len += snprintf(buff + len, sizeof(buff) - len - 1, "\n%*.s", space, "");
    }
    len += snprintf(buff + len, sizeof(buff) - len - 1, "%02X ", buf[i]);
    if (buf[i] >= 32 && buf[i] <= 126) {
      clen += snprintf(cbuf + clen, sizeof(cbuf) - len - 1, "%c", buf[i]);
    } else {
      clen += snprintf(cbuf + clen, sizeof(cbuf) - len - 1, ".");
    }
  }
  int numLast = i % CHAR_PER_LINE;
  if (numLast < CHAR_PER_LINE) {
    len += snprintf(buff + len, sizeof(buff) - len - 1, " ");
    if (numLast > 0) {
      len += snprintf(
        buff + len, sizeof(buff) - len - 1, "%*.s",
        (CHAR_PER_LINE - numLast) * 3, "");
    }
    len += snprintf(buff + len, sizeof(buff) - len - 1, "%s", cbuf);
  }
  len += snprintf(buff + len, sizeof(buff) - len - 1, "\n");

  if (m_fdDump != NULL) {
    LD2OS_writeFile(m_fdDump, buff, len);
  } else {
    LD2_LOG(buff);
  }
}

void BreamHandler::AsicData(unsigned char * pucData, unsigned short usLen)
{
#if 0
  LD2_LOG("BRM_RX : ");
  LD2_DUMP(pucData, usLen);
#endif

  unsigned short usIdx = 0;
  while (usIdx != usLen) {
    unsigned char ucData = pucData[usIdx++];
    m_aucRxDumpBuffer[m_uiRxDumpLen++] = ucData;
    LD2_ASSERT(m_uiRxDumpLen < LD2BRM_MAX_PACKET_SIZE);
    switch (m_uiParserState) {
      case WAIT_FOR_SYNC_CHAR1:
        {
          if (ucData == SYNC_CHAR1) {
            m_uiNmeaParseState = 0;
            m_uiParserState = WAIT_FOR_SYNC_CHAR2;
          } else if (m_uiNmeaParseState == 0 && ucData == '$') {
            m_uiNmeaParseState = 1;
            m_uiRxNmeaLen = 0;
            m_aucRxNmeaBuf[m_uiRxNmeaLen++] = ucData;
          } else if (m_uiNmeaParseState == 1 && ucData >= 0 && ucData < 128) { // ascii
            m_aucRxNmeaBuf[m_uiRxNmeaLen++] = ucData;
            LD2_ASSERT(m_uiRxNmeaLen < LD2BRM_MAX_PACKET_SIZE);
            if (m_uiRxNmeaLen > 2 && ucData == '\n') {
              if (m_aucRxNmeaBuf[m_uiRxNmeaLen - 2] == '\r') {
                m_aucRxNmeaBuf[m_uiRxNmeaLen] = 0;
                if (m_cbLD2BRM_Nmea) {
                  m_cbLD2BRM_Nmea(m_aucRxNmeaBuf, m_uiRxNmeaLen);
                }
                if (m_bPacketDump) {
                  DumpRawPacket(m_aucRxDumpBuffer, m_uiRxDumpLen, false);
                }
              } else {
                m_ulRxGarbageBytes += m_uiRxNmeaLen;
              }
              m_uiRxDumpLen = 0;
              m_uiRxNmeaLen = 0;
              m_uiNmeaParseState = 0;
            }
          } else {
            m_ulRxGarbageBytes++;
            m_uiNmeaParseState = 0;

            if (m_bPacketDump) {
              DumpRawPacket(m_aucRxDumpBuffer, m_uiRxDumpLen, false);
            }
            m_uiRxDumpLen = 0;
          }
        }
        break;
      case WAIT_FOR_SYNC_CHAR2:
        {
          if (ucData == SYNC_CHAR2) {
            m_uiParserState = WAIT_FOR_HEADER;
          } else {
            m_ulRxGarbageBytes += 2;     //SYNC_CHAR1 + SYNC_CHAR2
            m_uiParserState = WAIT_FOR_SYNC_CHAR1;
          }
        }
        break;
      case WAIT_FOR_HEADER:
        {
          m_aucRxMessageBuf[m_uiRxLen++] = ucData;
          LD2_ASSERT(m_uiRxLen < LD2BRM_MAX_PACKET_SIZE);
          if (m_uiRxLen == LD2BRM_HEADER_SIZE) {
            bool bLengthCheck = true;
#ifndef BREAM_V2
            m_uiPayloadLen = m_aucRxMessageBuf[3] << 8 | m_aucRxMessageBuf[2];
#else
            m_uiPayloadLen = m_aucRxMessageBuf[0] << 3 | m_aucRxMessageBuf[1] >> 5;
            uint8_t c = LengthCRC(m_uiPayloadLen);
            bLengthCheck = c == (m_aucRxMessageBuf[1] & 0x1F);
#endif
            if (m_uiPayloadLen < LD2BRM_MAX_PACKET_SIZE && bLengthCheck) {
              m_uiParserState = WAIT_FOR_MESSAGE_COMPLETE;
            } else {
              //LD2_LOG("Invalid payload length %d! treat as garbage!\n", m_uiPayloadLen);
              m_ulRxGarbageBytes += m_uiRxLen + 2;           //2 sync bytes;
              m_uiParserState = WAIT_FOR_SYNC_CHAR1;
              m_uiRxLen = 0;
            }
          }
        }
        break;
      case WAIT_FOR_MESSAGE_COMPLETE:
        {
          m_aucRxMessageBuf[m_uiRxLen++] = ucData;
          LD2_ASSERT(m_uiRxLen < LD2BRM_MAX_PACKET_SIZE);
          if (m_uiRxLen == (m_uiPayloadLen + LD2BRM_HEADER_SIZE + LD2BRM_CHKSUM_SIZE)) {
            bool bCrcCheck;
            uint8_t group;
            unsigned char * payload;
#ifndef BREAM_V2
            // check crc
            unsigned char chkA = 0, chkB = 0;
            for (uint32_t i = 0; i < (m_uiPayloadLen + LD2BRM_HEADER_SIZE); i++) {
              chkA = chkA + m_aucRxMessageBuf[i];
              chkB = chkB + chkA;
            }
            bCrcCheck = m_aucRxMessageBuf[m_uiPayloadLen + LD2BRM_HEADER_SIZE] == chkA &&
              m_aucRxMessageBuf[m_uiPayloadLen + LD2BRM_HEADER_SIZE + 1] == chkB;
            group = m_aucRxMessageBuf[0];
            uint8_t id = m_aucRxMessageBuf[1];
            payload = &m_aucRxMessageBuf[LD2BRM_HEADER_SIZE];
#else
            uint16_t ck = 0;
            for (uint32_t i = 0; i < (m_uiPayloadLen + LD2BRM_CHKSUM_SIZE); i++) {
              ck = DataCRC(ck, m_aucRxMessageBuf[i + LD2BRM_HEADER_SIZE]);
            }
            bCrcCheck = ck == 0;
            group = m_aucRxMessageBuf[2];
            uint16_t id = m_aucRxMessageBuf[3];
            payload = &m_aucRxMessageBuf[LD2BRM_HEADER_SIZE + 2];
            m_uiPayloadLen -= 2;
#endif
            if (bCrcCheck) {
              // process payload
              if (!m_cbLD2BRM_RawData || !m_cbLD2BRM_RawData(m_aucRxDumpBuffer, m_uiRxDumpLen)) {
                HandlePayload(group, (uint8_t)id, m_uiPayloadLen, payload);
              }
            } else {
              m_ulRxGarbageBytes += m_uiRxLen + 2;           // 2bytes sync
            }
            // reset
            m_uiParserState = WAIT_FOR_SYNC_CHAR1;
            m_uiRxLen = 0;
            m_uiPayloadLen = 0;
            if (m_bPacketDump && m_uiRxDumpLen > 0) {
              DumpRawPacket(m_aucRxDumpBuffer, m_uiRxDumpLen, false);
            }
            m_uiRxDumpLen = 0;
          }
        }
        break;
    }
    if (m_ulPrevRxGarbageBytes != m_ulRxGarbageBytes) {
      //LD2_LOG("Garbage: %u(+%u)\n", m_ulRxGarbageBytes, m_ulRxGarbageBytes-m_ulPrevRxGarbageBytes);
    }
    m_ulPrevRxGarbageBytes = m_ulRxGarbageBytes;
  }
}

void BreamHandler::HandlePayload(
  uint8_t cls, uint8_t id, uint16_t payloadLen,
  unsigned char * pucPayload)
{
  uint16_t clsid = cls << 8 | id;
  //LD2_LOG("RX %s\n", BreamHelper::GetInstance().GetNameOfBreamPacket(cls, id));
  switch (clsid) {
    case BRM_ACK_ACK:
      LD2_ASSERT(payloadLen == 2);
      OnReceive(*(LD2BRM_AckAckOutputPayload *)pucPayload);
      break;
    case BRM_ACK_NAK:
      LD2_ASSERT(payloadLen == 2);
      OnReceive(*(LD2BRM_AckNackOutputPayload *) pucPayload);
      break;
    case BRM_STP_ESRC:
      OnReceive(*(LD2BRM_StpSynchGetPayload *)pucPayload);
      break;
    case BRM_STP_GNSS:
      OnReceive(*(LD2BRM_StpMeSettingsGetPayload *) pucPayload);
      break;
    case BRM_STP_NAVX5:
      OnReceive(*(LD2BRM_StpPeSettingsGetPayload *) pucPayload);
      break;
    case BRM_STP_NAV5:
      OnReceive(*(LD2BRM_StpConfig2CommandPayload *) pucPayload);
      break;
    case BRM_STP_INF:
      OnReceive(*(LD2BRM_StpInfoSetPayload *)pucPayload);
      break;
    case BRM_STP_PRT:
      OnReceive(*(LD2BRM_StpPortPollPayload *)pucPayload);
    case BRM_STP_RATE:
      OnReceive(*(LD2BRM_StpRateSetPayload *)pucPayload);
      break;
    case BRM_STP_MSG:
      OnReceive(*(LD2BRM_StpMessageSetsPayload *)pucPayload);
      break;
    case BRM_STP_PMS:
      OnReceive(*(LD2BRM_StpPwrModeSetPayload *)pucPayload);
      break;
    case BRM_STP_LTE:
      OnReceive(*(LD2BRM_StpLteGetPayload *)pucPayload);
      break;
    case BRM_STP_BIAS:
      OnReceive(*(LD2BRM_StpBiasGetPayload *)pucPayload);
      break;
    case BRM_STP_LNA:
      OnReceive(*(LD2BRM_StpLnaGetPayload *)pucPayload);
      break;
    case BRM_STP_FCTTEST:
      OnReceive(*(LD2BRM_StpFctTestGetPayload *)pucPayload);
      break;
    case BRM_STP_PPS:
      OnReceive(*(LD2BRM_StpPpsPayload *)pucPayload);
      break;
    case BRM_WNG_DEBUG:
      pucPayload[payloadLen] = '\0';
      OnReceive(*(LD2BRM_WngDebugOutputPayload *) pucPayload);
      break;
    case BRM_WNG_ERROR:
      pucPayload[payloadLen] = '\0';
      OnReceive(*(LD2BRM_WngErrorOutputPayload *) pucPayload);
      break;
    case BRM_WNG_NOTICE:
      pucPayload[payloadLen] = '\0';
      OnReceive(*(LD2BRM_WngNoticeOutputPayload *) pucPayload);
      break;
    case BRM_WNG_TEST:
      pucPayload[payloadLen] = '\0';
      OnReceive(*(LD2BRM_WngTestOutputPayload *) pucPayload);
      break;
    case BRM_WNG_WARNING:
      pucPayload[payloadLen] = '\0';
      OnReceive(*(LD2BRM_WngWarningOutputPayload *)pucPayload);
      break;
    case BRM_LOG_BATCH:
      OnReceive(*(LD2BRM_LogBatchPolledPayload *) pucPayload);
      break;
    case BRM_LOG_FINDTIME:
      OnReceive(*(LD2BRM_LogFindtimeOutputPayload *) pucPayload);
      break;
    case BRM_LOG_INFO:
      OnReceive(*(LD2BRM_LogInfoOutputPayload *) pucPayload);
      break;
    case BRM_LOG_RETRIEVEPOSEXTRA:
      OnReceive(*(LD2BRM_LogRetrieveposextraOutputPayload *) pucPayload);
      break;
    case BRM_LOG_RETRIEVEPOS:
      OnReceive(*(LD2BRM_LogRetrieveposOutputPayload *) pucPayload);
      break;
    case BRM_LOG_RETRIEVESTRING:
      OnReceive(*(LD2BRM_LogRetrievestringOutputPayload *) pucPayload);
      break;
    case BRM_AST_ACK_DATA0:
      OnReceive(*(LD2BRM_AstAckOutputPayload *) pucPayload);
      break;
    case BRM_AST_DBD:
      ((LD2BRM_AstNvmemOutputPayload *)pucPayload)->num = payloadLen;
      OnReceive(*(LD2BRM_AstNvmemOutputPayload *)pucPayload);
      break;
    case BRM_STS_BATCH:
      OnReceive(*(LD2BRM_StsBatchPolledPayload *) pucPayload);
      break;
    case BRM_STS_GNSS:
      OnReceive(*(LD2BRM_StsGnssPolledPayload *) pucPayload);
      break;
    case BRM_STS_HW:
      OnReceive(*(LD2BRM_StsHwPolledPayload *) pucPayload);
      break;
    case BRM_STS_IO:
      ((LD2BRM_StsIoPolledPayload *)pucPayload)->num = payloadLen / 20;
      OnReceive(*(LD2BRM_StsIoPolledPayload *) pucPayload);
      break;
    case BRM_STS_RXR:
      OnReceive(*(LD2BRM_StsRxrOutputPayload *)pucPayload);
      break;
    case BRM_STS_VER:
      ((LD2BRM_StsVerPolledPayload *)pucPayload)->num = (payloadLen - 40) / 30;  // 40 for swVersion[10] and hwVersion[30]
      OnReceive(*(LD2BRM_StsVerPolledPayload *) pucPayload);
      break;
    case BRM_PVT_CLOCK:
      OnReceive(*(LD2BRM_PvtClockPolledPayload *)pucPayload);
      break;
    case BRM_PVT_DOP:
      OnReceive(*(LD2BRM_PvtDopPolledPayload *) pucPayload);
      break;
    case BRM_PVT_EOE:
      OnReceive(*(LD2BRM_PvtEoePeriodicPayload *) pucPayload);
      break;
    case BRM_PVT_GEOFENCE:
      OnReceive(*(LD2BRM_PvtGeofencePolledPayload *) pucPayload);
      break;
    case BRM_PVT_ODO:
      OnReceive(*(LD2BRM_PvtAccDistancePolledPayload *) pucPayload);
      break;
    case BRM_PVT_ORB:
      OnReceive(*(LD2BRM_PvtOrbPolledPayload *) pucPayload);
      break;
    case BRM_PVT_PVT:
      OnReceive(*(LD2BRM_PvtPvtPolledPayload *) pucPayload);
      break;
    case BRM_PVT_SAT:
      OnReceive(*(LD2BRM_PvtSatPolledPayload *) pucPayload);
      break;
    case BRM_PVT_STATUS:
      OnReceive(*(LD2BRM_PvtStatusPolledPayload *) pucPayload);
      break;
    case BRM_PVT_SVINFO:
      OnReceive(*(LD2BRM_PvtSvInfoPolledPayload *) pucPayload);
      break;
    case BRM_PVT_VELNED:
      OnReceive(*(LD2BRM_PvtVelNedPolledPayload *) pucPayload);
      break;
    case BRM_PVT_CBEE_STATUS:
      OnReceive(*(LD2BRM_PvtCbeeStatusPayload *)pucPayload);
      break;
    case BRM_ASC_MEAS:
      OnReceive(*(LD2BRM_AscMeasPolledPayload *) pucPayload);
      break;
    case BRM_ASC_SUBFRAMES:
      OnReceive(*(LD2BRM_AscSubframesPolledPayload *) pucPayload);
      break;
    case BRM_ASC_AGC:
      OnReceive(*(LD2BRM_AscAgcPolledPayload *) pucPayload);
      break;
    case BRM_BUP:
      OnReceive(*(LD2BRM_BupPolledPayload *) pucPayload);
      break;
    case BRM_STP_NMEA:
      OnReceive(*(LD2BRM_StpNmeaSetDeprecatedPayload *) pucPayload);
      break;
    default:
      LD2_LOG(
        "Unknown packet with cls(%02X) id(%02X) payloadLength(%d) -- ignored\n", cls, id,
        payloadLen);
  }
}

void BreamHandler::Send(uint16_t payloadLen, unsigned char * pucPayload)
{
  DumpRawPacket(pucPayload, payloadLen, true);
  LD2OS_writeToSerial(pucPayload, payloadLen);
}

void BreamHandler::BuildAndSend(uint16_t clsid, uint16_t payloadLen, unsigned char * pucPayload)
{
  uint8_t buf[LD2BRM_MAX_PACKET_SIZE] = {};

#ifndef BREAM_V2
  LD2_ASSERT(
    payloadLen + LD2BRM_HEADER_SIZE + LD2BRM_CHKSUM_SIZE + LD2BRM_PREAMBLE_SIZE <=
    LD2BRM_MAX_PACKET_SIZE);

  buf[0] = SYNC_CHAR1;
  buf[1] = SYNC_CHAR2;
  buf[2] = (clsid >> 8) & 0xFF;
  buf[3] = clsid & 0xFF;
  buf[4] = payloadLen & 0xFF;
  buf[5] = (payloadLen >> 8) & 0xFF;
  memcpy(&buf[6], pucPayload, payloadLen);

  // check crc
  unsigned char chkA = 0, chkB = 0;
  for (int i = 2; i < (LD2BRM_PREAMBLE_SIZE + LD2BRM_HEADER_SIZE + payloadLen); i++) {
    chkA = chkA + buf[i];
    chkB = chkB + chkA;
    //LD2_LOG("ch=%02X, A=%02X B=%02X\n", buf[i], chkA, chkB);
  }

  buf[LD2BRM_PREAMBLE_SIZE + LD2BRM_HEADER_SIZE + payloadLen] = chkA;
  buf[LD2BRM_PREAMBLE_SIZE + LD2BRM_HEADER_SIZE + payloadLen + 1] = chkB;
#else
  payloadLen += 2;
  LD2_ASSERT(
    payloadLen + LD2BRM_HEADER_SIZE + LD2BRM_CHKSUM_SIZE + LD2BRM_PREAMBLE_SIZE <=
    LD2BRM_MAX_PACKET_SIZE);

  buf[0] = SYNC_CHAR1;
  buf[1] = SYNC_CHAR2;
  uint8_t c = LengthCRC(payloadLen);
  buf[2] = payloadLen >> 3;
  buf[3] = payloadLen << 5 | c;
  buf[4] = (clsid >> 8) & 0xFF;
  buf[5] = clsid & 0xFF;
  memcpy(&buf[6], pucPayload, payloadLen - 2);

  uint16_t c2 = 0;
  for (int i = 4; i < (LD2BRM_PREAMBLE_SIZE + LD2BRM_HEADER_SIZE + payloadLen); i++) {
    c2 = DataCRC(c2, buf[i]);
  }
  buf[LD2BRM_PREAMBLE_SIZE + LD2BRM_HEADER_SIZE + payloadLen] = c2 >> 8;
  buf[LD2BRM_PREAMBLE_SIZE + LD2BRM_HEADER_SIZE + payloadLen + 1] = c2 & 0xFF;

#endif

  Send(LD2BRM_PREAMBLE_SIZE + LD2BRM_HEADER_SIZE + payloadLen + LD2BRM_CHKSUM_SIZE, buf);
}


void BreamHandler::Send(LD2BRM_StpBatchSetPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.bufSize = _LD2_SWAP16(payload.bufSize);
    payload.notifThrs = _LD2_SWAP16(payload.notifThrs);
  }

  BuildAndSend(BRM_STP_BATCH, sizeof(payload), (unsigned char *) &payload);
}

void BreamHandler::Send(LD2BRM_StpConfigCommandPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.clearMask = _LD2_SWAP32(payload.clearMask);
    payload.saveMask = _LD2_SWAP32(payload.saveMask);
    payload.loadMask = _LD2_SWAP32(payload.loadMask);
  }

  BuildAndSend(BRM_STP_CFG, sizeof(payload), (unsigned char *)&payload);
}

void BreamHandler::Send(LD2BRM_StpConfigCommandPayloadWithOption & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.clearMask = _LD2_SWAP32(payload.clearMask);
    payload.saveMask = _LD2_SWAP32(payload.saveMask);
    payload.loadMask = _LD2_SWAP32(payload.loadMask);
  }

  BuildAndSend(BRM_STP_CFG, sizeof(payload), (unsigned char *)&payload);
}

void BreamHandler::Send(LD2BRM_StpGeofenceSetPayload & payload)
{
  LD2_ASSERT(payload.numFences < LD2BRM_MAX_GEOFENCES);

  if (LD2_HOST_BIG_ENDIAN) {
    for (int i = 0; i < payload.numFences; i++) {
      payload.fences[i].lat = _LD2_SWAP32(payload.fences[i].lat);
      payload.fences[i].lon = _LD2_SWAP32(payload.fences[i].lon);
      payload.fences[i].radius = _LD2_SWAP32(payload.fences[i].radius);
    }
  }

  BuildAndSend(BRM_STP_GEOFENCE, (8 + 12 * payload.numFences), (unsigned char *)&payload);
}

void BreamHandler::Send(LD2BRM_StpSynchPollPayload & payload)
{
  BuildAndSend(BRM_STP_ESRC, 0, NULL);
}

void BreamHandler::Send(LD2BRM_StpSynchSetPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    for (int i = 0; i < payload.numSources; i++) {
      payload.sources[i].flags = _LD2_SWAP16(payload.sources[i].flags);
      payload.sources[i].freq = _LD2_SWAP32(payload.sources[i].freq);
      payload.sources[i].withTemp = _LD2_SWAP32(payload.sources[i].withTemp);
      payload.sources[i].withAge = _LD2_SWAP32(payload.sources[i].withAge);
      payload.sources[i].timeToTemp = _LD2_SWAP16(payload.sources[i].timeToTemp);
      payload.sources[i].maxDevLifeTime = _LD2_SWAP16(payload.sources[i].maxDevLifeTime);
      payload.sources[i].offset = _LD2_SWAP32(payload.sources[i].offset);
      payload.sources[i].offsetUncertainty = _LD2_SWAP32(payload.sources[i].offsetUncertainty);
      payload.sources[i].jitter = _LD2_SWAP32(payload.sources[i].jitter);
    }
  }

  BuildAndSend(BRM_STP_ESRC, sizeof(payload), (unsigned char *)&payload);
}

void BreamHandler::Send(LD2BRM_StpMeSettingsPollPayload & payload)
{
  BuildAndSend(BRM_STP_GNSS, 0, (unsigned char *)&payload);
}

void BreamHandler::Send(LD2BRM_StpMeSettingsSetPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    for (int i = 0; i < payload.numConfigBlocks; i++) {
      payload.configBlocks[i].flags = _LD2_SWAP32(payload.configBlocks[i].flags);
    }
  }

  BuildAndSend(
    BRM_STP_GNSS,
    4 + payload.numConfigBlocks * sizeof(LD2BRM_StpMeSettingsConfigBlockType),
    (unsigned char *)&payload);
}

void BreamHandler::Send(LD2BRM_StpInfoPollPayload & payload)
{
  BuildAndSend(BRM_STP_INF, sizeof(payload), (unsigned char *)&payload);
}

void BreamHandler::Send(LD2BRM_StpInfoSetPayload & payload)
{
  BuildAndSend(BRM_STP_INF, sizeof(payload), (unsigned char *)&payload);
}

void BreamHandler::Send(LD2BRM_StpLogfilterSetPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.minInterval = _LD2_SWAP16(payload.minInterval);
    payload.timeThreshold = _LD2_SWAP16(payload.timeThreshold);
    payload.speedThreshold = _LD2_SWAP16(payload.speedThreshold);
    payload.positionThreshold = _LD2_SWAP16(payload.positionThreshold);
  }

  BuildAndSend(BRM_STP_LOGFILTER, sizeof(payload), (unsigned char *)&payload);
}

void BreamHandler::Send(LD2BRM_StpPeSettingsPollPayload & payload)
{
  BuildAndSend(BRM_STP_NAVX5, 0, (unsigned char *)&payload);
}

void BreamHandler::Send(LD2BRM_StpPeSettingsSetPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.version = _LD2_SWAP16(payload.version);
    payload.mask1 = _LD2_SWAP16(payload.mask1);
    payload.mask2 = _LD2_SWAP32(payload.mask2);
    payload.wknRollover = _LD2_SWAP16(payload.wknRollover);
    payload.aopOrbmaxErr = _LD2_SWAP16(payload.aopOrbmaxErr);
  }

  BuildAndSend(BRM_STP_NAVX5, sizeof(payload), (unsigned char *)&payload);
}

void BreamHandler::Send(LD2BRM_StpMessagePollPayload & payload)
{
  BuildAndSend(BRM_STP_MSG, sizeof(payload), (unsigned char *)&payload);
}

void BreamHandler::Send(LD2BRM_StpMessageSetsPayload & payload)
{
  BuildAndSend(BRM_STP_MSG, sizeof(payload), (unsigned char *)&payload);
}

void BreamHandler::Send(LD2BRM_StpMessageSetPayload & payload)
{
  BuildAndSend(BRM_STP_MSG, sizeof(payload), (unsigned char *)&payload);
}

void BreamHandler::Send(LD2BRM_StpNmeaSetDeprecatedPayload & payload)
{
  BuildAndSend(BRM_STP_NMEA, sizeof(payload), (unsigned char *)&payload);
}

void BreamHandler::Send(LD2BRM_StpNmeaSetV0Payload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.gnssToFilter = _LD2_SWAP32(payload.gnssToFilter);
  }

  BuildAndSend(BRM_STP_NMEA, sizeof(payload), (unsigned char *)&payload);
}

void BreamHandler::Send(LD2BRM_StpNmeaSetV1Payload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.gnssToFilter = _LD2_SWAP32(payload.gnssToFilter);
  }

  BuildAndSend(BRM_STP_NMEA, sizeof(payload), (unsigned char *)&payload);
}

void BreamHandler::Send(LD2BRM_StpPwrModeSetPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.period = _LD2_SWAP16(payload.period);
    payload.onTime = _LD2_SWAP16(payload.onTime);
  }

  BuildAndSend(BRM_STP_PMS, sizeof(payload), (unsigned char *)&payload);
}

void BreamHandler::Send(LD2BRM_StpPortPollPayload & payload)
{
  BuildAndSend(BRM_STP_PRT, sizeof(payload), (unsigned char *)&payload);
}

void BreamHandler::Send(LD2BRM_StpPortSetUartPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.txReady = _LD2_SWAP16(payload.txReady);
    payload.mode = _LD2_SWAP32(payload.mode);
    payload.baudRate = _LD2_SWAP32(payload.baudRate);
    payload.inProtoMask = _LD2_SWAP16(payload.inProtoMask);
    payload.outProtoMask = _LD2_SWAP16(payload.outProtoMask);
    payload.flags = _LD2_SWAP16(payload.flags);
  }

  BuildAndSend(BRM_STP_PRT, sizeof(payload), (unsigned char *)&payload);
}

void BreamHandler::Send(LD2BRM_StpPortSetUsbPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.txReady = _LD2_SWAP16(payload.txReady);
    payload.inProtoMask = _LD2_SWAP16(payload.inProtoMask);
    payload.outProtoMask = _LD2_SWAP16(payload.outProtoMask);
  }

  BuildAndSend(BRM_STP_PRT, sizeof(payload), (unsigned char *)&payload);
}

void BreamHandler::Send(LD2BRM_StpPortSetSpiPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.txReady = _LD2_SWAP16(payload.txReady);
    payload.mode = _LD2_SWAP32(payload.mode);
    payload.inProtoMask = _LD2_SWAP16(payload.inProtoMask);
    payload.outProtoMask = _LD2_SWAP16(payload.outProtoMask);
    payload.flags = _LD2_SWAP16(payload.flags);
  }

  BuildAndSend(BRM_STP_PRT, sizeof(payload), (unsigned char *)&payload);
}

void BreamHandler::Send(LD2BRM_StpPortSetDdcPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.txReady = _LD2_SWAP16(payload.txReady);
    payload.mode = _LD2_SWAP32(payload.mode);
    payload.inProtoMask = _LD2_SWAP16(payload.inProtoMask);
    payload.outProtoMask = _LD2_SWAP16(payload.outProtoMask);
    payload.flags = _LD2_SWAP16(payload.flags);
  }

  BuildAndSend(BRM_STP_PRT, sizeof(payload), (unsigned char *)&payload);
}

void BreamHandler::Send(LD2BRM_StpRateSetPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.measRate = _LD2_SWAP16(payload.measRate);
    payload.navRate = _LD2_SWAP16(payload.navRate);
    payload.timeRef = _LD2_SWAP16(payload.timeRef);
  }

  BuildAndSend(BRM_STP_RATE, sizeof(payload), (unsigned char *)&payload);
}

void BreamHandler::Send(LD2BRM_StpResetCommandPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.navBbrMask = _LD2_SWAP16(payload.navBbrMask);
  }

  BuildAndSend(BRM_STP_RST, sizeof(payload), (unsigned char *)&payload);
}

void BreamHandler::Send(LD2BRM_StpRxmSetPayload & payload)
{
  BuildAndSend(BRM_STP_RXM, sizeof(payload), (unsigned char *)&payload);
}

void BreamHandler::Send(LD2BRM_StpLteSetPayload & payload)
{
  BuildAndSend(BRM_STP_LTE, sizeof(payload), (unsigned char *)&payload);
}

void BreamHandler::Send(LD2BRM_StpLtePollPayload & payload)
{
  BuildAndSend(BRM_STP_LTE, 0, NULL);
}

void BreamHandler::Send(LD2BRM_StpBiasSetPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.biasL5 = _LD2_SWAP32(payload.biasL5);
  }

  BuildAndSend(BRM_STP_BIAS, sizeof(payload), (unsigned char *)&payload);
}

void BreamHandler::Send(LD2BRM_StpBiasPollPayload & payload)
{
  BuildAndSend(BRM_STP_BIAS, 0, NULL);
}

void BreamHandler::Send(LD2BRM_StpLnaSetPayload & payload)
{
  BuildAndSend(BRM_STP_LNA, sizeof(payload), (unsigned char *)&payload);
}

void BreamHandler::Send(LD2BRM_StpLnaPollPayload & payload)
{
  BuildAndSend(BRM_STP_LNA, 0, NULL);
}

void BreamHandler::Send(LD2BRM_StpFctTestSetPayload & payload)
{
  BuildAndSend(BRM_STP_FCTTEST, sizeof(payload), (unsigned char *)&payload);
}

void BreamHandler::Send(LD2BRM_StpFctTestPollPayload & payload)
{
  BuildAndSend(BRM_STP_FCTTEST, 0, NULL);
}

void BreamHandler::Send(LD2BRM_StpPpsPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.freqPeriod = _LD2_SWAP32(payload.freqPeriod);
    payload.pulseLenRatio = _LD2_SWAP32(payload.pulseLenRatio);
    payload.flags = _LD2_SWAP32(payload.flags);
  }

  BuildAndSend(BRM_STP_PPS, sizeof(payload), (unsigned char *)&payload);
}

void BreamHandler::Send(LD2BRM_LogCreateCommandPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.userDefinedSize = _LD2_SWAP32(payload.userDefinedSize);
  }

  BuildAndSend(BRM_LOG_CREATE, sizeof(payload), (unsigned char *)&payload);
}

void BreamHandler::Send(LD2BRM_LogEraseCommand & payload)
{
  BuildAndSend(BRM_LOG_ERASE, sizeof(payload), (unsigned char *)&payload);
}

void BreamHandler::Send(LD2BRM_LogFindtimeInputPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.year = _LD2_SWAP16(payload.year);
  }

  BuildAndSend(BRM_LOG_FINDTIME, sizeof(payload), (unsigned char *)&payload);
}

void BreamHandler::Send(LD2BRM_LogInfoPollPayload & payload)
{
  BuildAndSend(BRM_LOG_INFO, 0, NULL);
}

void BreamHandler::Send(LD2BRM_LogRetrievebatchCommandPayload & payload)
{
  BuildAndSend(BRM_LOG_RETRIEVEBATCH, sizeof(payload), (unsigned char *)&payload);
}

void BreamHandler::Send(LD2BRM_LogRetrieveCommandPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.startNumber = _LD2_SWAP32(payload.startNumber);
    payload.entryCount = _LD2_SWAP32(payload.entryCount);
  }

  BuildAndSend(BRM_LOG_RETRIEVE, sizeof(payload), (unsigned char *)&payload);
}

void BreamHandler::Send(LD2BRM_AstLtoInputPayload & payload)
{
  BuildAndSend(BRM_AST_ANO, sizeof(payload), (unsigned char *)&payload);
}

void BreamHandler::Send(LD2BRM_AstNvmemPollPayload & payload)
{
  BuildAndSend(BRM_AST_DBD, 0, NULL);
}

void BreamHandler::Send(LD2BRM_AstNvmemInputPayload & payload)
{
  BuildAndSend(BRM_AST_DBD, sizeof(payload), (unsigned char *)&payload);
}

void BreamHandler::Send(LD2BRM_AstIniPosXyzInputPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.ecefX = _LD2_SWAP32(payload.ecefX);
    payload.ecefY = _LD2_SWAP32(payload.ecefY);
    payload.ecefZ = _LD2_SWAP32(payload.ecefZ);
    payload.posAcc = _LD2_SWAP32(payload.posAcc);
  }

  BuildAndSend(BRM_AST_INI, sizeof(payload), (unsigned char *)&payload);
}

void BreamHandler::Send(LD2BRM_AstIniPosLlhInputPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.lat = _LD2_SWAP32(payload.lat);
    payload.lon = _LD2_SWAP32(payload.lon);
    payload.alt = _LD2_SWAP32(payload.alt);
    payload.posAcc = _LD2_SWAP32(payload.posAcc);
  }

  BuildAndSend(BRM_AST_INI, sizeof(payload), (unsigned char *)&payload);
}

void BreamHandler::Send(LD2BRM_AstRefTimeUtcInputPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.year = _LD2_SWAP16(payload.year);
    payload.ns = _LD2_SWAP32(payload.ns);
    payload.tAccS = _LD2_SWAP16(payload.tAccS);
    payload.tAccNs = _LD2_SWAP32(payload.tAccNs);
  }

  BuildAndSend(BRM_AST_INI, sizeof(payload), (unsigned char *)&payload);
}

void BreamHandler::Send(LD2BRM_AstIniTimeGnssInputPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.week = _LD2_SWAP16(payload.week);
    payload.tow = _LD2_SWAP32(payload.tow);
    payload.ns = _LD2_SWAP32(payload.ns);
    payload.tAccS = _LD2_SWAP16(payload.tAccS);
    payload.tAccNs = _LD2_SWAP32(payload.tAccNs);
  }

  BuildAndSend(BRM_AST_INI, sizeof(payload), (unsigned char *)&payload);
}

void BreamHandler::Send(LD2BRM_AstIniClkdInputPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.clkD = _LD2_SWAP32(payload.clkD);
    payload.clkDAcc = _LD2_SWAP32(payload.clkDAcc);
  }

  BuildAndSend(BRM_AST_INI, sizeof(payload), (unsigned char *)&payload);
}

void BreamHandler::Send(LD2BRM_StsVerPollPayload & payload)
{
  BuildAndSend(BRM_STS_VER, 0, NULL);
}

void BreamHandler::Send(LD2BRM_PvtResetAccDistanceCommandPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
  }
  BuildAndSend(BRM_PVT_RESETODO, 0, NULL);
}

void BreamHandler::Send(LD2BRM_AscPwrCycleCommandPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.duration = _LD2_SWAP32(payload.duration);
    payload.flags = _LD2_SWAP32(payload.flags);
    payload.wakeupSources = _LD2_SWAP32(payload.wakeupSources);
  }
  BuildAndSend(BRM_ASC_PMREQ, sizeof(payload), (unsigned char *)&payload);
}

void BreamHandler::Send(LD2BRM_BupPollPayload & payload)
{
  BuildAndSend(BRM_BUP, 0, NULL);
}

void BreamHandler::Send(LD2BRM_BupCommandPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
  }

  BuildAndSend(BRM_BUP, sizeof(payload), (unsigned char *)&payload);
}

void BreamHandler::Send(LD2BRM_StpConfig2PollPayload & payload)
{
  BuildAndSend(BRM_STP_NAV5, 0, NULL);
}

void BreamHandler::Send(LD2BRM_StpConfig2CommandPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.mask = _LD2_SWAP16(payload.mask);
    payload.pAcc = _LD2_SWAP16(payload.pAcc);
    payload.staticHoldMaxDist = _LD2_SWAP16(payload.staticHoldMaxDist);
  }
  BuildAndSend(BRM_STP_NAV5, sizeof(payload), (unsigned char *)&payload);
}

void BreamHandler::OnReceive(LD2BRM_AckAckOutputPayload & payload)
{
  if (m_cbLD2BRM_ACK_ACK_Output) {
    m_cbLD2BRM_ACK_ACK_Output(payload);
  }
}

void BreamHandler::OnReceive(LD2BRM_AckNackOutputPayload & payload)
{
  if (m_cbLD2BRM_ACK_NACK_Output) {
    m_cbLD2BRM_ACK_NACK_Output(payload);
  }
}

void BreamHandler::OnReceive(LD2BRM_StpSynchGetPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    for (int i = 0; i < payload.numSources; i++) {
      payload.sources[i].flags = _LD2_SWAP16(payload.sources[i].flags);
      payload.sources[i].freq = _LD2_SWAP32(payload.sources[i].freq);
      payload.sources[i].withTemp = _LD2_SWAP32(payload.sources[i].withTemp);
      payload.sources[i].withAge = _LD2_SWAP32(payload.sources[i].withAge);
      payload.sources[i].timeToTemp = _LD2_SWAP16(payload.sources[i].timeToTemp);
      payload.sources[i].maxDevLifeTime = _LD2_SWAP16(payload.sources[i].maxDevLifeTime);
      payload.sources[i].offset = _LD2_SWAP32(payload.sources[i].offset);
      payload.sources[i].offsetUncertainty = _LD2_SWAP32(payload.sources[i].offsetUncertainty);
      payload.sources[i].jitter = _LD2_SWAP32(payload.sources[i].jitter);
    }
  }
  if (m_cbLD2BRM_Stp_ESRC_Get) {
    m_cbLD2BRM_Stp_ESRC_Get(payload);
  }
}

void BreamHandler::OnReceive(LD2BRM_StpBatchSetPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.bufSize = _LD2_SWAP16(payload.bufSize);
    payload.notifThrs = _LD2_SWAP16(payload.notifThrs);
    payload.elapsedFix = _LD2_SWAP16(payload.elapsedFix);
    payload.distanceTraveled = _LD2_SWAP16(payload.distanceTraveled);
  }

  if (m_cbLD2BRM_StpBatchSet) {
    m_cbLD2BRM_StpBatchSet(payload);
  }
}

void BreamHandler::OnReceive(LD2BRM_StpMeSettingsGetPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    for (int i = 0; i < payload.numConfigBlocks; i++) {
      payload.configBlocks[i].flags = _LD2_SWAP32(payload.configBlocks[i].flags);
    }
  }

  if (m_cbLD2BRM_Stp_GNSS_Get) {
    m_cbLD2BRM_Stp_GNSS_Get(payload);
  }
}

void BreamHandler::OnReceive(LD2BRM_StpPeSettingsGetPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.version = _LD2_SWAP16(payload.version);
    payload.mask1 = _LD2_SWAP16(payload.mask1);
    payload.mask2 = _LD2_SWAP32(payload.mask2);
    payload.wknRollover = _LD2_SWAP16(payload.wknRollover);
    payload.aopOrbmaxErr = _LD2_SWAP16(payload.aopOrbmaxErr);
  }
  if (m_cbLD2BRM_Stp_NAVX5_Get) {
    m_cbLD2BRM_Stp_NAVX5_Get(payload);
  }
}
void BreamHandler::OnReceive(LD2BRM_StpPortPollPayload & payload)
{
  if ((payload.PortID == 1 || payload.PortID == 2) && m_cbLD2BRM_StpPortUart_Get) {
    LD2BRM_StpPortSetUartPayload & uart = *(LD2BRM_StpPortSetUartPayload *)&payload;
    if (LD2_HOST_BIG_ENDIAN) {
      uart.txReady = _LD2_SWAP16(uart.txReady);
      uart.mode = _LD2_SWAP32(uart.mode);
      uart.baudRate = _LD2_SWAP32(uart.baudRate);
      uart.inProtoMask = _LD2_SWAP16(uart.inProtoMask);
      uart.outProtoMask = _LD2_SWAP16(uart.outProtoMask);
      uart.flags = _LD2_SWAP16(uart.flags);
    }
    m_cbLD2BRM_StpPortUart_Get(uart);
  }
  if (payload.PortID == 4 && m_cbLD2BRM_StpPortSpi_Get) {
    LD2BRM_StpPortSetSpiPayload & spi = *(LD2BRM_StpPortSetSpiPayload *)&payload;
    if (LD2_HOST_BIG_ENDIAN) {
      spi.txReady = _LD2_SWAP16(spi.txReady);
      spi.mode = _LD2_SWAP32(spi.mode);
      spi.inProtoMask = _LD2_SWAP16(spi.inProtoMask);
      spi.outProtoMask = _LD2_SWAP16(spi.outProtoMask);
      spi.flags = _LD2_SWAP16(spi.flags);
    }
    m_cbLD2BRM_StpPortSpi_Get(spi);
  }
}

void BreamHandler::OnReceive(LD2BRM_StpRateSetPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.measRate = _LD2_SWAP16(payload.measRate);
    payload.navRate = _LD2_SWAP16(payload.navRate);
    payload.timeRef = _LD2_SWAP16(payload.timeRef);
  }

  if (m_cbLD2BRM_StpRateGet) {
    m_cbLD2BRM_StpRateGet(payload);
  }
}

void BreamHandler::OnReceive(LD2BRM_StpInfoSetPayload & payload)
{
  if (m_cbLD2BRM_StpInfoGet) {
    m_cbLD2BRM_StpInfoGet(payload);
  }
}
void BreamHandler::OnReceive(LD2BRM_StpMessageSetsPayload & payload)
{
  if (m_cbLD2BRM_StpMessageGet) {
    m_cbLD2BRM_StpMessageGet(payload);
  }
}

void BreamHandler::OnReceive(LD2BRM_StpPwrModeSetPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.period = _LD2_SWAP16(payload.period);
    payload.onTime = _LD2_SWAP16(payload.onTime);
  }

  if (m_cbLD2BRM_StpPwrModeGet) {
    m_cbLD2BRM_StpPwrModeGet(payload);
  }
}

void BreamHandler::OnReceive(LD2BRM_StpLteGetPayload & payload)
{
  if (m_cbLD2BRM_StpLteGet) {
    m_cbLD2BRM_StpLteGet(payload);
  }
}

void BreamHandler::OnReceive(LD2BRM_StpBiasGetPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.biasL5 = _LD2_SWAP32(payload.biasL5);
  }

  if (m_cbLD2BRM_StpBiasGet) {
    m_cbLD2BRM_StpBiasGet(payload);
  }
}

void BreamHandler::OnReceive(LD2BRM_StpLnaGetPayload & payload)
{
  if (m_cbLD2BRM_StpLnaGet) {
    m_cbLD2BRM_StpLnaGet(payload);
  }
}

void BreamHandler::OnReceive(LD2BRM_StpFctTestGetPayload & payload)
{
  if (m_cbLD2BRM_StpFctTestGet) {
    m_cbLD2BRM_StpFctTestGet(payload);
  }
}

void BreamHandler::OnReceive(LD2BRM_StpPpsPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.freqPeriod = _LD2_SWAP32(payload.freqPeriod);
    payload.pulseLenRatio = _LD2_SWAP32(payload.pulseLenRatio);
    payload.flags = _LD2_SWAP32(payload.flags);
  }
  if (m_cbLD2BRM_StpPpsGet) {
    m_cbLD2BRM_StpPpsGet(payload);
  }
}

void BreamHandler::OnReceive(LD2BRM_WngDebugOutputPayload & payload)
{
  if (m_cbLD2BRM_Wng_DEBUG_Output) {
    m_cbLD2BRM_Wng_DEBUG_Output(payload);
  }
}

void BreamHandler::OnReceive(LD2BRM_WngErrorOutputPayload & payload)
{
  if (m_cbD2BRM_WNG_ERROR_Output) {
    m_cbD2BRM_WNG_ERROR_Output(payload);
  }
}

void BreamHandler::OnReceive(LD2BRM_WngNoticeOutputPayload & payload)
{
  if (m_cbLD2BRM_Wng_NOTICE_Output) {
    m_cbLD2BRM_Wng_NOTICE_Output(payload);
  }
}

void BreamHandler::OnReceive(LD2BRM_WngTestOutputPayload & payload)
{
  if (m_cbLD2BRM_Wng_TEST_Output) {
    m_cbLD2BRM_Wng_TEST_Output(payload);
  }
}

void BreamHandler::OnReceive(LD2BRM_WngWarningOutputPayload & payload)
{
  if (m_cbLD2BRM_Wng_WARNING_Output) {
    m_cbLD2BRM_Wng_WARNING_Output(payload);
  }
}

void BreamHandler::OnReceive(LD2BRM_LogBatchPolledPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.msgCnt = _LD2_SWAP16(payload.msgCnt);
    payload.iTOW = _LD2_SWAP32(payload.iTOW);
    payload.year = _LD2_SWAP16(payload.year);
    payload.tAcc = _LD2_SWAP32(payload.tAcc);
    payload.lon = _LD2_SWAP32(payload.lon);
    payload.lat = _LD2_SWAP32(payload.lat);
    payload.height = _LD2_SWAP32(payload.height);
    payload.hMSL = _LD2_SWAP32(payload.hMSL);
    payload.hAcc = _LD2_SWAP32(payload.hAcc);
    payload.vAcc = _LD2_SWAP32(payload.vAcc);
    payload.velN = _LD2_SWAP32(payload.velN);
    payload.velE = _LD2_SWAP32(payload.velE);
    payload.velD = _LD2_SWAP32(payload.velD);
    payload.gSpeed = _LD2_SWAP32(payload.gSpeed);
    payload.headMot = _LD2_SWAP32(payload.headMot);
    payload.sAcc = _LD2_SWAP32(payload.sAcc);
    payload.headAcc = _LD2_SWAP32(payload.headAcc);
    payload.pDOP = _LD2_SWAP16(payload.pDOP);
    payload.distance = _LD2_SWAP32(payload.distance);
    payload.totalDistance = _LD2_SWAP32(payload.totalDistance);
    payload.distanceStd = _LD2_SWAP32(payload.distanceStd);
  }

  if (m_cbLD2BRM_LOG_BATCH_Polled) {
    m_cbLD2BRM_LOG_BATCH_Polled(payload);
  }
}

void BreamHandler::OnReceive(LD2BRM_LogFindtimeOutputPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.entryNumber = _LD2_SWAP32(payload.entryNumber);
  }

  if (m_cbLD2BRM_LOG_FINDTIME_Output) {
    m_cbLD2BRM_LOG_FINDTIME_Output(payload);
  }
}

void BreamHandler::OnReceive(LD2BRM_LogInfoOutputPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.filestoreCapacity = _LD2_SWAP32(payload.filestoreCapacity);
    payload.currentMaxLogSize = _LD2_SWAP32(payload.currentMaxLogSize);
    payload.currentLogSize = _LD2_SWAP32(payload.currentLogSize);
    payload.entryCount = _LD2_SWAP32(payload.entryCount);
    payload.oldestYear = _LD2_SWAP16(payload.oldestYear);
    payload.newestYear = _LD2_SWAP16(payload.newestYear);
  }

  if (m_cbLD2BRM_LOG_INFO_Output) {
    m_cbLD2BRM_LOG_INFO_Output(payload);
  }
}

void BreamHandler::OnReceive(LD2BRM_LogRetrieveposextraOutputPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.entryIndex = _LD2_SWAP32(payload.entryIndex);
    payload.year = _LD2_SWAP16(payload.year);
    payload.distance = _LD2_SWAP32(payload.distance);
  }

  if (m_cbLD2BRM_LOG_RETRIEVEPOSEXTRA_Output) {
    m_cbLD2BRM_LOG_RETRIEVEPOSEXTRA_Output(payload);
  }
}

void BreamHandler::OnReceive(LD2BRM_LogRetrieveposOutputPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.entryIndex = _LD2_SWAP32(payload.entryIndex);
    payload.lon = _LD2_SWAP32(payload.lon);
    payload.lat = _LD2_SWAP32(payload.lat);
    payload.hMSL = _LD2_SWAP32(payload.hMSL);
    payload.hAcc = _LD2_SWAP32(payload.hAcc);
    payload.gSpeed = _LD2_SWAP32(payload.gSpeed);
    payload.heading = _LD2_SWAP32(payload.heading);
    payload.year = _LD2_SWAP16(payload.year);
  }

  if (m_cbLD2BRM_LOG_RETRIEVEPOS_Output) {
    m_cbLD2BRM_LOG_RETRIEVEPOS_Output(payload);
  }
}

void BreamHandler::OnReceive(LD2BRM_LogRetrievestringOutputPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.entryIndex = _LD2_SWAP32(payload.entryIndex);
    payload.year = _LD2_SWAP16(payload.year);
    payload.byteCount = _LD2_SWAP16(payload.byteCount);
  }

  if (m_cbLD2BRM_LOG_RETRIEVESTRING_Output) {
    m_cbLD2BRM_LOG_RETRIEVESTRING_Output(payload);
  }
}

void BreamHandler::OnReceive(LD2BRM_AstAckOutputPayload & payload)
{
  if (m_cbLD2BRM_Ast_ACK_Output) {
    m_cbLD2BRM_Ast_ACK_Output(payload);
  }
}

void BreamHandler::OnReceive(LD2BRM_AstNvmemOutputPayload & payload)
{
  if (m_cbLD2BRM_Ast_DBD_Output) {
    m_cbLD2BRM_Ast_DBD_Output(payload);
  }
}

void BreamHandler::OnReceive(LD2BRM_StsBatchPolledPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.fillLevel = _LD2_SWAP16(payload.fillLevel);
    payload.dropsAll = _LD2_SWAP16(payload.dropsAll);
    payload.dropsSinceMon = _LD2_SWAP16(payload.dropsSinceMon);
    payload.nextMsgCnt = _LD2_SWAP16(payload.nextMsgCnt);
  }

  if (m_cbLD2BRM_Sts_BATCH_Polled) {
    m_cbLD2BRM_Sts_BATCH_Polled(payload);
  }
}

void BreamHandler::OnReceive(LD2BRM_StsGnssPolledPayload & payload)
{
  if (m_cbLD2BRM_Sts_GNSS_Polled) {
    m_cbLD2BRM_Sts_GNSS_Polled(payload);
  }
}

void BreamHandler::OnReceive(LD2BRM_StsHwPolledPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.pinSel = _LD2_SWAP32(payload.pinSel);
    payload.pinBank = _LD2_SWAP32(payload.pinBank);
    payload.pinDir = _LD2_SWAP32(payload.pinDir);
    payload.pinVal = _LD2_SWAP32(payload.pinVal);
    payload.noisePerMS = _LD2_SWAP16(payload.noisePerMS);
    payload.agcCnt = _LD2_SWAP16(payload.agcCnt);
    payload.usedMask = _LD2_SWAP32(payload.usedMask);
    payload.pinIrq = _LD2_SWAP32(payload.pinIrq);
    payload.pullH = _LD2_SWAP32(payload.pullH);
    payload.pullL = _LD2_SWAP32(payload.pullL);
  }

  if (m_cbLD2BRM_Sts_HW_Polled) {
    m_cbLD2BRM_Sts_HW_Polled(payload);
  }
}

void BreamHandler::OnReceive(LD2BRM_StsIoPolledPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    for (U4 i = 0; i < payload.num; i++) {
      payload.port[i].rxBytes = _LD2_SWAP32(payload.port[i].rxBytes);
      payload.port[i].txBytes = _LD2_SWAP32(payload.port[i].txBytes);
      payload.port[i].parityErrs = _LD2_SWAP16(payload.port[i].parityErrs);
      payload.port[i].framingErrs = _LD2_SWAP16(payload.port[i].framingErrs);
      payload.port[i].overrunErrs = _LD2_SWAP16(payload.port[i].overrunErrs);
      payload.port[i].breakCond = _LD2_SWAP16(payload.port[i].breakCond);
    }
  }

  if (m_cbLD2BRM_Sts_IO_Polled) {
    m_cbLD2BRM_Sts_IO_Polled(payload);
  }
}

void BreamHandler::OnReceive(LD2BRM_StsRxrOutputPayload & payload)
{
  if (m_cbLD2BRM_Sts_RXR_Output) {
    m_cbLD2BRM_Sts_RXR_Output(payload);
  }
}

void BreamHandler::OnReceive(LD2BRM_StsVerPolledPayload & payload)
{
  if (m_cbLD2BRM_Sts_VER_Polled) {
    m_cbLD2BRM_Sts_VER_Polled(payload);
  }
}

void BreamHandler::OnReceive(LD2BRM_PvtClockPolledPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.iTOW = _LD2_SWAP32(payload.iTOW);
    payload.clkB = _LD2_SWAP32(payload.clkB);
    payload.clkD = _LD2_SWAP32(payload.clkD);
    payload.tAcc = _LD2_SWAP32(payload.tAcc);
    payload.fAcc = _LD2_SWAP32(payload.fAcc);
  }

  if (m_cbLD2BRM_Pvt_CLOCK_Polled) {
    m_cbLD2BRM_Pvt_CLOCK_Polled(payload);
  }
}

void BreamHandler::OnReceive(LD2BRM_PvtDopPolledPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.iTOW = _LD2_SWAP32(payload.iTOW);
    payload.gDOP = _LD2_SWAP16(payload.gDOP);
    payload.pDOP = _LD2_SWAP16(payload.pDOP);
    payload.tDOP = _LD2_SWAP16(payload.tDOP);
    payload.vDOP = _LD2_SWAP16(payload.vDOP);
    payload.hDOP = _LD2_SWAP16(payload.hDOP);
    payload.nDOP = _LD2_SWAP16(payload.nDOP);
    payload.eDOP = _LD2_SWAP16(payload.eDOP);
  }

  if (m_cbLD2BRM_Pvt_DOP_Polled) {
    m_cbLD2BRM_Pvt_DOP_Polled(payload);
  }
}

void BreamHandler::OnReceive(LD2BRM_PvtEoePeriodicPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.iTOW = _LD2_SWAP32(payload.iTOW);
  }

  if (m_cbLD2BRM_Pvt_EOE_Periodic) {
    m_cbLD2BRM_Pvt_EOE_Periodic(payload);
  }
}

void BreamHandler::OnReceive(LD2BRM_PvtGeofencePolledPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.iTOW = _LD2_SWAP32(payload.iTOW);
  }

  if (m_cbLD2BRM_Pvt_GEOFENCE_Polled) {
    m_cbLD2BRM_Pvt_GEOFENCE_Polled(payload);
  }
}

void BreamHandler::OnReceive(LD2BRM_PvtAccDistancePolledPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.iTOW = _LD2_SWAP32(payload.iTOW);
    payload.distance = _LD2_SWAP32(payload.distance);
    payload.totalDistance = _LD2_SWAP32(payload.totalDistance);
    payload.distanceStd = _LD2_SWAP32(payload.distanceStd);
  }

  if (m_cbLD2BRM_Pvt_ODO_Polled) {
    m_cbLD2BRM_Pvt_ODO_Polled(payload);
  }
}

void BreamHandler::OnReceive(LD2BRM_PvtOrbPolledPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.iTOW = _LD2_SWAP32(payload.iTOW);
  }

  if (m_cbLD2BRM_Pvt_ORB_Polled) {
    m_cbLD2BRM_Pvt_ORB_Polled(payload);
  }
}

void BreamHandler::OnReceive(LD2BRM_PvtPvtPolledPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.iTOW = _LD2_SWAP32(payload.iTOW);
    payload.year = _LD2_SWAP16(payload.year);
    payload.tAcc = _LD2_SWAP32(payload.tAcc);
    payload.nano = _LD2_SWAP32(payload.nano);
    payload.lon = _LD2_SWAP32(payload.lon);
    payload.lat = _LD2_SWAP32(payload.lat);
    payload.height = _LD2_SWAP32(payload.height);
    payload.hMSL = _LD2_SWAP32(payload.hMSL);
    payload.hAcc = _LD2_SWAP32(payload.hAcc);
    payload.vAcc = _LD2_SWAP32(payload.vAcc);
    payload.velN = _LD2_SWAP32(payload.velN);
    payload.velE = _LD2_SWAP32(payload.velE);
    payload.velD = _LD2_SWAP32(payload.velD);
    payload.gSpeed = _LD2_SWAP32(payload.gSpeed);
    payload.headMot = _LD2_SWAP32(payload.headMot);
    payload.sAcc = _LD2_SWAP32(payload.sAcc);
    payload.headAcc = _LD2_SWAP32(payload.headAcc);
    payload.pDOP = _LD2_SWAP16(payload.pDOP);
    payload.headVeh = _LD2_SWAP32(payload.headVeh);
    payload.magDec = _LD2_SWAP16(payload.magDec);
    payload.magAcc = _LD2_SWAP16(payload.magAcc);
  }

  if (m_cbLD2BRM_Pvt_PVT_Polled) {
    m_cbLD2BRM_Pvt_PVT_Polled(payload);
  }
}

void BreamHandler::OnReceive(LD2BRM_PvtSatPolledPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.iTOW = _LD2_SWAP32(payload.iTOW);
    for (int i = 0; i < payload.numSvs; i++) {
      payload.Svs[i].azim = _LD2_SWAP16(payload.Svs[i].azim);
      payload.Svs[i].prRes = _LD2_SWAP16(payload.Svs[i].prRes);
      payload.Svs[i].flags = _LD2_SWAP32(payload.Svs[i].flags);
    }
  }

  if (m_cbLD2BRM_Pvt_SAT_Polled) {
    m_cbLD2BRM_Pvt_SAT_Polled(payload);
  }
}

void BreamHandler::OnReceive(LD2BRM_PvtStatusPolledPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.iTOW = _LD2_SWAP32(payload.iTOW);
    payload.ttff = _LD2_SWAP32(payload.ttff);
    payload.msss = _LD2_SWAP32(payload.msss);
  }

  if (m_cbLD2BRM_Pvt_STATUS_Polled) {
    m_cbLD2BRM_Pvt_STATUS_Polled(payload);
  }
}

void BreamHandler::OnReceive(LD2BRM_PvtSvInfoPolledPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.iTOW = _LD2_SWAP32(payload.iTOW);
    for (int i = 0; i < payload.numCh; i++) {
      payload.SvInfo[i].azim = _LD2_SWAP16(payload.SvInfo[i].azim);
      payload.SvInfo[i].prRes = _LD2_SWAP32(payload.SvInfo[i].prRes);
    }
  }

  if (m_cbLD2BRM_Pvt_SVINFO_Polled) {
    m_cbLD2BRM_Pvt_SVINFO_Polled(payload);
  }
}

void BreamHandler::OnReceive(LD2BRM_PvtVelNedPolledPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.iTOW = _LD2_SWAP32(payload.iTOW);
    payload.velN = _LD2_SWAP32(payload.velN);
    payload.velE = _LD2_SWAP32(payload.velE);
    payload.velD = _LD2_SWAP32(payload.velD);
    payload.speed = _LD2_SWAP32(payload.speed);
    payload.gSpeed = _LD2_SWAP32(payload.gSpeed);
    payload.heading = _LD2_SWAP32(payload.heading);
    payload.sAcc = _LD2_SWAP32(payload.sAcc);
    payload.cAcc = _LD2_SWAP32(payload.cAcc);
  }

  if (m_cbLD2BRM_Pvt_VELNED_Polled) {
    m_cbLD2BRM_Pvt_VELNED_Polled(payload);
  }
}

void BreamHandler::OnReceive(LD2BRM_PvtCbeeStatusPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.iTOW = _LD2_SWAP32(payload.iTOW);
  }

  if (m_cbLD2BRM_PvtCbeeStatus) {
    m_cbLD2BRM_PvtCbeeStatus(payload);
  }
}

void BreamHandler::OnReceive(LD2BRM_AscMeasPolledPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.rcvTow = _LD2_SwapDouble(payload.rcvTow);
    payload.week = _LD2_SWAP16(payload.week);
    for (int i = 0; i < payload.numMeas; i++) {
      payload.meas[i].prMes = _LD2_SwapDouble(payload.meas[i].prMes);
      payload.meas[i].cpMes = _LD2_SwapDouble(payload.meas[i].cpMes);
      payload.meas[i].doMes = _LD2_SwapFloat(payload.meas[i].doMes);
      payload.meas[i].locktime = _LD2_SWAP16(payload.meas[i].locktime);
    }
  }

  if (m_cbLD2BRM_AscMeasPolled) {
    m_cbLD2BRM_AscMeasPolled(payload);
  }
}

void BreamHandler::OnReceive(LD2BRM_AscSubframesPolledPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    for (int i = 0; i < payload.numWords; i++) {
      payload.dataWords[i] = _LD2_SWAP32(payload.dataWords[i]);
    }
  }

  if (m_cbLD2BRM_AscSubframesPolled) {
    m_cbLD2BRM_AscSubframesPolled(payload);
  }
}

void BreamHandler::OnReceive(LD2BRM_AscAgcPolledPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.iTow = _LD2_SWAP32(payload.iTow);

    for (int i = 0; i < payload.numMeas; i++) {
      payload.contentBlocks[i].AGCdB = _LD2_SWAP16(payload.contentBlocks[i].AGCdB);
    }
  }
  if (m_cbLD2BRM_AscAgcPolled) {
    m_cbLD2BRM_AscAgcPolled(payload);
  }
}

void BreamHandler::OnReceive(LD2BRM_BupPolledPayload & payload)
{
  if (m_cbLD2BRM_BupPolled) {
    m_cbLD2BRM_BupPolled(payload);
  }
}

void BreamHandler::OnReceive(LD2BRM_StpConfig2CommandPayload & payload)
{
  if (LD2_HOST_BIG_ENDIAN) {
    payload.mask = _LD2_SWAP16(payload.mask);
    payload.pAcc = _LD2_SWAP16(payload.pAcc);
    payload.staticHoldMaxDist = _LD2_SWAP16(payload.staticHoldMaxDist);
  }

  if (m_cbLD2BRM_Stp_NAV5_Get) {
    m_cbLD2BRM_Stp_NAV5_Get(payload);
  }
}

void BreamHandler::OnReceive(LD2BRM_StpNmeaSetDeprecatedPayload & payload)
{
  if (m_cbLD2BRM_StpNmeaSetDeprecated) {
    m_cbLD2BRM_StpNmeaSetDeprecated(payload);
  }
}
