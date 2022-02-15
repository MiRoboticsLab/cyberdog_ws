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
/** @file bream_helper.cpp
*============================================================================*/

#include <stdio.h>
#include <string.h>

#include "lodi2_os_glue.h"
#include "bream_helper.h"
#include "bream_handler.h"
#include "bream_ast_data.h"
#include "timegm.h"

BreamHelper & BreamHelper::GetInstance(void)
{
  static BreamHelper s_instance;
  return s_instance;
}

BreamHelper::BreamHelper()
: m_refTime(0),
  m_ackNack(0),
  m_respCircBuf(sizeof(m_respBuf), m_respBuf),
  m_fdRawMsmt(NULL),
  m_bRawMsmt(false),
  m_numCbs(0)
{
}


void BreamHelper::WaitAck(uint16_t clsid)
{
#define TIMEOUT_ACK 5000
  int timeout = TIMEOUT_ACK;   //ms
  m_ackNackClsid = clsid;
  while (!m_ackNack && timeout > 0) {
    LD2OS_delay(1);
    timeout--;
  }
  if (!m_ackNack) {
    LD2_LOG("[%s] 0x%04X timeout!\n", __FUNCTION__, clsid);
  } else if (m_ackNack == 2) {
    LD2_LOG("[%s] 0x%04X NACK received!\n", __FUNCTION__, clsid);
  } else {
    //LD2_LOG("[%s] %dms elapsed\n", __FUNCTION__, TIMEOUT_ACK - timeout);
  }
  m_ackNack = 0;
#undef TIMEOUT_ACK
}

void BreamHelper::NotifyAckNack(int acknack, uint16_t clsid)
{
  if (m_ackNackClsid == clsid) {
    m_ackNack = acknack;
  }
}

void BreamHelper::GnssStart(void)
{
  LD2BRM_StpResetCommandPayload payload = {};
  payload.resetMode = 9;   // Controlled GNSS start
  BreamHandler::GetInstance().Send(payload);
}

void BreamHelper::GnssStop(void)
{
  LD2BRM_StpResetCommandPayload payload = {};
  payload.resetMode = 8;   // Controlled GNSS start
  BreamHandler::GetInstance().Send(payload);
}

bool BreamHelper::PushMultiAstData(BreamAstData & astData)
{
  const int DATA_WAIT_TIME_MS = 5000;
  const int BREAM_BUF_SIZE = 256;
  int nSent = 0, nAck = 0;
  bool bRet = true;
  int bytesSent = 0;
  bool skipSend = false;

  uint32_t start = LD2OS_getTime();
  m_respCircBuf.reset();
  do {
    if (!skipSend) {
      int sent = astData.Send();
      if (sent > 0) {
        bytesSent += sent;
        nSent++;
      }
    }
    if (bytesSent <= BREAM_BUF_SIZE) {
      continue;
    }
    if (m_respCircBuf.cnt() == 0) {
      LD2OS_delay(1);
      skipSend = true;
    } else {
      char rescode = m_respCircBuf.popc();
      if (rescode != 0) {
        LD2_LOG("%s() failed to push nvmem data!\n", __FUNCTION__);
        bRet = false;
        break;
      }
      nAck++;
      skipSend = false;
    }
  } while (nSent != nAck && LD2OS_getTime() - start < DATA_WAIT_TIME_MS);

  if (bRet && nSent != nAck) {
    LD2_LOG("%s() timeout! Sent:%d, Acked:%d\n", __FUNCTION__, nSent, nAck);
    bRet = false;
  }
  return bRet;
}

void BreamHelper::InjectLto(const char * path)
{
  if (m_refTime == 0) {
    LD2_LOG("%s() reference time is not set!\n", __FUNCTION__);
    return;
  }
  struct LtoData : public BreamAstData
  {
    LtoData(const char * path, time_t reftime)
    : BreamAstData(path, BRM_AST_ANO), tmRef(reftime),
      bh(BreamHandler::GetInstance())
    {}
    int Send() override
    {
      int szRead = 0;
      while ((szRead = ReadOnePacket()) > 0) {
        LD2BRM_AstLtoInputPayload * p = (LD2BRM_AstLtoInputPayload *)(buff + 6);
        LD2_ASSERT(p->gnssId < MAX_GNSS_ID);

        U4 fct = p->reserved2[3] << 24 | p->reserved2[2] << 16 |
          p->reserved2[1] << 8 | p->reserved2[0];

        //convert fct(full cycle time) to unix epoch
        time_t diff = fct2utc(fct) - tmRef;
        const time_t thresh = 10800;
        //select the slices within +-thresh
        if (diff > -thresh && diff <= thresh) {
          bh.Send((uint16_t)szRead, (unsigned char *)buff);
          return szRead;
        }
      }
      return 0;
    }
    time_t tmRef;
    BreamHandler & bh;
  } ld(path, m_refTime);

  if (ld.fp != NULL) {
    PushMultiAstData(ld);
  }
}

void BreamHelper::InjectTime(
  uint16_t year, uint16_t month, uint16_t day, uint16_t hour,
  uint16_t min, uint16_t sec, uint16_t accSec, uint32_t accNSec,
  uint8_t leapSec)
{
  LD2BRM_AstRefTimeUtcInputPayload payload = {};

  payload.type = 0x10;
  payload.version = 0x00;

  payload.type = 0x10;
  payload.version = 0x00;
  payload.ref = 0;
  payload.leapSecs = leapSec;   //TODO: THIS SHOULD NOT BE HARDCODED
  payload.year = (U2)year;
  payload.month = (U1)month;
  payload.day = (U1)day;
  payload.hour = (U1)hour;
  payload.minute = (U1)min;
  payload.second = (U1)sec;
  payload.tAccS = sec;
  payload.tAccNs = accNSec;

  struct tm t { sec, min, hour, day, month - 1, year - 1900 };
  m_refTime = bream_timegm(&t);

  BreamHandler::GetInstance().Send(payload);
}

void BreamHelper::InjectRefLocation(double lat, double lon, double alt, double acc)
{
  LD2BRM_AstIniPosLlhInputPayload payload = {};
  payload.type = 0x01;
  payload.version = 0x00;
  payload.lat = I4(lat * 1e7);
  payload.lon = I4(lon * 1e7);
  payload.alt = I4(alt * 1e2);
  payload.posAcc = U4(acc * 1e2);
  BreamHandler::GetInstance().Send(payload);
}

void BreamHelper::SetOsc(uint32_t freqHz, uint16_t calUncPpb, uint16_t uncalUncPpb)
{
  LD2BRM_StpSynchSetPayload payload = {};
  payload.version = 0;
  payload.numSources = 1;
  payload.sources[0].freq = freqHz * 4;
  payload.sources[0].withTemp = calUncPpb * 256;
  payload.sources[0].maxDevLifeTime = uncalUncPpb;
  BreamHandler::GetInstance().Send(payload);
}

void BreamHelper::SetMsgRate(uint8_t cls, uint8_t id, uint8_t rate)
{
  LD2BRM_StpMessageSetPayload payload = {};
  payload.msgClass = cls;
  payload.msgID = id;
  payload.rate = rate;
  BreamHandler::GetInstance().Send(payload);
}

void BreamHelper::SetAckAiding(bool enable)
{
  LD2BRM_StpPeSettingsSetPayload payload = {};
  payload.mask1 = (1 << 10) | (1 << 14);   // ackAiding, CBEE
  payload.ackAiding = enable ? 1 : 0;
  payload.aopCfg = 1;
  BreamHandler::GetInstance().Send(payload);
}

void BreamHelper::SetLogging(uint8_t (& infMsgMask)[6])
{
  LD2BRM_StpInfoSetPayload payload = {};
  payload.protocolID = 0;

  for (int i = 0; i < 6; i++) {
    payload.infMsgMask[i] = infMsgMask[i];
  }
  BreamHandler::GetInstance().Send(payload);
  WaitAck(BRM_STP_INF);
}

void BreamHelper::SetGnss(uint8_t enabledGnss, uint8_t maxChnNum, bool enableL5)
{
  LD2BRM_StpMeSettingsSetPayload payload = {};
  payload.msgVer = 0;
  //payload.numTrkChHw; /* read only field */
  payload.numTrkChUse = maxChnNum;
  payload.numConfigBlocks = 7;
  for (int i = 0; i < payload.numConfigBlocks; i++) {
    payload.configBlocks[i].gnssId = i;     // 0 (GPS), 1 (SBAS), 2 (Galileo), 3 (BeiDou), 4 (IMES), 5 (QZSS), 6 (GLONASS)
    payload.configBlocks[i].maxTrkCh = maxChnNum;
    if (enabledGnss & (1UL << i)) {
      payload.configBlocks[i].flags = (0x01 << 16) | 0x01;       // L1 |  EN
      payload.configBlocks[i].flags |= enableL5 ? (0x04 << 16) : 0;      // L5
    }
  }
  BreamHandler::GetInstance().Send(payload);
}

void BreamHelper::SetPowerMode(uint8_t value, uint16_t period, uint16_t onTime)
{
  LD2BRM_StpPwrModeSetPayload payload = {};
  payload.version = 0x00;
  payload.powerSetupValue = value;
  payload.period = period;
  payload.onTime = onTime;
  BreamHandler::GetInstance().Send(payload);
}

void BreamHelper::ResetOdometer(void)
{
  LD2BRM_PvtResetAccDistanceCommandPayload payload = {};
  BreamHandler::GetInstance().Send(payload);
}

void BreamHelper::EnterSuspend(uint32_t duration, uint32_t flags, uint32_t wakeupSources)
{
  LD2BRM_AscPwrCycleCommandPayload payload = {};
  payload.version = 0x00;
  payload.duration = duration;
  payload.flags = flags;
  payload.wakeupSources = wakeupSources;
  BreamHandler::GetInstance().Send(payload);
}

void BreamHelper::SetLteFilterEn(uint8_t en)
{
  LD2BRM_StpLteSetPayload payload = {};
  payload.lteFilterEn = en;
  BreamHandler::GetInstance().Send(payload);
}

void BreamHelper::GetLteFilterEn(void)
{
  LD2BRM_StpLtePollPayload payload = {};
  BreamHandler::GetInstance().Send(payload);
}

void BreamHelper::SetL5Bias(uint32_t biasCm)
{
  LD2BRM_StpBiasSetPayload payload = {};
  payload.biasL5 = biasCm;
  BreamHandler::GetInstance().Send(payload);
}

void BreamHelper::GetL5Bias(void)
{
  LD2BRM_StpBiasPollPayload payload = {};
  BreamHandler::GetInstance().Send(payload);
}

void BreamHelper::SetBaudrate(uint32_t baudrate, uint8_t portid, uint16_t outprotomask)
{
  LD2BRM_StpPortSetUartPayload payload = {};
  payload.PortID = portid;   //uart
  payload.baudRate = baudrate;
  payload.outProtoMask = outprotomask;   //bream + nmea
  BreamHandler::GetInstance().Send(payload);
}

uint16_t BreamHelper::ClsId(uint8_t * buf)
{
#ifndef BREAM_V2
  U2 clsid = buf[2] << 8 | buf[3];
#else
  U2 clsid = buf[4] << 8 | buf[5];
#endif
  return clsid;
}

uint16_t BreamHelper::PayloadLen(uint8_t * buf)
{
#ifndef BREAM_V2
  U4 payloadLen = buf[5] << 8 | buf[4];
#else
  U4 payloadLen = (buf[2] << 3 | buf[3] >> 5) - 2;
#endif
  return payloadLen;
}

static bool OnNvmemOut(U1 * buf, U2 len)
{
  static void * fd = NULL;
  U2 clsid = BreamHelper::ClsId(buf);
  if (clsid != BRM_AST_DBD && clsid != BRM_AST_ACK_DATA0) {
    return false;
  }

  if (fd == NULL) {
    fd = LD2OS_openFile(NVMEM_FILE_NAME, LD2OS_O_RDWR | LD2OS_O_CREAT);
  }
  if (fd == NULL) {
    return false;
  }
  if (clsid == BRM_AST_ACK_DATA0) {
    LD2_LOG("%s() Engine isn't loaded yet!\n", __FUNCTION__);
    return true;
  }

  LD2OS_writeFile(fd, (void *)buf, len);

  LD2BRM_AstNvmemInputPayload * p = (LD2BRM_AstNvmemInputPayload *)(buf + 6);
  U1 dbtype = p->reserved1[0];
  U2 offset = p->reserved1[7] << 8 | p->reserved1[6];
  U2 size = p->reserved1[9] << 8 | p->reserved1[8];
  LD2_LOG(
    "MGA-DBD : type:%d, offset:%d, total size:%d\n",
    dbtype, offset, size);

  U4 payloadLen = BreamHelper::PayloadLen(buf);
  //db type 5 comes first and then 1 comes after.
  if (dbtype == 1 && offset + payloadLen - 12 == size) {
    LD2OS_closeFile(fd);
    fd = NULL;
    LD2_LOG("MGA-DBD: nvmem saved in %s\n", NVMEM_FILE_NAME);

    BreamHelper::GetInstance().RemoveRawdataCb(OnNvmemOut);
  }
  return true;
}

void BreamHelper::PullNvmem(void)
{
  LD2BRM_AstNvmemPollPayload payload = {};

  AddRawdataCb(OnNvmemOut);
  BreamHandler::GetInstance().Send(payload);
}

void BreamHelper::PushNvmem(void)
{
  struct NvmemData : public BreamAstData
  {
    NvmemData(const char * fname)
    : BreamAstData(fname, BRM_AST_DBD),
      bh(BreamHandler::GetInstance())
    {
    }
    int Send() override
    {
      int szRead = ReadOnePacket();
      if (szRead <= 0) {return 0;}

      bh.Send((uint16_t)szRead, (unsigned char *)buff);
      return szRead;
    }
    BreamHandler & bh;
  } nvmem(NVMEM_FILE_NAME);
  if (nvmem.fp != NULL) {
    PushMultiAstData(nvmem);
  }
}

void BreamHelper::RestoreNvmemFromBackup(void)
{
  LD2BRM_BupPollPayload payload = {};
  BreamHandler::GetInstance().Send(payload);
}

void BreamHelper::CreateNvmemBackup(void)
{
  LD2BRM_BupCommandPayload payload = {};
  payload.cmd = 0;
  BreamHandler::GetInstance().Send(payload);
}

void BreamHelper::ClearNvmemBackup(void)
{
  LD2BRM_BupCommandPayload payload = {};
  payload.cmd = 1;
  BreamHandler::GetInstance().Send(payload);
}

void BreamHelper::GetVer(void)
{
  LD2BRM_StsVerPollPayload payload = {};
  BreamHandler::GetInstance().Send(payload);
}

void BreamHelper::GnssReset(uint16_t navBbrMask)
{
  LD2BRM_StpResetCommandPayload payload = {};
  payload.navBbrMask = navBbrMask;
  payload.resetMode = 1;   // Controlled software reset
  BreamHandler::GetInstance().Send(payload);
}

static bool OnRawData(U1 * buf, U2 size)
{
  BreamHelper & bh = BreamHelper::GetInstance();
  if (bh.m_numCbs == 0) {
    return false;
  }
  for (int i = 0; i < bh.m_numCbs; i++) {
    if (bh.m_rawdataCallbacks[i](buf, size)) {
      return true;
    }
  }
  return false;
}

void BreamHelper::AddRawdataCb(LD2BRM_RawData cb)
{
  int i = 0;
  for (; i < m_numCbs; i++) {
    if (m_rawdataCallbacks[i] == cb) {
      break;
    }
  }
  if (m_numCbs < MAX_CALLBACK && i == m_numCbs) {
    m_rawdataCallbacks[m_numCbs++] = cb;
    if (m_numCbs == 1) {
      BreamHandler::GetInstance().SetCallback(OnRawData);
    }
  } else {
    LD2_LOG(
      "%s() too many callback(%d) or same callback(%p,%p)\n",
      __FUNCTION__, m_numCbs, m_rawdataCallbacks[i], cb);
  }
}

void BreamHelper::RemoveRawdataCb(LD2BRM_RawData cb)
{
  int i = 0;
  for (; i < m_numCbs; i++) {
    if (m_rawdataCallbacks[i] == cb) {
      break;
    }
  }
  if (i < m_numCbs) {
    if (i != m_numCbs - 1) {   //not last item.
      memmove(&m_rawdataCallbacks[i], &m_rawdataCallbacks[i + 1], m_numCbs - i - 1);
    }
    m_numCbs--;
  }
  if (m_numCbs == 0) {
    BreamHandler::GetInstance().SetCallback(LD2BRM_RawData(nullptr));
  }
}
static bool OnRawMeasurment(U1 * buf, U2 size)
{
  BreamHelper & bh = BreamHelper::GetInstance();
  U2 clsid = BreamHelper::ClsId(buf);
  if (bh.m_fdRawMsmt && (clsid == BRM_ASC_MEAS || clsid == BRM_ASC_SUBFRAMES)) {
    LD2OS_writeFile(bh.m_fdRawMsmt, (void *)buf, size);
    if (bh.m_bRawMsmt == false) {
      LD2OS_closeFile(bh.m_fdRawMsmt);
      bh.m_fdRawMsmt = NULL;
      bh.RemoveRawdataCb(OnRawMeasurment);
    }
  }
  return false;
}

void BreamHelper::StartSavingRawMsmg()
{
#if defined(_WIN32)
    #define RAW_MSMT_FILE ".\\raw_msmt.brx"
#elif defined(__unix__) || defined (__APPLE__)
    #define RAW_MSMT_FILE "./raw_msmt.brx"
#elif defined(SDK_OS_FREE_RTOS)
    #define RAW_MSMT_FILE "/raw_msmt.brx"
#endif

#if defined(RAW_MSMT_FILE)
  if (m_fdRawMsmt) {
    LD2_LOG("%s() %s already opend\n", __FUNCTION__, RAW_MSMT_FILE);
    return;
  }

  m_fdRawMsmt = LD2OS_openFile(RAW_MSMT_FILE, LD2OS_O_RDWR | LD2OS_O_CREAT);
  if (m_fdRawMsmt == NULL) {
    LD2_LOG("%s() failed to open file %s\n", __FUNCTION__, RAW_MSMT_FILE);
    return;
  }
  AddRawdataCb(OnRawMeasurment);
  m_bRawMsmt = true;
#endif
}

void BreamHelper::StopSavingRawMsmg()
{
  m_bRawMsmt = false;
}

void BreamHelper::EnableBlindGalSearch()
{
  LD2BRM_StpConfig2CommandPayload payload = {};
  payload.mask = (1 << 11);   // blindGalSearch
  payload.blindGalSearch = 1;
  BreamHandler::GetInstance().Send(payload);
}

const char * BreamHelper::GetNameOfBreamPacket(uint8_t cls, uint8_t id)
{
  uint16_t clsid = (cls << 8) | id;
  switch (clsid) {
    case BRM_ACK_ACK:        return "ACK-ACK";
    case BRM_ACK_NAK:        return "ACK-NAK";
    case BRM_STP_BATCH:      return "STP-BATCH";
    case BRM_STP_CFG:        return "STP-CFG";
    case BRM_STP_GEOFENCE:   return "STP-GEOFENCE";
    case BRM_STP_ESRC:      return "STP-ESRC";
    case BRM_STP_GNSS:      return "STP-GNSS";
    case BRM_STP_INF:       return "STP-INF";
    case BRM_STP_LOGFILTER: return "STP-LOGFILTER";
    case BRM_STP_MSG:       return "STP-MSG";
    case BRM_STP_NAVX5:     return "STP-NAVX5";
    case BRM_STP_NAV5:      return "STP_NAV5";
    case BRM_STP_NMEA:      return "STP-NMEA";
    case BRM_STP_PMS:       return "STP-PMS";
    case BRM_STP_PRT:       return "STP-PRT";
    case BRM_STP_RATE:      return "STP-RATE";
    case BRM_STP_RST:       return "STP-RST";
    case BRM_STP_RXM:       return "STP-RXM";
    case BRM_STP_LTE:       return "STP-LTE";
    case BRM_STP_BIAS:      return "STP-BIAS";
    case BRM_STP_FCTTEST:   return "STP-FCTTEST";
    case BRM_WNG_DEBUG:     return "INF-DEBUG";
    case BRM_WNG_ERROR:     return "INF-ERROR";
    case BRM_WNG_NOTICE:    return "INF-NOTICE";
    case BRM_WNG_TEST:      return "INF-TEST";
    case BRM_WNG_WARNING:   return "INF-WARNING";
    case BRM_LOG_BATCH:     return "LOG-BATCH";
    case BRM_LOG_CREATE:    return "LOG-CREATE";
    case BRM_LOG_ERASE:     return "LOG-ERASE";
    case BRM_LOG_FINDTIME:  return "LOG-FINDTIME";
    case BRM_LOG_INFO:      return "LOG-INFO";
    case BRM_LOG_RETRIEVEBATCH:    return "LOG-RETRIEVEBATCH";
    case BRM_LOG_RETRIEVEPOSEXTRA: return "LOG-RETRIEVEPOSEXTRA";
    case BRM_LOG_RETRIEVEPOS:      return "LOG-RETRIEVEPOS";
    case BRM_LOG_RETRIEVESTRING:   return "LOG-RETRIEVESTRING";
    case BRM_LOG_RETRIEVE:  return "LOG-RETRIEVE";
    case BRM_AST_ACK_DATA0: return "MGA-ACK-DATA0";
    case BRM_AST_ANO:       return "MGA-ANO";
    case BRM_AST_DBD:       return "MGA-DBD";
    case BRM_AST_INI:       return "MGA-INI";
    case BRM_STS_BATCH:     return "MON-BATCH";
    case BRM_STS_GNSS:      return "MON-GNSS";
    case BRM_STS_HW:        return "MON-HW";
    case BRM_STS_IO:        return "MON-IO";
    case BRM_STS_RXR:       return "MON-RXR";
    case BRM_STS_VER:       return "MON-VER";
    case BRM_PVT_CLOCK:     return "NAV-CLOCK";
    case BRM_PVT_DOP:       return "NAV-DOP";
    case BRM_PVT_EOE:       return "NAV-EOE";
    case BRM_PVT_GEOFENCE:  return "NAV-GEOFENCE";
    case BRM_PVT_ODO:       return "NAV-ODO";
    case BRM_PVT_ORB:       return "NAV-ORB";
    case BRM_PVT_PVT:       return "NAV-PVT";
    case BRM_PVT_RESETODO:  return "NAV-RESETODO";
    case BRM_PVT_SAT:       return "NAV-SAT";
    case BRM_PVT_STATUS:    return "NAV-STATUS";
    case BRM_PVT_SVINFO:    return "NAV-SVINFO";
    case BRM_PVT_VELNED:    return "NAV-VELNED";
    case BRM_ASC_PMREQ:     return "ASC-PMREQ";
    case BRM_ASC_SUBFRAMES: return "BRM-ASC-SUBFRAMES";
    case BRM_ASC_MEAS:      return "BRM-ASC-MEAS";
    case BRM_BUP:           return "BRM-BUP";
    default:    return "Unknown";
  }
}

bool BreamHelper::SetPowerModePreset(uint8_t rfMode, uint8_t pwrMode)
{
  U1 chUse[5] = {20, 32, 20, 32, 12};
  X4 cfg[5][7][3] = {
    {     // single, best
      {0, 0x01, 0x01},
      {1, 0x00, 0x00},
      {2, 0x01, 0x01},
      {3, 0x00, 0x00},
      {5, 0x01, 0x01},
      {6, 0x01, 0x01},
      {7, 0x00, 0x00},
    },
    {     // dual, best
      {0, 0x05, 0x01},
      {1, 0x00, 0x00},
      {2, 0x05, 0x01},
      {3, 0x00, 0x00},
      {5, 0x05, 0x01},
      {6, 0x01, 0x01},
      {7, 0x00, 0x00}
    },
    {     //single, auto
      {0, 0x01, 0x01},
      {1, 0x00, 0x00},
      {2, 0x01, 0x01},
      {3, 0x00, 0x00},
      {5, 0x01, 0x01},
      {6, 0x01, 0x01},
      {7, 0x00, 0x00}
    },
    {     //dual, auto
      {0, 0x05, 0x01},
      {1, 0x00, 0x00},
      {2, 0x05, 0x01},
      {3, 0x00, 0x00},
      {5, 0x05, 0x01},
      {6, 0x01, 0x01},
      {7, 0x00, 0x00}
    },
    {     //single, ULP
      {0, 0x01, 0x01},
      {1, 0x00, 0x00},
      {2, 0x01, 0x00},
      {3, 0x00, 0x00},
      {5, 0x01, 0x01},
      {6, 0x01, 0x01},
      {7, 0x00, 0x00}
    }
  };

  uint8_t mode = 0;
  uint8_t period = 1;
  switch (rfMode) {
    case 0: //single frequency
      switch (pwrMode) {
        case 0: mode = 0; break; //best
        case 1: mode = 2; break; //auto
        case 3: mode = 4; period = 2;  break; //ULP
        default: return false;
      }
      break;
    case 1: //dual frequency
      switch (pwrMode) {
        case 0: mode = 1; break; //best
        case 1: mode = 3; break; //auto
        default: return false;
      }
      break;
    default: return false;
  }
  LD2BRM_StpMeSettingsSetPayload me = {};
  me.msgVer = 0;
  me.numTrkChUse = chUse[mode];
  me.numConfigBlocks = 7;
  for (int i = 0; i < 7; i++) {
    me.configBlocks[i].gnssId = (U1)cfg[mode][i][0];
    me.configBlocks[i].maxTrkCh = 32;
    me.configBlocks[i].resTrkCh = 0;
    me.configBlocks[i].flags = cfg[mode][i][1] << 16 | cfg[mode][i][2];
  }
  BreamHandler::GetInstance().Send(me);

  LD2BRM_StpPwrModeSetPayload pm = {0, pwrMode, period};
  BreamHandler::GetInstance().Send(pm);

  return true;
}
