/*******************************************************************************
 *
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
/** @file bream_callbacks.cpp
*============================================================================*/
#if defined(_WIN32) || defined(_WIN64)
#include <windows.h>
#endif
#include "lodi2_os_glue.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdint.h>
#include <stdlib.h>
#include "bream_handler.h"
#include "bream_helper.h"
#include "bream_callbacks.hpp"
#if defined(SDK_OS_FREE_RTOS)
#include "fsl_debug_console.h"
#else
#define PRINTF printf
#endif
////////////////////////////////////////////////////////////////////////////////////
// BREAM callbacks example
////////////////////////////////////////////

std::map<int, NMEA_callback> NMEA_callbacks_;
std::map<int, PAYLOAD_callback> PAYLOAD_callbacks_;


void User_LD2BRM_AckAckOutput(LD2BRM_AckAckOutputPayload & payload)
{
  LD2_LOG(
    "ACK-ACK : ACK %s \n",
    BreamHelper::GetInstance().GetNameOfBreamPacket(payload.clsID, payload.msgIdD));

  uint16_t clsid = (payload.clsID << 8) | payload.msgIdD;
  BreamHelper::GetInstance().NotifyAckNack(1, clsid);
}

void User_LD2BRM_AckNackOutput(LD2BRM_AckNackOutputPayload & payload)
{
  LD2_LOG(
    "ACK-NACK : NACK %s \n",
    BreamHelper::GetInstance().GetNameOfBreamPacket(payload.clsID, payload.msgIdD));
  uint16_t clsid = (payload.clsID << 8) | payload.msgIdD;
  BreamHelper::GetInstance().NotifyAckNack(2, clsid);
}

void User_LD2BRM_StpSynchGet(LD2BRM_StpSynchGetPayload & payload)
{
  LD2_LOG("CFG-ESRC : NumSrc=%u \n", payload.numSources);
  for (int i = 0; i < payload.numSources; i++) {
    LD2_LOG(
      "CFG-ESRC : Src[%u] = [ ExtInt=%u, SrcType=%u, Flags=0x%04X, Freq=%u, WithTemp=%u, WithAge=%u, TimeToTemp=%u, MaxDevLifeTime=%u ] \n",
      i, payload.sources[i].extInt, payload.sources[i].sourceType, payload.sources[i].flags,
      payload.sources[i].freq, payload.sources[i].withTemp, payload.sources[i].withAge,
      payload.sources[i].timeToTemp, payload.sources[i].maxDevLifeTime);
  }
}

void User_LD2BRM_StpMeSettingsGet(LD2BRM_StpMeSettingsGetPayload & payload)
{
  LD2_LOG(
    "CFG-GNSS : NumTrkChHw=%u, NumTrkChUse=%u, NumCfgBlocks=%u \n", payload.numTrkChHw,
    payload.numTrkChUse, payload.numConfigBlocks);
  for (int i = 0; i < payload.numConfigBlocks; i++) {
    LD2_LOG(
      "CFG-GNSS : CfgBlock[%u] = [ GnssId=%u, NumRsvdTrkCh=%u, MaxTrkCh=%u, flags=0x%08X ] \n",
      i, payload.configBlocks[i].gnssId, payload.configBlocks[i].resTrkCh,
      payload.configBlocks[i].maxTrkCh, payload.configBlocks[i].flags);
  }
}

void User_LD2BRM_StpPeSettingsGet(LD2BRM_StpPeSettingsGetPayload & payload)
{
  LD2_LOG(
    "CFG-NAVX5 : Mask1=0x%02X, AckAiding=%u UseAdr=%u \n", payload.mask1, payload.ackAiding,
    payload.useAdr);
}

void User_LD2BRM_StpBiasGet(LD2BRM_StpBiasGetPayload & payload)
{
  LD2_LOG("CFG-BIAS : BiasL5=%u \n", payload.biasL5);
}

void User_LD2BRM_WngDebugOutput(LD2BRM_WngDebugOutputPayload & payload)
{
  LD2_FLOG("INF-DEBUG : %s", payload.str);
}

void User_LD2BRM_WngErrorOutput(LD2BRM_WngErrorOutputPayload & payload)
{
  LD2_FLOG("INF-ERROR : %s", payload.str);
}

void User_LD2BRM_WngNoticeOutput(LD2BRM_WngNoticeOutputPayload & payload)
{
  LD2_FLOG("INF-NOTICE : %s", payload.str);
}

void User_LD2BRM_WngTestOutput(LD2BRM_WngTestOutputPayload & payload)
{
  LD2_FLOG("INF-TEST : %s", payload.str);
}

void User_LD2BRM_WngWarningOutput(LD2BRM_WngWarningOutputPayload & payload)
{
  LD2_FLOG("INF-WARNING : %s", payload.str);
}

void User_LD2BRM_LogBatchPolled(LD2BRM_LogBatchPolledPayload & payload)
{
  LD2_LOG(
    "LOG-BATCH : Tow=%u, %.4u-%.2u-%.2u %.2u:%.2u:%.2u, FixType(%u), NumSV(%u), LLA=(%f, %f, %f), DoP=%f, LeapS=%d, Speed=%d,Heading=%f, TotalDistance=%u, Valid(%u) \n",
    payload.iTOW, payload.year, payload.month, payload.day, payload.hour, payload.min, payload.sec, payload.fixType, payload.numSV, payload.lat * 1e-7, payload.lon * 1e-7, payload.height * 1e-3, payload.pDOP * 1e-2, payload.leapS, payload.gSpeed * 1e3, payload.headMot * 1e-5, payload.totalDistance,
    payload.valid);
}

void User_LD2BRM_LogFindtimeOutput(LD2BRM_LogFindtimeOutputPayload & payload)
{
  LD2_LOG("LOG-FINDTIME : %u \n", payload.entryNumber);
}

void User_LD2BRM_LogInfoOutput(LD2BRM_LogInfoOutputPayload & payload)
{
  LD2_LOG(
    "LOG-INFO : Capacity=%u, CurMaxLogSize=%u, CurLogSize=%u, EntryCnt=%u, Oldest=%.4u-%.2u-%.2u %.2u:%.2u:%.2u, Newest=%.4u-%.2u-%.2u %.2u:%.2u:%.2u \n",
    payload.filestoreCapacity, payload.currentMaxLogSize, payload.currentLogSize, payload.entryCount, payload.oldestYear, payload.oldestMonth, payload.oldestDay, payload.oldestHour, payload.oldestMinute, payload.oldestSecond, payload.newestYear, payload.newestMonth, payload.newestDay, payload.newestHour, payload.newestMinute,
    payload.newestSecond);
}

void User_LD2BRM_LogRetrieveposextraOutput(LD2BRM_LogRetrieveposextraOutputPayload & payload)
{

  LD2_LOG(
    "LOG-RETRIEVEPOSEXTRA : EntryIndex=%u, DateTime=%.4u-%.2u-%.2u %.2u:%.2u:%.2u, Distance=%u \n",
    payload.entryIndex, payload.year, payload.month, payload.day, payload.hour, payload.minute,
    payload.second, payload.distance);
}

void User_LD2BRM_LogRetrieveposOutput(LD2BRM_LogRetrieveposOutputPayload & payload)
{
  LD2_LOG(
    "LOG-RETRIEVEPOS : DateTime=%.4u-%.2u-%.2u %.2u:%.2u:%.2u, LLA=(%f, %f, %f), Speed=%f, Heading=%f, FixType=%u, NumSV=%u \n",
    payload.year, payload.month, payload.day, payload.hour, payload.minute, payload.second, payload.lat * 1e-7, payload.lon * 1e-7, payload.hMSL * 1e-3, payload.gSpeed * 1e-3, payload.heading * 1e-5, payload.fixType,
    payload.numSV);
}

void User_LD2BRM_LogRetrievestringOutput(LD2BRM_LogRetrievestringOutputPayload & payload)
{
  LD2_LOG(
    "LOG-RETRIEVESTRING : EntryIdx=%u, DateTime=%.4u-%.2u-%.2u %.2u:%.2u:%.2u, ByteCnt=%u\n",
    payload.entryIndex, payload.year, payload.month, payload.day, payload.hour, payload.minute,
    payload.second, payload.byteCount);
}

void User_LD2BRM_AstAckOutput(LD2BRM_AstAckOutputPayload & payload)
{
  BreamHelper & bh = BreamHelper::GetInstance();
  LD2_LOG(
    "MGA-ACK : %s, %u, MsgId=%u (Payload starts with 0x%02X 0x%02X 0x%02X 0x%02X) \n",
    (payload.type == 1 ? "OK" : "NOK"), payload.infoCode, payload.msgId, payload.msgPayloadStart[0],
    payload.msgPayloadStart[1], payload.msgPayloadStart[2], payload.msgPayloadStart[3]);
  if (bh.m_respCircBuf.space() > 0) {
    bh.m_respCircBuf.pushc(payload.infoCode);
  } else {
    LD2_LOG("%s() response buffer full!\n", __FUNCTION__);
  }
}

void User_LD2BRM_AstNvmemOutput(LD2BRM_AstNvmemOutputPayload & payload)
{

}

void User_LD2BRM_StsBatchPolled(LD2BRM_StsBatchPolledPayload & payload)
{
  LD2_LOG(
    "MON-BATCH : Filled=%u, Dropped=%u, DroppedSinceLastMon=%u, NextMsgCnt=%u \n",
    payload.fillLevel, payload.dropsAll, payload.dropsSinceMon, payload.nextMsgCnt);
}

void User_LD2BRM_StsGnssPolled(LD2BRM_StsGnssPolledPayload & payload)
{
  LD2_LOG(
    "User_LD2BRM_StsGnssPolled : SupportedGnss=0x%02X, DeafultGnss=0x%02X, EnabledGnss=0x%02X, SimultaneousGnss=%02X \n",
    payload.supported, payload.defaultGnss, payload.enabled, payload.simultaneous);
}

void User_LD2BRM_StsHwPolled(LD2BRM_StsHwPolledPayload & payload)
{
  LD2_LOG(
    "MON-HW : PinSel=0x%02X, PinBank=0x%02X, PinDir=0x%02X, PinVal=0x%02X, NoisePerMS=%u, AgcCnt=%u, AntennaStatus=%u, AntennaPower=%u, JamIndi=%u, PinIRQ=0x%02X, PullUp=0x%02X, PullDown=0x%02X \n",
    payload.pinSel, payload.pinBank, payload.pinDir, payload.pinVal, payload.noisePerMS, payload.agcCnt, payload.aStatus, payload.aPower, payload.jamInd, payload.pinIrq, payload.pullH,
    payload.pullL);
}

void User_LD2BRM_StsIoPolled(LD2BRM_StsIoPolledPayload & payload)
{
  for (unsigned int i = 0; i < payload.num; i++) {
    LD2_LOG(
      "MON-IO : Port=%u, RxByte=%u, TxByte=%u, ParErr=%u, FrameErr=%u, OverrunErr=%u, RxBusy=%u, TxBusy=%u\n",
      i, payload.port[i].rxBytes, payload.port[i].txBytes, payload.port[i].parityErrs,
      payload.port[i].framingErrs, payload.port[i].overrunErrs, payload.port[i].rxBusy,
      payload.port[i].txBusy);
  }
}

void User_LD2BRM_StsRxrOutput(LD2BRM_StsRxrOutputPayload & payload)
{
  LD2_LOG("MON-RXR : flags=%u \n", payload.flags);
}

void User_LD2BRM_StsVerPolled(LD2BRM_StsVerPolledPayload & payload)
{
  LD2_LOG("MON-VER : SwVer=%s, HwVer=%s \n", payload.swVersion, payload.hwVersion);
  for (unsigned int i = 0; i < payload.num; i++) {
    LD2_LOG("MON-VER : Extension=%s\n", payload.extension[i].extension);
  }
}

void User_LD2BRM_PvtClockPolled(LD2BRM_PvtClockPolledPayload & payload)
{
  LD2_FLOG(
    "NAV-CLOCK : Tow=%u, ClkBias=%u, ClkDrift=%u, TimeAccuracy=%u, FreqAccuracy=%u \n",
    payload.iTOW, payload.clkB, payload.clkD, payload.tAcc, payload.fAcc);
}

void User_LD2BRM_PvtDopPolled(LD2BRM_PvtDopPolledPayload & payload)
{
  LD2_FLOG(
    "NAV-DOP : Tow=%u, GeometricDOP=%f, PositionDOP=%f, TimeDOP=%f, VertDOP=%f, HorDOP=%f, NorthDOP=%f, EastDOP=%f\n",
    payload.iTOW, payload.gDOP * 1e-2, payload.pDOP * 1e-2, payload.tDOP * 1e-2, payload.vDOP * 1e-2, payload.hDOP * 1e-2, payload.nDOP * 1e-2,
    payload.eDOP * 1e-2);
}

void User_LD2BRM_PvtEoePeriodic(LD2BRM_PvtEoePeriodicPayload & payload)
{
  LD2_FLOG("NAV-EOE : Tow=%u\n", payload.iTOW);
}

void User_LD2BRM_PvtGeofencePolled(LD2BRM_PvtGeofencePolledPayload & payload)
{
#ifndef SDK_OS_FREE_RTOS
  LD2_FLOG(
    "NAV-GEOFENCE : Tow=%u, Status=%s, Num=%u CombinedState=%s \n",
    payload.iTOW, (payload.status == 1) ? "Active" : "Not Active", payload.numFences,
    (payload.combState == 0) ? "Unknown" : (payload.combState == 1) ? "Inside" : "Outside");
  for (int i = 0; i < payload.numFences; i++) {
    LD2_FLOG(
      "NAV-GEOFENCE : Geofence[%u], State=%s \n",
      i, (payload.combState == 0) ? "Unknown" : (payload.combState == 1) ? "Inside" : "Outside");
  }
#endif
}

void User_LD2BRM_PvtAccDistancePolled(LD2BRM_PvtAccDistancePolledPayload & payload)
{
  LD2_FLOG(
    "NAV-ODO : Tow=%u, Distance=%u, TotalDistance=%u, DistanceStd=%u \n",
    payload.iTOW, payload.distance, payload.totalDistance, payload.distanceStd);
}

void User_LD2BRM_PvtOrbPolled(LD2BRM_PvtOrbPolledPayload & payload)
{
#ifndef SDK_OS_FREE_RTOS
  LD2_FLOG("NAV-ORB : Tow=5u, NumSV=%u \n", payload.iTOW, payload.numSv);
  for (int i = 0; i < payload.numSv; i++) {
    LD2_FLOG(
      "NAV-ORB : Orbit[%u] = [ GnssId=%u, SvId=%u, SvFlag=0x%02X, Eph=0x%02X, Alm=0x%02X, Other=0x%02X ]\n",
      i, payload.OrbInfo[i].gnssId, payload.OrbInfo[i].svId, payload.OrbInfo[i].svFlag,
      payload.OrbInfo[i].eph, payload.OrbInfo[i].alm, payload.OrbInfo[i].otherOrb);
  }
#endif
}

void User_LD2BRM_PvtPvtPolled(LD2BRM_PvtPvtPolledPayload & payload)
{
  LD2_FLOG(
    "NAV-PVT : Tow=%u, DateTime=%.4u-%.2u-%.2u %.2u:%.2u:%.2u, FixType=%u, NumSv=%u, LLA=(%f, %f, %f), DoP=%f, leapS:%d, Speed=%f, Heading=%f, Valid=%u \n",
    payload.iTOW, payload.year, payload.month, payload.day, payload.hour, payload.min, payload.sec, payload.fixType, payload.numSV, payload.lat * 1e-7, payload.lon * 1e-7, payload.hMSL * 1e-3, payload.pDOP * 1e-2, payload.leapS, payload.gSpeed * 1e-3, payload.headMot * 1e-5,
    payload.valid);
  auto payload_ptr = std::make_shared<LD2BRM_PvtPvtPolledPayload>(payload);
  for (auto & cb : PAYLOAD_callbacks_) {
    if (cb.second != nullptr) {
      cb.second(payload_ptr);
    }
  }
  // switch (payload.fixType) {
  //   case 2:
  //     PRINTF("+");
  //     break;
  //   case 3:
  //     PRINTF("*");
  //     break;
  //   case 5:
  //     PRINTF("-");
  //     break;
  //   default:
  //     PRINTF(".");
  // }
}

void User_LD2BRM_PvtSatPolled(LD2BRM_PvtSatPolledPayload & payload)
{
#ifndef SDK_OS_FREE_RTOS
  LD2_FLOG("NAV-SAT : Tow=%u, NumSv=%u \n", payload.iTOW, payload.numSvs);
  for (int i = 0; i < payload.numSvs; i++) {
    LD2_FLOG(
      "NAV-SAT : Sv[%u] = [ GnssId=%u, SvId=%u, CN0=%u, Elev=%d, Azimuth=%d, PseuroRangeResidual=%f, Flags=%08X ] \n",
      i, payload.Svs[i].gnssId, payload.Svs[i].svId, payload.Svs[i].cno, payload.Svs[i].elev,
      payload.Svs[i].azim, payload.Svs[i].prRes * 1e-1, payload.Svs[i].flags);
  }
#endif
}

void User_LD2BRM_PvtStatusPolled(LD2BRM_PvtStatusPolledPayload & payload)
{
  LD2_FLOG(
    "NAV-STATUS : Tow=%u, GpsFix=%u, Flags=0x%02X, FixStatus=0x%02X, Flags2==0x%02X, TTFF=%f \n",
    payload.iTOW, payload.gpsFix, payload.flags, payload.fixStat, payload.flags2,
    payload.ttff * 1e-3);
}

void User_LD2BRM_PvtSvInfoPolled(LD2BRM_PvtSvInfoPolledPayload & payload)
{
#ifndef SDK_OS_FREE_RTOS
  LD2_FLOG(
    "NAV-SVINFO : Tow=%u, NumCh=%u, GlbFlags=0x%02X \n", payload.iTOW, payload.numCh,
    payload.globalFlags);
  for (int i = 0; i < payload.numCh; i++) {
    LD2_FLOG(
      "NAV-SVINFO : SvInfo[%u] = [ Chn=%u, SvId=%u, Flags=0x%02X, Quality=0x%02X, CN0=%u, Elev=%d, Azim=%d, PseuroRangeResidual=%d ] \n",
      i, payload.SvInfo[i].chn, payload.SvInfo[i].svid, payload.SvInfo[i].flags, payload.SvInfo[i].quality,
      payload.SvInfo[i].cno, payload.SvInfo[i].elev, payload.SvInfo[i].azim,
      payload.SvInfo[i].prRes);
  }
#endif
}

void User_LD2BRM_PvtVelNedPolled(LD2BRM_PvtVelNedPolledPayload & payload)
{
  LD2_FLOG(
    "NAV-VELNED : Tow=%u, Vel=(%f %f %f), Speed=%f, Heading=%f \n", payload.iTOW,
    payload.velN * 1e-2, payload.velE * 1e-2, payload.velD * 1e-2, payload.gSpeed * 1e-2,
    payload.heading * 1e-5);
}

void User_LD2BRM_Nmea(U1 * str, U4 len)
{
  LD2_LOG("USER NMEA : %s", str);
  for (auto & cb : NMEA_callbacks_) {
    if (cb.second != nullptr) {
      cb.second(str, len);
    }
  }
}

void User_LD2BRM_PvtCbeeStatus(LD2BRM_PvtCbeeStatusPayload & payload)
{
  LD2_FLOG(
    "PVT-CBEE_STATUS : Tow=%u, CbeeCfg=%u, Status=%u \n", payload.iTOW, payload.cbeeCfg,
    payload.status);
}

void User_LD2BRM_AscSubframesPolled(LD2BRM_AscSubframesPolledPayload & payload)
{
  LD2_FLOG(
    "ASC-SUBFRAMES: gnssId=%u, svId=%u, freqId=%u, numWords=%u, version=%u\n",
    payload.gnssId, payload.svId, payload.freqId, payload.numWords, payload.version);
}

void User_LD2BRM_AscMeasPolled(LD2BRM_AscMeasPolledPayload & payload)
{
#ifndef SDK_OS_FREE_RTOS
  LD2_FLOG(
    "ASC-MEAS : RcvTow=%f, Week=%u, LeapS=%d, NumMeas=%u, RecStat=0x%02X, Version=%u \n",
    payload.rcvTow, payload.week, payload.leapS, payload.numMeas, payload.recStat, payload.version);
  for (int i = 0; i < payload.numMeas; i++) {
    LD2_FLOG(
      "ASC-MEAS : Meas[%u] = [ PrMes=%f, CpMes=%f, DoMes=%f, GnssId=%u, SvId=%u, SigId=%u, "
      "FreqId=%u, Locktime=%u, Cn0=%u, PrStdev=0x%02X, CpStdev=0x%02X, DoStdev=0X%02X, "
      "TrkStat=0x%02X, Extra=0x%02X ] \n",
      i, payload.meas[i].prMes, payload.meas[i].cpMes, payload.meas[i].doMes,
      payload.meas[i].gnssId,
      payload.meas[i].svId, payload.meas[i].sigId, payload.meas[i].freqId, payload.meas[i].locktime,
      payload.meas[i].cn0, payload.meas[i].prStdev, payload.meas[i].cpStdev,
      payload.meas[i].doStdev,
      payload.meas[i].trkStat, payload.meas[i].extra);
  }
#endif
}

void User_LD2BRM_LD2BRM_AscAgcPolled(LD2BRM_AscAgcPolledPayload & p)
{
#ifndef SDK_OS_FREE_RTOS
  LD2_FLOG("ASC-AGC : iTow:%u, numMeas:%u\n", p.iTow, p.numMeas);
  for (int i = 0; i < p.numMeas; i++) {
    LD2BRM_AscAgcContent & b = p.contentBlocks[i];
    LD2_FLOG(
      "ASC-AGC : Contents[%d] : gnssId:%u, svId:%u, sigId:%u, freqId:%u, AGCdB:%0.3f\n",
      i, b.gnssId, b.svId, b.sigId, b.freqId, b.AGCdB * 0.125);
  }
#endif
}

void User_LD2BRM_BupPolled(LD2BRM_BupPolledPayload & payload)
{
  if (payload.cmd == 2) {
    switch (payload.response) {
      case 0: LD2_FLOG("BUP : command NACK\n"); break;
      case 1: LD2_FLOG("BUP : command ACK\n"); break;
      default: LD2_ASSERT(0);
    }
  } else if (payload.cmd == 3) {
    switch (payload.response) {
      case 0: LD2_FLOG("BUP: restore result UNKNOWN\n"); break;
      case 1: LD2_FLOG("BUP: restore result FAILED\n"); break;
      case 2: LD2_FLOG("BUP: restore result OK\n"); break;
      case 3: LD2_FLOG("BUP: restore result FAILED. Backup not found\n"); break;
      default: LD2_ASSERT(0);
    }
  } else {
    LD2_ASSERT(0);
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////


void RegisterBreamCallbacks(void)
{
  // Register callbacks
  BreamHandler::GetInstance().SetCallback(User_LD2BRM_AckAckOutput);
  BreamHandler::GetInstance().SetCallback(User_LD2BRM_AckNackOutput);
  BreamHandler::GetInstance().SetCallback(User_LD2BRM_StpSynchGet);
  BreamHandler::GetInstance().SetCallback(User_LD2BRM_StpMeSettingsGet);
  BreamHandler::GetInstance().SetCallback(User_LD2BRM_StpPeSettingsGet);
  BreamHandler::GetInstance().SetCallback(User_LD2BRM_StpBiasGet);
  BreamHandler::GetInstance().SetCallback(User_LD2BRM_WngDebugOutput);
  BreamHandler::GetInstance().SetCallback(User_LD2BRM_WngErrorOutput);
  BreamHandler::GetInstance().SetCallback(User_LD2BRM_WngNoticeOutput);
  BreamHandler::GetInstance().SetCallback(User_LD2BRM_WngTestOutput);
  BreamHandler::GetInstance().SetCallback(User_LD2BRM_WngWarningOutput);
  BreamHandler::GetInstance().SetCallback(User_LD2BRM_LogBatchPolled);
  BreamHandler::GetInstance().SetCallback(User_LD2BRM_LogFindtimeOutput);
  BreamHandler::GetInstance().SetCallback(User_LD2BRM_LogInfoOutput);
  BreamHandler::GetInstance().SetCallback(User_LD2BRM_LogRetrieveposextraOutput);
  BreamHandler::GetInstance().SetCallback(User_LD2BRM_LogRetrieveposOutput);
  BreamHandler::GetInstance().SetCallback(User_LD2BRM_LogRetrievestringOutput);
  BreamHandler::GetInstance().SetCallback(User_LD2BRM_AstAckOutput);
  BreamHandler::GetInstance().SetCallback(User_LD2BRM_AstNvmemOutput);
  BreamHandler::GetInstance().SetCallback(User_LD2BRM_StsBatchPolled);
  BreamHandler::GetInstance().SetCallback(User_LD2BRM_StsGnssPolled);
  BreamHandler::GetInstance().SetCallback(User_LD2BRM_StsHwPolled);
  BreamHandler::GetInstance().SetCallback(User_LD2BRM_StsIoPolled);
  BreamHandler::GetInstance().SetCallback(User_LD2BRM_StsRxrOutput);
  BreamHandler::GetInstance().SetCallback(User_LD2BRM_StsVerPolled);
  BreamHandler::GetInstance().SetCallback(User_LD2BRM_PvtClockPolled);
  BreamHandler::GetInstance().SetCallback(User_LD2BRM_PvtDopPolled);
  BreamHandler::GetInstance().SetCallback(User_LD2BRM_PvtEoePeriodic);
  BreamHandler::GetInstance().SetCallback(User_LD2BRM_PvtGeofencePolled);
  BreamHandler::GetInstance().SetCallback(User_LD2BRM_PvtAccDistancePolled);
  BreamHandler::GetInstance().SetCallback(User_LD2BRM_PvtOrbPolled);
  BreamHandler::GetInstance().SetCallback(User_LD2BRM_PvtPvtPolled);
  BreamHandler::GetInstance().SetCallback(User_LD2BRM_PvtSatPolled);
  BreamHandler::GetInstance().SetCallback(User_LD2BRM_PvtStatusPolled);
  BreamHandler::GetInstance().SetCallback(User_LD2BRM_PvtSvInfoPolled);
  BreamHandler::GetInstance().SetCallback(User_LD2BRM_PvtVelNedPolled);
  BreamHandler::GetInstance().SetCallback(User_LD2BRM_Nmea);
  BreamHandler::GetInstance().SetCallback(User_LD2BRM_BupPolled);
  BreamHandler::GetInstance().SetCallback(User_LD2BRM_AscSubframesPolled);
  BreamHandler::GetInstance().SetCallback(User_LD2BRM_LD2BRM_AscAgcPolled);
  BreamHandler::GetInstance().SetCallback(User_LD2BRM_AscMeasPolled);
  BreamHandler::GetInstance().SetCallback(User_LD2BRM_PvtCbeeStatus);
}
