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
/** @file bream_handler.h
*============================================================================*/
#include <string.h>
#include <stdint.h>
#include "bream.h"

#ifndef BREAM_HANDLER_H
#define BREAM_HANDLER_H

typedef void (* LD2BRM_Nmea)(U1 *, U4);
typedef bool (* LD2BRM_RawData)(U1 *, U2);
class BreamHandler
{
public:
  BreamHandler();
  ~BreamHandler();

  static BreamHandler & GetInstance(void);

  void AsicData(unsigned char * pucData, unsigned short usLen);

public:
  void SetRawPacketDumpPath(const char * path)
  {
    strncpy(m_acRawPacketDumpPath, path, sizeof(m_acRawPacketDumpPath) - 1);
    m_bPacketDump = true;
  }
  void ResetState();
  void Send(uint16_t payloadLen, unsigned char * pucPayload);
  void BuildAndSend(uint16_t clsid, uint16_t payloadLen, unsigned char * pucPayload);

  static uint8_t LengthCRC(uint16_t data);
  static uint16_t DataCRC(uint16_t c, uint8_t d);

  void Send(LD2BRM_StpBatchSetPayload & payload);
  void Send(LD2BRM_StpConfigCommandPayload & payload);
  void Send(LD2BRM_StpConfigCommandPayloadWithOption & payload);
  void Send(LD2BRM_StpGeofenceSetPayload & payload);
  void Send(LD2BRM_StpSynchPollPayload & payload);
  void Send(LD2BRM_StpSynchSetPayload & payload);
  void Send(LD2BRM_StpMeSettingsPollPayload & payload);
  void Send(LD2BRM_StpMeSettingsSetPayload & payload);
  void Send(LD2BRM_StpInfoPollPayload & payload);
  void Send(LD2BRM_StpInfoSetPayload & payload);
  void Send(LD2BRM_StpLogfilterSetPayload & payload);
  void Send(LD2BRM_StpPeSettingsPollPayload & payload);
  void Send(LD2BRM_StpPeSettingsSetPayload & payload);
  void Send(LD2BRM_StpMessagePollPayload & payload);
  void Send(LD2BRM_StpMessageSetsPayload & payload);
  void Send(LD2BRM_StpMessageSetPayload & payload);
  void Send(LD2BRM_StpNmeaSetDeprecatedPayload & payload);
  void Send(LD2BRM_StpNmeaSetV0Payload & payload);
  void Send(LD2BRM_StpNmeaSetV1Payload & payload);
  void Send(LD2BRM_StpPwrModeSetPayload & payload);
  void Send(LD2BRM_StpPortPollPayload & payload);
  void Send(LD2BRM_StpPortSetUartPayload & payload);
  void Send(LD2BRM_StpPortSetUsbPayload & payload);
  void Send(LD2BRM_StpPortSetSpiPayload & payload);
  void Send(LD2BRM_StpPortSetDdcPayload & payload);
  void Send(LD2BRM_StpRateSetPayload & payload);
  void Send(LD2BRM_StpResetCommandPayload & payload);
  void Send(LD2BRM_StpRxmSetPayload & payload);
  void Send(LD2BRM_StpLteSetPayload & payload);
  void Send(LD2BRM_StpLtePollPayload & payload);
  void Send(LD2BRM_StpBiasSetPayload & payload);
  void Send(LD2BRM_StpBiasPollPayload & payload);
  void Send(LD2BRM_StpLnaSetPayload & payload);
  void Send(LD2BRM_StpLnaPollPayload & payload);
  void Send(LD2BRM_StpFctTestSetPayload & payload);
  void Send(LD2BRM_StpFctTestPollPayload & payload);
  void Send(LD2BRM_StpPpsPayload & payload);
  void Send(LD2BRM_LogCreateCommandPayload & payload);
  void Send(LD2BRM_LogEraseCommand & payload);
  void Send(LD2BRM_LogFindtimeInputPayload & payload);
  void Send(LD2BRM_LogInfoPollPayload & payload);
  void Send(LD2BRM_LogRetrievebatchCommandPayload & payload);
  void Send(LD2BRM_LogRetrieveCommandPayload & payload);
  void Send(LD2BRM_AstLtoInputPayload & payload);
  void Send(LD2BRM_AstNvmemPollPayload & payload);
  void Send(LD2BRM_AstNvmemInputPayload & payload);
  void Send(LD2BRM_AstIniPosXyzInputPayload & payload);
  void Send(LD2BRM_AstIniPosLlhInputPayload & payload);
  void Send(LD2BRM_AstRefTimeUtcInputPayload & payload);
  void Send(LD2BRM_AstIniTimeGnssInputPayload & payload);
  void Send(LD2BRM_AstIniClkdInputPayload & payload);
  void Send(LD2BRM_StsVerPollPayload & payload);
  void Send(LD2BRM_PvtResetAccDistanceCommandPayload & payload);
  void Send(LD2BRM_AscPwrCycleCommandPayload & payload);
  void Send(LD2BRM_BupPollPayload & payload);
  void Send(LD2BRM_BupCommandPayload & payload);
  void Send(LD2BRM_StpConfig2PollPayload & payload);
  void Send(LD2BRM_StpConfig2CommandPayload & payload);

public:
  void SetCallback(LD2BRM_RawData cb) {m_cbLD2BRM_RawData = cb;}
  void SetCallback(LD2BRM_Nmea cb) {m_cbLD2BRM_Nmea = cb;}

  void SetCallback(LD2BRM_AckAckOutput cb) {m_cbLD2BRM_ACK_ACK_Output = cb;}
  void SetCallback(LD2BRM_AckNackOutput cb) {m_cbLD2BRM_ACK_NACK_Output = cb;}
  void SetCallback(LD2BRM_StpSynchGet cb) {m_cbLD2BRM_Stp_ESRC_Get = cb;}
  void SetCallback(LD2BRM_StpBatchSet cb) {m_cbLD2BRM_StpBatchSet = cb;}
  void SetCallback(LD2BRM_StpMeSettingsGet cb) {m_cbLD2BRM_Stp_GNSS_Get = cb;}
  void SetCallback(LD2BRM_StpPeSettingsGet cb) {m_cbLD2BRM_Stp_NAVX5_Get = cb;}
  void SetCallback(LD2BRM_StpConfig2Get cb) {m_cbLD2BRM_Stp_NAV5_Get = cb;}
  void SetCallback(LD2BRM_StpPortSetUart cb) {m_cbLD2BRM_StpPortUart_Get = cb;}
  void SetCallback(LD2BRM_StpPortSetUsb cb) {m_cbLD2BRM_StpPortUsb_Get = cb;}
  void SetCallback(LD2BRM_StpPortSetSpi cb) {m_cbLD2BRM_StpPortSpi_Get = cb;}
  void SetCallback(LD2BRM_StpPortSetDdc cb) {m_cbLD2BRM_StpPortDdc_Get = cb;}
  void SetCallback(LD2BRM_StpRateSet cb) {m_cbLD2BRM_StpRateGet = cb;}
  void SetCallback(LD2BRM_StpInfoSet cb) {m_cbLD2BRM_StpInfoGet = cb;}
  void SetCallback(LD2BRM_StpMessageSets cb) {m_cbLD2BRM_StpMessageGet = cb;}
  void SetCallback(LD2BRM_StpPwrModeSet cb) {m_cbLD2BRM_StpPwrModeGet = cb;}
  void SetCallback(LD2BRM_StpLteGet cb) {m_cbLD2BRM_StpLteGet = cb;}
  void SetCallback(LD2BRM_StpBiasGet cb) {m_cbLD2BRM_StpBiasGet = cb;}
  void SetCallback(LD2BRM_StpLnaGet cb) {m_cbLD2BRM_StpLnaGet = cb;}
  void SetCallback(LD2BRM_StpFctTestGet cb) {m_cbLD2BRM_StpFctTestGet = cb;}
  void SetCallback(LD2BRM_StpPpsGet cb) {m_cbLD2BRM_StpPpsGet = cb;}
  void SetCallback(LD2BRM_WngDebugOutput cb) {m_cbLD2BRM_Wng_DEBUG_Output = cb;}
  void SetCallback(LD2BRM_WngErrorOutput cb) {m_cbD2BRM_WNG_ERROR_Output = cb;}
  void SetCallback(LD2BRM_WngNoticeOutput cb) {m_cbLD2BRM_Wng_NOTICE_Output = cb;}
  void SetCallback(LD2BRM_WngTestOutput cb) {m_cbLD2BRM_Wng_TEST_Output = cb;}
  void SetCallback(LD2BRM_WngWarningOutput cb) {m_cbLD2BRM_Wng_WARNING_Output = cb;}
  void SetCallback(LD2BRM_LogBatchPolled cb) {m_cbLD2BRM_LOG_BATCH_Polled = cb;}
  void SetCallback(LD2BRM_LogFindtimeOutput cb) {m_cbLD2BRM_LOG_FINDTIME_Output = cb;}
  void SetCallback(LD2BRM_LogInfoOutput cb) {m_cbLD2BRM_LOG_INFO_Output = cb;}
  void SetCallback(LD2BRM_LogRetrieveposextraOutput cb)
  {
    m_cbLD2BRM_LOG_RETRIEVEPOSEXTRA_Output = cb;
  }
  void SetCallback(LD2BRM_LogRetrieveposOutput cb) {m_cbLD2BRM_LOG_RETRIEVEPOS_Output = cb;}
  void SetCallback(LD2BRM_LogRetrievestringOutput cb) {m_cbLD2BRM_LOG_RETRIEVESTRING_Output = cb;}
  void SetCallback(LD2BRM_AstAckOutput cb) {m_cbLD2BRM_Ast_ACK_Output = cb;}
  void SetCallback(LD2BRM_AstNvmemOutput cb) {m_cbLD2BRM_Ast_DBD_Output = cb;}
  void SetCallback(LD2BRM_StsBatchPolled cb) {m_cbLD2BRM_Sts_BATCH_Polled = cb;}
  void SetCallback(LD2BRM_StsGnssPolled cb) {m_cbLD2BRM_Sts_GNSS_Polled = cb;}
  void SetCallback(LD2BRM_StsHwPolled cb) {m_cbLD2BRM_Sts_HW_Polled = cb;}
  void SetCallback(LD2BRM_StsIoPolled cb) {m_cbLD2BRM_Sts_IO_Polled = cb;}
  void SetCallback(LD2BRM_StsRxrOutput cb) {m_cbLD2BRM_Sts_RXR_Output = cb;}
  void SetCallback(LD2BRM_StsVerPolled cb) {m_cbLD2BRM_Sts_VER_Polled = cb;}
  void SetCallback(LD2BRM_PvtClockPolled cb) {m_cbLD2BRM_Pvt_CLOCK_Polled = cb;}
  void SetCallback(LD2BRM_PvtDopPolled cb) {m_cbLD2BRM_Pvt_DOP_Polled = cb;}
  void SetCallback(LD2BRM_PvtEoePeriodic cb) {m_cbLD2BRM_Pvt_EOE_Periodic = cb;}
  void SetCallback(LD2BRM_PvtGeofencePolled cb) {m_cbLD2BRM_Pvt_GEOFENCE_Polled = cb;}
  void SetCallback(LD2BRM_PvtAccDistancePolled cb) {m_cbLD2BRM_Pvt_ODO_Polled = cb;}
  void SetCallback(LD2BRM_PvtOrbPolled cb) {m_cbLD2BRM_Pvt_ORB_Polled = cb;}
  void SetCallback(LD2BRM_Pvt_PVT_Polled cb) {m_cbLD2BRM_Pvt_PVT_Polled = cb;}
  void SetCallback(LD2BRM_PvtSatPolled cb) {m_cbLD2BRM_Pvt_SAT_Polled = cb;}
  void SetCallback(LD2BRM_PvtStatusPolled cb) {m_cbLD2BRM_Pvt_STATUS_Polled = cb;}
  void SetCallback(LD2BRM_PvtSvInfoPolled cb) {m_cbLD2BRM_Pvt_SVINFO_Polled = cb;}
  void SetCallback(LD2BRM_PvtVelNedPolled cb) {m_cbLD2BRM_Pvt_VELNED_Polled = cb;}
  void SetCallback(LD2BRM_PvtCbeeStatus cb) {m_cbLD2BRM_PvtCbeeStatus = cb;}
  void SetCallback(LD2BRM_AscSubframesPolled cb) {m_cbLD2BRM_AscSubframesPolled = cb;}
  void SetCallback(LD2BRM_AscMeasPolled cb) {m_cbLD2BRM_AscMeasPolled = cb;}
  void SetCallback(LD2BRM_AscAgcPolled cb) {m_cbLD2BRM_AscAgcPolled = cb;}
  void SetCallback(LD2BRM_BupPolled cb) {m_cbLD2BRM_BupPolled = cb;}
  void SetCallback(LD2BRM_StpNmeaSetDeprecated cb) {m_cbLD2BRM_StpNmeaSetDeprecated = cb;}

private:
  void OnReceive(LD2BRM_AckAckOutputPayload & payload);
  void OnReceive(LD2BRM_AckNackOutputPayload & payload);
  void OnReceive(LD2BRM_StpBatchSetPayload & payload);
  void OnReceive(LD2BRM_StpSynchGetPayload & payload);
  void OnReceive(LD2BRM_StpMeSettingsGetPayload & payload);
  void OnReceive(LD2BRM_StpPeSettingsGetPayload & payload);
  void OnReceive(LD2BRM_StpConfig2CommandPayload & payload);
  void OnReceive(LD2BRM_StpPortPollPayload & payload);
  void OnReceive(LD2BRM_StpRateSetPayload & payload);
  void OnReceive(LD2BRM_StpInfoSetPayload & payload);
  void OnReceive(LD2BRM_StpMessageSetsPayload & payload);
  void OnReceive(LD2BRM_StpPwrModeSetPayload & payload);
  void OnReceive(LD2BRM_StpLteGetPayload & payload);
  void OnReceive(LD2BRM_StpBiasGetPayload & payload);
  void OnReceive(LD2BRM_StpLnaGetPayload & payload);
  void OnReceive(LD2BRM_StpFctTestGetPayload & payload);
  void OnReceive(LD2BRM_StpPpsPayload & payload);
  void OnReceive(LD2BRM_WngDebugOutputPayload & payload);
  void OnReceive(LD2BRM_WngErrorOutputPayload & payload);
  void OnReceive(LD2BRM_WngNoticeOutputPayload & payload);
  void OnReceive(LD2BRM_WngTestOutputPayload & payload);
  void OnReceive(LD2BRM_WngWarningOutputPayload & payload);
  void OnReceive(LD2BRM_LogBatchPolledPayload & payload);
  void OnReceive(LD2BRM_LogFindtimeOutputPayload & payload);
  void OnReceive(LD2BRM_LogInfoOutputPayload & payload);
  void OnReceive(LD2BRM_LogRetrieveposextraOutputPayload & payload);
  void OnReceive(LD2BRM_LogRetrieveposOutputPayload & payload);
  void OnReceive(LD2BRM_LogRetrievestringOutputPayload & payload);
  void OnReceive(LD2BRM_AstAckOutputPayload & payload);
  void OnReceive(LD2BRM_AstNvmemOutputPayload & payload);
  void OnReceive(LD2BRM_StsBatchPolledPayload & payload);
  void OnReceive(LD2BRM_StsGnssPolledPayload & payload);
  void OnReceive(LD2BRM_StsHwPolledPayload & payload);
  void OnReceive(LD2BRM_StsIoPolledPayload & payload);
  void OnReceive(LD2BRM_StsRxrOutputPayload & payload);
  void OnReceive(LD2BRM_StsVerPolledPayload & payload);
  void OnReceive(LD2BRM_PvtClockPolledPayload & payload);
  void OnReceive(LD2BRM_PvtDopPolledPayload & payload);
  void OnReceive(LD2BRM_PvtEoePeriodicPayload & payload);
  void OnReceive(LD2BRM_PvtGeofencePolledPayload & payload);
  void OnReceive(LD2BRM_PvtAccDistancePolledPayload & payload);
  void OnReceive(LD2BRM_PvtOrbPolledPayload & payload);
  void OnReceive(LD2BRM_PvtPvtPolledPayload & payload);
  void OnReceive(LD2BRM_PvtSatPolledPayload & payload);
  void OnReceive(LD2BRM_PvtStatusPolledPayload & payload);
  void OnReceive(LD2BRM_PvtSvInfoPolledPayload & payload);
  void OnReceive(LD2BRM_PvtVelNedPolledPayload & payload);
  void OnReceive(LD2BRM_PvtCbeeStatusPayload & payload);
  void OnReceive(LD2BRM_AscMeasPolledPayload & payload);
  void OnReceive(LD2BRM_AscSubframesPolledPayload & payload);
  void OnReceive(LD2BRM_AscAgcPolledPayload & payload);
  void OnReceive(LD2BRM_BupPolledPayload & payload);
  void OnReceive(LD2BRM_StpNmeaSetDeprecatedPayload & payload);

private:
  LD2BRM_AckAckOutput m_cbLD2BRM_ACK_ACK_Output;
  LD2BRM_AckNackOutput m_cbLD2BRM_ACK_NACK_Output;
  LD2BRM_StpBatchSet m_cbLD2BRM_StpBatchSet;
  LD2BRM_StpSynchGet m_cbLD2BRM_Stp_ESRC_Get;
  LD2BRM_StpMeSettingsGet m_cbLD2BRM_Stp_GNSS_Get;
  LD2BRM_StpPeSettingsGet m_cbLD2BRM_Stp_NAVX5_Get;
  LD2BRM_StpConfig2Get m_cbLD2BRM_Stp_NAV5_Get;
  LD2BRM_Nmea m_cbLD2BRM_Nmea;
  LD2BRM_StpPortSetUart m_cbLD2BRM_StpPortUart_Get;
  LD2BRM_StpPortSetUsb m_cbLD2BRM_StpPortUsb_Get;
  LD2BRM_StpPortSetSpi m_cbLD2BRM_StpPortSpi_Get;
  LD2BRM_StpPortSetDdc m_cbLD2BRM_StpPortDdc_Get;
  LD2BRM_StpRateSet m_cbLD2BRM_StpRateGet;
  LD2BRM_StpInfoSet m_cbLD2BRM_StpInfoGet;
  LD2BRM_StpMessageSets m_cbLD2BRM_StpMessageGet;
  LD2BRM_StpPwrModeSet m_cbLD2BRM_StpPwrModeGet;
  LD2BRM_StpLteGet m_cbLD2BRM_StpLteGet;
  LD2BRM_StpBiasGet m_cbLD2BRM_StpBiasGet;
  LD2BRM_StpLnaGet m_cbLD2BRM_StpLnaGet;
  LD2BRM_StpFctTestGet m_cbLD2BRM_StpFctTestGet;
  LD2BRM_StpPpsGet m_cbLD2BRM_StpPpsGet;
  LD2BRM_WngDebugOutput m_cbLD2BRM_Wng_DEBUG_Output;
  LD2BRM_WngErrorOutput m_cbD2BRM_WNG_ERROR_Output;
  LD2BRM_WngNoticeOutput m_cbLD2BRM_Wng_NOTICE_Output;
  LD2BRM_WngTestOutput m_cbLD2BRM_Wng_TEST_Output;
  LD2BRM_WngWarningOutput m_cbLD2BRM_Wng_WARNING_Output;
  LD2BRM_LogBatchPolled m_cbLD2BRM_LOG_BATCH_Polled;
  LD2BRM_LogFindtimeOutput m_cbLD2BRM_LOG_FINDTIME_Output;
  LD2BRM_LogInfoOutput m_cbLD2BRM_LOG_INFO_Output;
  LD2BRM_LogRetrieveposextraOutput m_cbLD2BRM_LOG_RETRIEVEPOSEXTRA_Output;
  LD2BRM_LogRetrieveposOutput m_cbLD2BRM_LOG_RETRIEVEPOS_Output;
  LD2BRM_LogRetrievestringOutput m_cbLD2BRM_LOG_RETRIEVESTRING_Output;
  LD2BRM_AstAckOutput m_cbLD2BRM_Ast_ACK_Output;
  LD2BRM_AstNvmemOutput m_cbLD2BRM_Ast_DBD_Output;
  LD2BRM_StsBatchPolled m_cbLD2BRM_Sts_BATCH_Polled;
  LD2BRM_StsGnssPolled m_cbLD2BRM_Sts_GNSS_Polled;
  LD2BRM_StsHwPolled m_cbLD2BRM_Sts_HW_Polled;
  LD2BRM_StsIoPolled m_cbLD2BRM_Sts_IO_Polled;
  LD2BRM_StsRxrOutput m_cbLD2BRM_Sts_RXR_Output;
  LD2BRM_StsVerPolled m_cbLD2BRM_Sts_VER_Polled;
  LD2BRM_PvtClockPolled m_cbLD2BRM_Pvt_CLOCK_Polled;
  LD2BRM_PvtDopPolled m_cbLD2BRM_Pvt_DOP_Polled;
  LD2BRM_PvtEoePeriodic m_cbLD2BRM_Pvt_EOE_Periodic;
  LD2BRM_PvtGeofencePolled m_cbLD2BRM_Pvt_GEOFENCE_Polled;
  LD2BRM_PvtAccDistancePolled m_cbLD2BRM_Pvt_ODO_Polled;
  LD2BRM_PvtOrbPolled m_cbLD2BRM_Pvt_ORB_Polled;
  LD2BRM_Pvt_PVT_Polled m_cbLD2BRM_Pvt_PVT_Polled;
  LD2BRM_PvtSatPolled m_cbLD2BRM_Pvt_SAT_Polled;
  LD2BRM_PvtStatusPolled m_cbLD2BRM_Pvt_STATUS_Polled;
  LD2BRM_PvtSvInfoPolled m_cbLD2BRM_Pvt_SVINFO_Polled;
  LD2BRM_PvtVelNedPolled m_cbLD2BRM_Pvt_VELNED_Polled;
  LD2BRM_PvtCbeeStatus m_cbLD2BRM_PvtCbeeStatus;
  LD2BRM_AscSubframesPolled m_cbLD2BRM_AscSubframesPolled;
  LD2BRM_AscMeasPolled m_cbLD2BRM_AscMeasPolled;
  LD2BRM_AscAgcPolled m_cbLD2BRM_AscAgcPolled;
  LD2BRM_BupPolled m_cbLD2BRM_BupPolled;
  LD2BRM_StpNmeaSetDeprecated m_cbLD2BRM_StpNmeaSetDeprecated;

  LD2BRM_RawData m_cbLD2BRM_RawData;

private:
  void HandlePayload(uint8_t cls, uint8_t id, uint16_t payloadLen, unsigned char * pucPayload);
  void DumpRawPacket(uint8_t * buf, uint16_t size, bool bWrite);

  unsigned char m_aucRxMessageBuf[LD2BRM_MAX_PACKET_SIZE];
  unsigned char m_aucRxNmeaBuf[LD2BRM_MAX_PACKET_SIZE];
  unsigned char m_aucRxDumpBuffer[LD2BRM_MAX_PACKET_SIZE];
  char m_acRawPacketDumpPath[512];
  uint32_t m_uiParserState;
  uint32_t m_uiNmeaParseState;
  uint32_t m_uiRxLen;
  uint32_t m_uiPayloadLen;
  uint32_t m_ulPrevRxGarbageBytes;
  uint32_t m_ulRxGarbageBytes;
  uint32_t m_uiRxNmeaLen;
  uint32_t m_uiRxDumpLen;
  void * m_fdDump;
  bool m_bPacketDump;
};
#endif  // BREAM_HANDLER_H
