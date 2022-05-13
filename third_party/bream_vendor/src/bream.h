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
/** @file bream.h  
*============================================================================*/

#include <stdint.h>
#include "lodi2_os_glue.h"

#ifndef BREAM_H
#define BREAM_H

typedef uint8_t U1;
typedef uint16_t U2;
typedef uint32_t U4;

typedef int8_t I1;
typedef int16_t I2;
typedef int32_t I4;

typedef uint8_t X1;
typedef uint16_t X2;
typedef uint32_t X4;

typedef char CH;

typedef float R4;
typedef double R8;


enum {
    BRM_ACK_ACK = 0x0501,
    BRM_ACK_NAK = 0x0500,

    BRM_STP_BATCH = 0x0693,
    BRM_STP_CFG = 0x0609,
    BRM_STP_GEOFENCE = 0x0669,
    BRM_STP_ESRC = 0x0660,
    BRM_STP_GNSS = 0x063E,
    BRM_STP_INF = 0x0602,
    BRM_STP_LOGFILTER = 0x0647,
    BRM_STP_MSG = 0x0601,
    BRM_STP_NAVX5 = 0x0623,
    BRM_STP_NAV5 = 0x0624,
    BRM_STP_NMEA = 0x0617,
    BRM_STP_PMS = 0x0686,
    BRM_STP_PRT = 0x0600,
    BRM_STP_RATE = 0x0608,
    BRM_STP_RST = 0x0604,
    BRM_STP_RXM = 0x0611,
    BRM_STP_LTE = 0x06A0,
    BRM_STP_BIAS = 0x06B0,
    BRM_STP_LNA = 0x06B1,
    BRM_STP_FCTTEST = 0x06B2,
    BRM_STP_PPS = 0x0631,

    BRM_WNG_DEBUG = 0x0404,
    BRM_WNG_ERROR = 0x0400,
    BRM_WNG_NOTICE = 0x0402,
    BRM_WNG_TEST = 0x0403,
    BRM_WNG_WARNING = 0x0401,

    BRM_LOG_BATCH = 0x2111,
    BRM_LOG_CREATE = 0x2107,
    BRM_LOG_ERASE = 0x2103,
    BRM_LOG_FINDTIME = 0x210E,
    BRM_LOG_INFO = 0x2108,
    BRM_LOG_RETRIEVEBATCH = 0x2110,
    BRM_LOG_RETRIEVEPOSEXTRA = 0x210F,
    BRM_LOG_RETRIEVEPOS = 0x210B,
    BRM_LOG_RETRIEVESTRING = 0x210D,
    BRM_LOG_RETRIEVE = 0x2109,

    BRM_AST_ACK_DATA0 = 0X1360,
    BRM_AST_ANO = 0x1320,
    BRM_AST_DBD = 0x1380,
    BRM_AST_INI = 0x1340,

    BRM_STS_BATCH = 0x0A32,
    BRM_STS_GNSS = 0x0A28,
    BRM_STS_HW = 0x0A09,
    BRM_STS_IO = 0x0A02,
    BRM_STS_RXR = 0x0A21,
    BRM_STS_VER = 0x0A04,

    BRM_PVT_CLOCK = 0x0122,
    BRM_PVT_DOP = 0x0104,
    BRM_PVT_EOE = 0x0161,
    BRM_PVT_GEOFENCE = 0x0139,
    BRM_PVT_ODO = 0x0109,
    BRM_PVT_ORB = 0x0134,
    BRM_PVT_PVT = 0x0107,
    BRM_PVT_RESETODO = 0x0110,
    BRM_PVT_SAT = 0x0135,
    BRM_PVT_STATUS = 0x0103,
    BRM_PVT_SVINFO = 0x0130,
    BRM_PVT_VELNED = 0x0112,
    BRM_PVT_CBEE_STATUS = 0x0160,

    BRM_ASC_PMREQ = 0x0241,
    BRM_ASC_SUBFRAMES = 0x0213,
    BRM_ASC_MEAS = 0x0215,
    BRM_ASC_AGC = 0x0280,

    BRM_BUP = 0x0914,
};


#define LD2BRM_MAX_WNG_MESSAGE             504
#define LD2BRM_MAX_CONFIG_BLOCKS            32
#define LD2BRM_MAX_RETRIEVESTRING          504
#define LD2BRM_MAX_STS_IO_STATUS             6
#define LD2BRM_MAX_STS_VER_EXTENSION        15
#define LD2BRM_MAX_GEOFENCES                40
#define LD2BRM_MAX_AST_NVMEM_DUMP           152//164
#define LD2BRM_MAX_ORB_DB_INFO              80
#define LD2BRM_MAX_CHANNEL                  50
#define LD2BRM_MAX_PVT_SVINFO_INFO   40
#define LD2BRM_MAX_FCTTEST_CONFIG_BLOCKS     3
#define LD2BRM_MAX_PACKET_SIZE              2048
#define LD2BRM_MAX_SUBFRAME_DATA_WORDS      10

//////////////////////////////////////////////////////////////////
// BRM Class ACK
//   Ack/Nak Messages
//////////////////////////////////////////////////////////////////


/** @brief BRM-ACK-ACK (0x05 0x01)
*
*/
PACK (
typedef struct
{
    U1 clsID;
    U1 msgIdD;
}) LD2BRM_AckAckOutputPayload;

typedef void (*LD2BRM_AckAckOutput)(LD2BRM_AckAckOutputPayload &payload);


/** @brief BRM-ACK-NACK (0x05 0x00)
*
*/
PACK(
typedef struct
{
    U1 clsID;
    U1 msgIdD;
}) LD2BRM_AckNackOutputPayload;

typedef void (*LD2BRM_AckNackOutput)(LD2BRM_AckNackOutputPayload &payload);




//////////////////////////////////////////////////////////////////
// BRM Class STP
//   Configuration Input Messages
//////////////////////////////////////////////////////////////////

/** @brief BRM-STP-BATCH (0x06 0x93)
*
*/
PACK(
typedef struct
{
    U1 version;
    X1 flags;
    U2 bufSize;
    U2 notifThrs;
    U1 pioId;
    U1 reserved1;
    U2 elapsedFix;
    U2 distanceTraveled;
}) LD2BRM_StpBatchSetPayload;

typedef void (*LD2BRM_StpBatchSet)(LD2BRM_StpBatchSetPayload &payload);


/** @brief BRM-STP-CFG (0x06 0x09)
*
*/
PACK(
typedef struct
{
    X4 clearMask;
    X4 saveMask;
    X4 loadMask;
}) LD2BRM_StpConfigCommandPayload;

PACK(
typedef struct
{
    X4 clearMask;
    X4 saveMask;
    X4 loadMask;
    X1 deviceMask;
}) LD2BRM_StpConfigCommandPayloadWithOption;

typedef void (*LD2BRM_StpConfigCommand)(LD2BRM_StpConfigCommandPayload &payload);
typedef void (*LD2BRM_StpConfigOptionCommand)(LD2BRM_StpConfigCommandPayloadWithOption &payload);


/** @brief BRM-STP-GEOFENCE (0x06 0x69)
*
*/
PACK(
typedef struct
{
    I4 lat;
    I4 lon;
    U4 radius;
}) LD2BRM_GeofenceType;

PACK(
typedef struct
{
    U1 version;
    U1 numFences;
    U1 confLvl;
    U1 reserved1[1];
    U1 pioEnabled;
    U1 pinPolarity;
    U1 pin;
    U1 reserved2[1];
    LD2BRM_GeofenceType fences[LD2BRM_MAX_GEOFENCES];
}) LD2BRM_StpGeofenceSetPayload;

typedef void (*LD2BRM_StpGeofenceSet)(LD2BRM_StpGeofenceSetPayload &payload);


/** @brief BRM-STP-ESRC (0x06 0x60)
*
*/
PACK(
typedef struct {

}) LD2BRM_StpSynchPollPayload;

PACK(
typedef struct {
    U1 extInt;
    U1 sourceType;
    X2 flags;
    U4 freq;
    U1 reserved2[4];
    U4 withTemp;
    U4 withAge;
    U2 timeToTemp;
    U2 maxDevLifeTime;
    I4 offset;
    U4 offsetUncertainty;
    U4 jitter;
}) LD2BRM_StpSynch;

PACK(
typedef struct {
    U1 version;
    U1 numSources;
    U1 reserved1[2];
    LD2BRM_StpSynch sources[1];
}) LD2BRM_StpSynchSetPayload;

typedef LD2BRM_StpSynchSetPayload LD2BRM_StpSynchGetPayload;

typedef void(*LD2BRM_StpSynchGet)(LD2BRM_StpSynchGetPayload &payload);

/** @brief BRM-STP-GNSS (0x06 0x3E)
*
*/
PACK(
typedef struct {
}) LD2BRM_StpMeSettingsPollPayload;

PACK(
typedef struct {
    U1 gnssId;
    U1 resTrkCh;
    U1 maxTrkCh;
    U1 reserved1;
    X4 flags;
}) LD2BRM_StpMeSettingsConfigBlockType;

PACK(
typedef struct
{
    U1 msgVer;
    U1 numTrkChHw;
    U1 numTrkChUse;
    U1 numConfigBlocks;
    LD2BRM_StpMeSettingsConfigBlockType configBlocks[LD2BRM_MAX_CONFIG_BLOCKS];
}) LD2BRM_StpMeSettingsSetPayload;

typedef LD2BRM_StpMeSettingsSetPayload LD2BRM_StpMeSettingsGetPayload;

typedef void(*LD2BRM_StpMeSettingsPoll)(LD2BRM_StpMeSettingsPollPayload &payload);
typedef void (*LD2BRM_StpMeSettingsGet)(LD2BRM_StpMeSettingsGetPayload &payload);



/** @brief BRM-STP-INF (0x06 0x02)
*
*/
PACK(
typedef struct
{
    U1 protocolID;
}) LD2BRM_StpInfoPollPayload;

PACK(
typedef struct
{
    U1 protocolID;
    U1 reserved1[3];
    X1 infMsgMask[6];
}) LD2BRM_StpInfoSetPayload;

typedef void (*LD2BRM_StpInfoPoll)(LD2BRM_StpInfoPollPayload &payload);
typedef void (*LD2BRM_StpInfoSet)(LD2BRM_StpInfoSetPayload &configs);


/** @brief BRM-STP-LOGFILTER (0x06 0x47)
*
*/
PACK(
typedef struct
{
    U1 version;
    X1 flags;
    U2 minInterval;
    U2 timeThreshold;
    U2 speedThreshold;
    U4 positionThreshold;
}) LD2BRM_StpLogfilterSetPayload;

typedef void (*LD2BRM_StpLogfilterSet)(LD2BRM_StpLogfilterSetPayload &payload);


/** @brief BRM-STP-MSG (0x06 0x01)
*
*/
PACK(
typedef struct
{
    U1 msgClass;
    U1 msgID;
}) LD2BRM_StpMessagePollPayload;

PACK(
typedef struct
{
    U1 msgClass;
    U1 msgID;
    U1 rate[6];
}) LD2BRM_StpMessageSetsPayload;

PACK(
typedef struct
{
    U1 msgClass;
    U1 msgID;
    U1 rate;
}) LD2BRM_StpMessageSetPayload;

typedef void (*LD2BRM_StpMessagePoll)(LD2BRM_StpMessagePollPayload &payload);
typedef void (*LD2BRM_StpMessageSets)(LD2BRM_StpMessageSetsPayload &payload);
typedef void (*LD2BRM_StpMessageSet)(LD2BRM_StpMessageSetPayload &payload);


/** @brief BRM-STP-NAVX5 (0x06 0x23)
*
*/
PACK(
typedef struct
{
}) LD2BRM_StpPeSettingsPollPayload;

PACK(
typedef struct
{
    U2 version;
    X2 mask1;
    X4 mask2;
    U1 reserved1[2];
    U1 minSVs;
    U1 maxSVs;
    U1 minCNO;
    U1 reserved2;
    U1 iniFix3D;
    U1 reserved3[2];
    U1 ackAiding;
    U2 wknRollover;
    U1 sigAttenCompMode;
    U1 reserved4;
    U1 reserved5[2];
    U1 reserved6[2];
    U1 usePPP;
    U1 aopCfg;
    U1 reserved7[2];
    U2 aopOrbmaxErr;
    U1 reserved8[4];
    U1 reserved9[3];
    U1 useAdr;
}) LD2BRM_StpPeSettingsSetPayload;

typedef LD2BRM_StpPeSettingsSetPayload LD2BRM_StpPeSettingsGetPayload;

typedef void (*LD2BRM_StpPeSettingsGet)(LD2BRM_StpPeSettingsGetPayload &payload);

/** @brief BRM-STP-NAV5 (0x06 0x24)
*
*/
PACK(
typedef struct
{
}) LD2BRM_StpConfig2PollPayload;

PACK(
typedef struct
{
    X2 mask;
    U1 context;
    U1 reserved1[15];
    U2 pAcc;
    U1 reserved2[8];
    U2 staticHoldMaxDist;
    U1 reserved3;
    U1 blindGalSearch;
    U1 reserved14[4];
}) LD2BRM_StpConfig2CommandPayload;

typedef void (*LD2BRM_StpConfig2Get)(LD2BRM_StpConfig2CommandPayload &payload);

/** @brief BRM-STP-NMEA (0x06 0x17)
*
*/
PACK(
typedef struct
{
    X1 filter;
    U1 nmeaVersion;
    U1 numSV;
    X1 flags;
}) LD2BRM_StpNmeaSetDeprecatedPayload;

PACK(
typedef struct
{
    X1 filter;
    U1 nmeaVersion;
    U1 numSV;
    X1 flags;
    X4 gnssToFilter;
    U1 svNumbering;
    U1 mainTalkerId;
    U1 gsvTalkerId;
    U1 version;
}) LD2BRM_StpNmeaSetV0Payload;

PACK(
typedef struct
{
    X1 filter;
    U1 nmeaVersion;
    U1 numSV;
    X1 flags;
    X4 gnssToFilter;
    U1 svNumbering;
    U1 mainTalkerId;
    U1 gsvTalkerId;
    U1 version;
    CH bdsTalkerId[2];
    U1 reserved1[6];
}) LD2BRM_StpNmeaSetV1Payload;

typedef void (*LD2BRM_StpNmeaSetDeprecated)(LD2BRM_StpNmeaSetDeprecatedPayload &payload);
typedef void (*LD2BRM_StpNmeaSetV0)(LD2BRM_StpNmeaSetV0Payload &payload);
typedef void (*LD2BRM_StpNmeaSetV1)(LD2BRM_StpNmeaSetV1Payload &payload);


/** @brief BRM-STP-PMS (0x06 0x86)
*
*/
PACK(
typedef struct
{
    U1 version;
    U1 powerSetupValue;
    U2 period;
    U2 onTime;
    U1 reserved1[2];
}) LD2BRM_StpPwrModeSetPayload;

typedef void (*LD2BRM_StpPwrModeSet)(LD2BRM_StpPwrModeSetPayload &payload);


/** @brief BRM-STP-PRT (0x06 0x00)
*
*/
PACK(
typedef struct
{
    U1 PortID;
}) LD2BRM_StpPortPollPayload;

PACK(
typedef struct
{
    U1 PortID;
    U1 reserved1;
    X2 txReady;
    X4 mode;
    U4 baudRate;
    X2 inProtoMask;
    X2 outProtoMask;
    X2 flags;
    U1 reserved2[2];
}) LD2BRM_StpPortSetUartPayload;

PACK(
typedef struct
{
    U1 portID;
    U1 reserved1;
    X2 txReady;
    U1 reserved2[8];
    X2 inProtoMask;
    X2 outProtoMask;
    U1 reserved3[2];
    U1 reserved4[2];
}) LD2BRM_StpPortSetUsbPayload;

PACK(
typedef struct
{
    U1 portID;
    U1 reserved1;
    X2 txReady;
    X4 mode;
    U1 reserved2[4];
    X2 inProtoMask;
    X2 outProtoMask;
    X2 flags;
    U1 reserved3[2];
}) LD2BRM_StpPortSetSpiPayload;

PACK(
typedef struct
{
    U1 portID;
    U1 reserved1;
    X2 txReady;
    X4 mode;
    U1 reserved2[4];
    X2 inProtoMask;
    X2 outProtoMask;
    X2 flags;
    U1 reserved3[2];
}) LD2BRM_StpPortSetDdcPayload;

typedef void (*LD2BRM_StpPortPoll)(LD2BRM_StpPortPollPayload &payload);
typedef void (*LD2BRM_StpPortSetUart)(LD2BRM_StpPortSetUartPayload &payload);
typedef void (*LD2BRM_StpPortSetUsb)(LD2BRM_StpPortSetUsbPayload &payload);
typedef void (*LD2BRM_StpPortSetSpi)(LD2BRM_StpPortSetSpiPayload &payload);
typedef void (*LD2BRM_StpPortSetDdc)(LD2BRM_StpPortSetDdcPayload &payload);


/** @brief BRM-STP-RATE (0x06 0x08)
*
*/
PACK(
typedef struct
{
    U2 measRate;
    U2 navRate;
    U2 timeRef;
}) LD2BRM_StpRateSetPayload;

typedef void (*LD2BRM_StpRateSet)(LD2BRM_StpRateSetPayload &payload);


/** @brief BRM-STP-RST (0x06 0x04)
*
*/
PACK(
typedef struct
{
    X2 navBbrMask;
    U1 resetMode;
    U1 reserved1;
}) LD2BRM_StpResetCommandPayload;

typedef void (*LD2BRM_StpResetCommand)(LD2BRM_StpResetCommandPayload &payload);


/** @brief BRM-STP-RXM (0x06 0x11)
*
*/
PACK(
typedef struct
{
    U1 reserved1;
    U1 lpMode;
}) LD2BRM_StpRxmSetPayload;

typedef void (*LD2BRM_StpRxmSet)(LD2BRM_StpRxmSetPayload &payload);

/** @brief BRM-STP-LTE (0x06 0xA0)
*
*/
PACK(
typedef struct
{
}) LD2BRM_StpLtePollPayload;

PACK(
typedef struct
{
    U1 lteFilterEn;
}) LD2BRM_StpLteSetPayload;

typedef LD2BRM_StpLteSetPayload LD2BRM_StpLteGetPayload;
typedef void(*LD2BRM_StpLteGet)(LD2BRM_StpLteGetPayload &payload);

/** @brief BRM-STP-BIAS (0x06 0xB0)
*
*/
PACK(
typedef struct
{
}) LD2BRM_StpBiasPollPayload;

PACK(
typedef struct
{
    U4 biasL5;
}) LD2BRM_StpBiasSetPayload;

typedef LD2BRM_StpBiasSetPayload LD2BRM_StpBiasGetPayload;
typedef void(*LD2BRM_StpBiasGet)(LD2BRM_StpBiasGetPayload &payload);

/** @brief BRM-CFG-LNA
 *
 */
PACK(
typedef struct
{
}) LD2BRM_StpLnaPollPayload;

PACK(
typedef struct
{
    U1 LnaMask;
}) LD2BRM_StpLnaSetPayload;

typedef LD2BRM_StpLnaSetPayload LD2BRM_StpLnaGetPayload;
typedef void(*LD2BRM_StpLnaGet)(LD2BRM_StpLnaGetPayload &payload);


/** @brief BRM-STP-FCTTEST (0x06 0xB2)
*
*/
PACK(
typedef struct
{
}) LD2BRM_StpFctTestPollPayload;

PACK(
typedef struct
{
    U1 gnssId;
    U1 svId;
    U1 flags;
}) LD2BRM_StpFctTestConfigBlockType;

PACK(
typedef struct
{
    U1 numConfigBlocks;
    LD2BRM_StpFctTestConfigBlockType configBlocks[LD2BRM_MAX_FCTTEST_CONFIG_BLOCKS];
}) LD2BRM_StpFctTestSetPayload;

typedef LD2BRM_StpFctTestSetPayload LD2BRM_StpFctTestGetPayload;

typedef void(*LD2BRM_StpFctTestGet)(LD2BRM_StpFctTestGetPayload &payload);


/** @brief BRM-STP-PPS (0x06 0x31)
*
*/
PACK(
typedef struct
{
    U1 tpIdx;
    U1 version;
    U1 reserved1[6];
    U4 freqPeriod;
    U1 reserved2[4];
    U4 pulseLenRatio;
    U1 reserved3[8];
    U4 flags;
}) LD2BRM_StpPpsPayload;

typedef void(*LD2BRM_StpPpsGet)(LD2BRM_StpPpsPayload &payload);


//////////////////////////////////////////////////////////////////
// BRM Class INF
//   Information Messages
//////////////////////////////////////////////////////////////////

/** @brief BRM-INF-DEBUG (0x04 0x04)
*
*/
PACK(
typedef struct
{
    CH str[LD2BRM_MAX_WNG_MESSAGE];
}) LD2BRM_WngDebugOutputPayload;

typedef void (*LD2BRM_WngDebugOutput)(LD2BRM_WngDebugOutputPayload &payload);


/** @brief BRM-INF-ERROR (0x04 0x00)
*
*/
PACK(
typedef struct
{
    CH str[LD2BRM_MAX_WNG_MESSAGE];
}) LD2BRM_WngErrorOutputPayload;

typedef void (*LD2BRM_WngErrorOutput)(LD2BRM_WngErrorOutputPayload &payload);


/** @brief BRM-INF-NOTICE (0x04 0x02)
*
*/
PACK(
typedef struct
{
    CH str[LD2BRM_MAX_WNG_MESSAGE];
}) LD2BRM_WngNoticeOutputPayload;

typedef void (*LD2BRM_WngNoticeOutput)(LD2BRM_WngNoticeOutputPayload &payload);


/** @brief BRM-INF-TEST (0x04 0x03)
*
*/
PACK(
typedef struct
{
    CH str[LD2BRM_MAX_WNG_MESSAGE];
}) LD2BRM_WngTestOutputPayload;

typedef void (*LD2BRM_WngTestOutput)(LD2BRM_WngTestOutputPayload &payload);


/** @brief BRM-INF-WARNING (0x04 0x01)
*
*/
PACK(
typedef struct
{
    CH str[LD2BRM_MAX_WNG_MESSAGE];
}) LD2BRM_WngWarningOutputPayload;

typedef void (*LD2BRM_WngWarningOutput)(LD2BRM_WngWarningOutputPayload &payload);



//////////////////////////////////////////////////////////////////
// BRM Class LOG    
//   Logging Messages
//////////////////////////////////////////////////////////////////


/** @brief BRM-LOG-BATCH (0x21 0x11)
*
*/
PACK(
typedef struct
{
    U1 version;
    X1 contentValid;
    U2 msgCnt;
    U4 iTOW;
    U2 year;
    U1 month;
    U1 day;
    U1 hour;
    U1 min;
    U1 sec;
    X1 valid;
    U4 tAcc;
    I4 fracSec;
    U1 fixType;
    X1 flags;
    X1 flags2;
    U1 numSV;
    I4 lon;
    I4 lat;
    I4 height;
    I4 hMSL;
    U4 hAcc;
    U4 vAcc;
    I4 velN;
    I4 velE;
    I4 velD;
    I4 gSpeed;
    I4 headMot;
    U4 sAcc;
    U4 headAcc;
    U2 pDOP;
    I1 leapS;
    U1 reserved1[1];
    U4 distance;
    U4 totalDistance;
    U4 distanceStd;
    U1 reserved2[4];
}) LD2BRM_LogBatchPolledPayload;

typedef void (*LD2BRM_LogBatchPolled)(LD2BRM_LogBatchPolledPayload &payload);


/** @brief BRM-LOG-CREATE (0x21 0x07)
*
*/
PACK(
typedef struct
{
    U1 version;
    X1 logCfg;
    U1 reserved1;
    U1 logSize;
    U4 userDefinedSize;
}) LD2BRM_LogCreateCommandPayload;

typedef void (*LD2BRM_LogCreateCommand)(LD2BRM_LogCreateCommandPayload &payload);


/** @brief BRM-LOG-ERASE (0x21 0x03)
*
*/
PACK(
typedef struct
{
}) LD2BRM_LogEraseCommandPayload;

typedef void (*LD2BRM_LogEraseCommand)(LD2BRM_LogEraseCommandPayload &payload);


/** @brief BRM-LOG-FINDTIME (0x21 0x0E)
*
*/
PACK(
typedef struct
{
    U1 version;
    U1 type;
    U1 reserved1[2];
    U2 year;
    U1 month;
    U1 day;
    U1 hour;
    U1 minute;
    U1 second;
    U1 reserved2;
}) LD2BRM_LogFindtimeInputPayload;

PACK(
typedef struct
{
    U1 version;
    U1 type;
    U1 reserved1[2];
    U4 entryNumber;
}) LD2BRM_LogFindtimeOutputPayload;

typedef void (*LD2BRM_LogFindtimeInput)(LD2BRM_LogFindtimeInputPayload &payload);
typedef void (*LD2BRM_LogFindtimeOutput)(LD2BRM_LogFindtimeOutputPayload &payload);


/** @brief BRM-LOG-INFO (0x21 0x08)
*
*/
PACK(
typedef struct
{

}) LD2BRM_LogInfoPollPayload;

PACK(
typedef struct
{
    U1 version;
    U1 reserved1[3];
    U4 filestoreCapacity;
    U1 reserved2[8];
    U4 currentMaxLogSize;
    U4 currentLogSize;
    U4 entryCount;
    U2 oldestYear;
    U1 oldestMonth;
    U1 oldestDay;
    U1 oldestHour;
    U1 oldestMinute;
    U1 oldestSecond;
    U1 reserved3;
    U2 newestYear;
    U1 newestMonth;
    U1 newestDay;
    U1 newestHour;
    U1 newestMinute;
    U1 newestSecond;
    U1 reserved4;
    X1 status;
    U1 reserved5[3];
}) LD2BRM_LogInfoOutputPayload;

typedef void (*LD2BRM_LogInfoPoll)(LD2BRM_LogInfoPollPayload &payload);
typedef void (*LD2BRM_LogInfoOutput)(LD2BRM_LogInfoOutputPayload &payload);


/** @brief BRM-LOG-RETRIEVEBATCH (0x21 0x10)
*
*/
PACK(
typedef struct
{
    U1 version;
    X1 flags;
    U1 reserved1[2];
}) LD2BRM_LogRetrievebatchCommandPayload;

typedef void (*LD2BRM_LogRetrievebatchCommand)(LD2BRM_LogRetrievebatchCommandPayload &payload);


/** @brief BRM-LOG-RETRIEVEPOSEXTRA (0x21 0x0f)
*
*/
PACK(
typedef struct
{
    U4 entryIndex;
    U1 version;
    U1 reserved1;
    U2 year;
    U1 month;
    U1 day;
    U1 hour;
    U1 minute;
    U1 second;
    U1 reserved2[3];
    U4 distance;
    U1 reserved3[12];
}) LD2BRM_LogRetrieveposextraOutputPayload;

typedef void (*LD2BRM_LogRetrieveposextraOutput)(LD2BRM_LogRetrieveposextraOutputPayload &payload);


/** @brief BRM-LOG-RETRIEVEPOS (0x21 0x0b)
*
*/
PACK(
typedef struct
{
    U4 entryIndex;
    I4 lon;
    I4 lat;
    I4 hMSL;
    U4 hAcc;
    U4 gSpeed;
    U4 heading;
    U1 version;
    U1 fixType;
    U2 year;
    U1 month;
    U1 day;
    U1 hour;
    U1 minute;
    U1 second;
    U1 reserved1;
    U1 numSV;
    U1 reserved2;
}) LD2BRM_LogRetrieveposOutputPayload;

typedef void (*LD2BRM_LogRetrieveposOutput)(LD2BRM_LogRetrieveposOutputPayload &payload);


/** @brief BRM-LOG-RETRIEVESTRING (0x21 0x0d)
*
*/
PACK(
typedef struct
{
    U4 entryIndex;
    U1 version;
    U1 reserved1;
    U2 year;
    U1 month;
    U1 day;
    U1 hour;
    U1 minute;
    U1 second;
    U1 reserved2;
    U2 byteCount;
    U1 bytes[LD2BRM_MAX_RETRIEVESTRING];
}) LD2BRM_LogRetrievestringOutputPayload;

typedef void (*LD2BRM_LogRetrievestringOutput)(LD2BRM_LogRetrievestringOutputPayload &payload);


/** @brief BRM-LOG-RETRIEVE (0x21 0x09)
*
*/
PACK(
typedef struct
{
    U4 startNumber;
    U4 entryCount;
    U1 version;
    U1 reserved1[3];
}) LD2BRM_LogRetrieveCommandPayload;

typedef void (*LD2BRM_LogRetrieveCommand)(LD2BRM_LogRetrieveCommandPayload &payload);



//////////////////////////////////////////////////////////////////
// BRM Class MGA    
//   Multiple GNSS Assistance Messages
//////////////////////////////////////////////////////////////////

/** @brief BRM-MGA-ACK (0x13 0x60)
*
*/
PACK(
typedef struct
{
    U1 type;
    U1 version;
    U1 infoCode;
    U1 msgId;
    U1 msgPayloadStart[4];
}) LD2BRM_AstAckOutputPayload;

typedef void (*LD2BRM_AstAckOutput)(LD2BRM_AstAckOutputPayload &payload);


/** @brief BRM-MGA-ANO (0x13 0x20)
*
*/
PACK(
typedef struct
{
    U1 type;
    U1 version;
    U1 svId;
    U1 gnssId;
    U1 year;
    U1 month;
    U1 day;
    U1 reserved1;
    U1 data[64];
    U1 reserved2[4];
}) LD2BRM_AstLtoInputPayload;

typedef void (*LD2BRM_AstLtoInput)(LD2BRM_AstLtoInputPayload &payload);



/** @brief BRM-MGA-DBD (0x13 0x80)
*
*/
PACK(
typedef struct
{

}) LD2BRM_AstNvmemPollPayload;

PACK(
typedef struct
{
    U1 reserved1[12];
    U1 data[LD2BRM_MAX_AST_NVMEM_DUMP];
}) LD2BRM_AstNvmemInputPayload;

PACK(
typedef struct
{
    U1 reserved1[12];
    U1 data[LD2BRM_MAX_AST_NVMEM_DUMP];
    U2 num;
}) LD2BRM_AstNvmemOutputPayload;

typedef void (*LD2BRM_AstNvmemPollRequest)(LD2BRM_AstNvmemPollPayload &payload);
typedef void (*LD2BRM_AstNvmemInput)(LD2BRM_AstNvmemInputPayload &payload);
typedef void (*LD2BRM_AstNvmemOutput)(LD2BRM_AstNvmemOutputPayload &payload);



/** @brief BRM-MGA-INI (0x13 0x40)
*
*/
PACK(
typedef struct
{
    U1 type;
    U1 version;
    U1 reserved1[2];
    I4 ecefX;
    I4 ecefY;
    I4 ecefZ;
    U4 posAcc;
}) LD2BRM_AstIniPosXyzInputPayload;

PACK(
typedef struct
{
    U1 type;
    U1 version;
    U1 reserved1[2];
    I4 lat;
    I4 lon;
    I4 alt;
    U4 posAcc;
}) LD2BRM_AstIniPosLlhInputPayload;

PACK(
typedef struct
{
    U1 type;
    U1 version;
    X1 ref;
    I1 leapSecs;
    U2 year;
    U1 month;
    U1 day;
    U1 hour;
    U1 minute;
    U1 second;
    U1 reserved1;
    U4 ns;
    U2 tAccS;
    U1 reserved2[2];
    U4 tAccNs;
}) LD2BRM_AstRefTimeUtcInputPayload;

PACK(
typedef struct
{
    U1 type;
    U1 version;
    X1 ref;
    U1 gnssId;
    U1 reserved1[2];
    U2 week;
    U4 tow;
    U4 ns;
    U2 tAccS;
    U1 reserved2[2];
    U4 tAccNs;
}) LD2BRM_AstIniTimeGnssInputPayload;

PACK(
typedef struct
{
    U1 type;
    U1 version;
    U1 reserved1[2];
    I4 clkD;
    U4 clkDAcc;
}) LD2BRM_AstIniClkdInputPayload;

typedef void (*LD2BRM_AstIniPosXyzInput)(LD2BRM_AstIniPosXyzInputPayload &payload);
typedef void (*LD2BRM_AstIniPosLlhInput)(LD2BRM_AstIniPosLlhInputPayload &payload);
typedef void (*LD2BRM_AstRefTimeUtcInput)(LD2BRM_AstRefTimeUtcInputPayload &payload);
typedef void (*LD2BRM_AstIniTimeGnssInput)(LD2BRM_AstIniTimeGnssInputPayload &payload);
typedef void (*LD2BRM_AstIniClkdInput)(LD2BRM_AstIniClkdInputPayload &payload);


//////////////////////////////////////////////////////////////////
// BRM Class MON
//   Monitoring Messages
//////////////////////////////////////////////////////////////////

/** @brief BRM-MON-BATCH (0x0A 0x32)
*
*/
PACK(
typedef struct
{
    U1 version;
    U1 reserved1[3];
    U2 fillLevel;
    U2 dropsAll;
    U2 dropsSinceMon;
    U2 nextMsgCnt;
}) LD2BRM_StsBatchPolledPayload;

typedef void (*LD2BRM_StsBatchPolled)(LD2BRM_StsBatchPolledPayload &payload);


/** @brief BRM-MON-GNSS (0x0A 0x28)
*
*/
PACK(
typedef struct
{
    U1 version;
    X1 supported;
    X1 defaultGnss;
    X1 enabled;
    U1 simultaneous;
    U1 reserved1[3];
}) LD2BRM_StsGnssPolledPayload;

typedef void (*LD2BRM_StsGnssPolled)(LD2BRM_StsGnssPolledPayload &payload);


/** @brief BRM-MON-HW (0x0A 0x09)
*
*/
PACK(
typedef struct
{
    X4 pinSel;
    X4 pinBank;
    X4 pinDir;
    X4 pinVal;
    U2 noisePerMS;
    U2 agcCnt;
    U1 aStatus;
    U1 aPower;
    X1 flags;
    U1 reserved1;
    X4 usedMask;
    U1 VP[17];
    U1 jamInd;
    U1 reserved2[2];
    X4 pinIrq;
    X4 pullH;
    X4 pullL;
}) LD2BRM_StsHwPolledPayload;

typedef void (*LD2BRM_StsHwPolled)(LD2BRM_StsHwPolledPayload &payload);


/** @brief BRM-MON-IO (0x0A 0x02)
*
*/
PACK(
typedef struct
{
    U4 rxBytes;
    U4 txBytes;
    U2 parityErrs;
    U2 framingErrs;
    U2 overrunErrs;
    U2 breakCond;
    U1 rxBusy;
    U1 txBusy;
    U1 reserved1[2];
}) LD2BRM_StsIoStatus;

PACK(
typedef struct
{
    LD2BRM_StsIoStatus port[LD2BRM_MAX_STS_IO_STATUS];
    U4 num;   /* not part of payload */
}) LD2BRM_StsIoPolledPayload;

typedef void (*LD2BRM_StsIoPolled)(LD2BRM_StsIoPolledPayload &payload);


/** @brief BRM-MON-RXR (0x0A 0x21)
*
*/
PACK(
    typedef struct
{
    X1 flags;
}) LD2BRM_StsRxrOutputPayload;

typedef void(*LD2BRM_StsRxrOutput)(LD2BRM_StsRxrOutputPayload &payload);


/** @brief BRM-MON-VER (0x0A 0x04)
*
*/
PACK(
typedef struct
{
}) LD2BRM_StsVerPollPayload;

PACK(
typedef struct
{
    CH extension[30];
}) LD2BRM_StsVerExtension;

PACK(
typedef struct
{
    CH swVersion[30];
    CH hwVersion[10];
    LD2BRM_StsVerExtension extension[LD2BRM_MAX_STS_VER_EXTENSION];
    U4 num; /* not part of payload */
}) LD2BRM_StsVerPolledPayload;

typedef void (*LD2BRM_StsVerPoll)(LD2BRM_StsVerPollPayload &payload);
typedef void (*LD2BRM_StsVerPolled)(LD2BRM_StsVerPolledPayload &payload);





//////////////////////////////////////////////////////////////////
// BRM Class NAV
//   Navigation Results Messages
//////////////////////////////////////////////////////////////////


/** @brief BRM-NAV-CLOCK (0x01 0x22)
*
*/
PACK(
typedef struct
{
    U4 iTOW;
    I4 clkB;
    I4 clkD;
    U4 tAcc;
    U4 fAcc;
}) LD2BRM_PvtClockPolledPayload;

typedef void (*LD2BRM_PvtClockPolled)(LD2BRM_PvtClockPolledPayload &payload);


/** @brief BRM-NAV-DOP (0x01 0x04)
*
*/
PACK(
typedef struct
{
    U4 iTOW;
    U2 gDOP;
    U2 pDOP;
    U2 tDOP;
    U2 vDOP;
    U2 hDOP;
    U2 nDOP;
    U2 eDOP;
}) LD2BRM_PvtDopPolledPayload;

typedef void (*LD2BRM_PvtDopPolled)(LD2BRM_PvtDopPolledPayload &payload);


/** @brief BRM-NAV-EOE (0x01 0x61)
*
*/
PACK(
typedef struct
{
    U4 iTOW;
}) LD2BRM_PvtEoePeriodicPayload;

typedef void (*LD2BRM_PvtEoePeriodic)(LD2BRM_PvtEoePeriodicPayload &payload);


/** @brief BRM-NAV-GEOFENCE (0x01 0x39)
*
*/
PACK(
typedef struct
{
    U1 state;
    U1 reserved1[1];
}) LD2BRM_PvtGeofenceState;

PACK(
typedef struct
{
    U4 iTOW;
    U1 version;
    U1 status;
    U1 numFences;
    U1 combState;
    LD2BRM_PvtGeofenceState state[LD2BRM_MAX_GEOFENCES];
}) LD2BRM_PvtGeofencePolledPayload;

typedef void (*LD2BRM_PvtGeofencePolled)(LD2BRM_PvtGeofencePolledPayload &payload);


/** @brief BRM-NAV-ODO (0x01 0x09)
*
*/
PACK(
typedef struct
{
    U1 version;
    U1 reserved1[3];
    U4 iTOW;
    U4 distance;
    U4 totalDistance;
    U4 distanceStd;
}) LD2BRM_PvtAccDistancePolledPayload;

typedef void (*LD2BRM_PvtAccDistancePolled)(LD2BRM_PvtAccDistancePolledPayload &payload);


/** @brief BRM-NAV-ORB (0x01 0x34)
*
*/
PACK(
typedef struct
{
    U1 gnssId;
    U1 svId;
    X1 svFlag;
    X1 eph;
    X1 alm;
    X1 otherOrb;
}) LD2BRM_PvtOrbDatabaseInfo;

PACK(
typedef struct 
{
    U4 iTOW;
    U1 version;
    U1 numSv;
    U1 reserved1[2];
    LD2BRM_PvtOrbDatabaseInfo OrbInfo[LD2BRM_MAX_ORB_DB_INFO];
}) LD2BRM_PvtOrbPolledPayload;

typedef void (*LD2BRM_PvtOrbPolled)(LD2BRM_PvtOrbPolledPayload &payload);


/** @brief BRM-NAV-PVT (0x01 0x07)
*
*/
PACK(
typedef struct {
    U4 iTOW;
    U2 year;
    U1 month;
    U1 day;
    U1 hour;
    U1 min;
    U1 sec;
    X1 valid;
    U4 tAcc;
    I4 nano;
    U1 fixType;
    X1 flags;
    X1 flags2;
    U1 numSV;
    I4 lon;
    I4 lat;
    I4 height;
    I4 hMSL;
    U4 hAcc;
    U4 vAcc;
    I4 velN;
    I4 velE;
    I4 velD;
    I4 gSpeed;
    I4 headMot;
    U4 sAcc;
    U4 headAcc;
    U2 pDOP;
    I1 leapS;
    U1 reserved1[5];
    I4 headVeh;
    I2 magDec;
    U2 magAcc;
}) LD2BRM_PvtPvtPolledPayload;

typedef void (*LD2BRM_Pvt_PVT_Polled)(LD2BRM_PvtPvtPolledPayload &payload);


/** @brief BRM-NAV-RESETODO (0x01 0x10)
*
*/
PACK(
typedef struct
{

}) LD2BRM_PvtResetAccDistanceCommandPayload;
typedef void (*LD2BRM_PvtResetAccDistanceCommand)(LD2BRM_PvtResetAccDistanceCommandPayload &payload);


/** @brief BRM-NAV-SAT (0x01 0x35)
*
*/
PACK(
typedef struct {
    U1 gnssId;
    U1 svId;
    U1 cno;
    I1 elev;
    I2 azim;
    I2 prRes;
    X4 flags;
}) LD2BRM_PvtSatSvInformation;

PACK(
typedef struct {
    U4 iTOW;
    U1 version;
    U1 numSvs;
    U1 reserved1[2];
    LD2BRM_PvtSatSvInformation Svs[LD2BRM_MAX_CHANNEL];
}) LD2BRM_PvtSatPolledPayload;

typedef void (*LD2BRM_PvtSatPolled)(LD2BRM_PvtSatPolledPayload &payload);


/** @brief BRM-NAV-STATUS (0x01 0x03)
*
*/
PACK(
typedef struct {
    U4 iTOW;
    U1 gpsFix;
    X1 flags;
    X1 fixStat;
    X1 flags2;
    U4 ttff;
    U4 msss;
}) LD2BRM_PvtStatusPolledPayload;

typedef void (*LD2BRM_PvtStatusPolled)(LD2BRM_PvtStatusPolledPayload &payload);


/** @brief BRM-NAV-SVINFO (0x01 0x30)
*
*/
PACK(
typedef struct {
    U1 chn;
    U1 svid;
    X1 flags;
    X1 quality;
    U1 cno;
    I1 elev;
    I2 azim;
    I4 prRes;
}) LD2BRM_PvtSvInfoInfo;

PACK(
typedef struct {
    U4 iTOW;
    U1 numCh;
    X1 globalFlags;
    U1 reserved1[2];
    LD2BRM_PvtSvInfoInfo SvInfo[LD2BRM_MAX_PVT_SVINFO_INFO];
}) LD2BRM_PvtSvInfoPolledPayload;

typedef void (*LD2BRM_PvtSvInfoPolled)(LD2BRM_PvtSvInfoPolledPayload &payload);


/** @brief BRM-NAV-VELNED (0x01 0x12)
*
*/
PACK(
typedef struct {
    U4 iTOW;
    I4 velN;
    I4 velE;
    I4 velD;
    U4 speed;
    U4 gSpeed;
    I4 heading;
    U4 sAcc;
    U4 cAcc;
}) LD2BRM_PvtVelNedPolledPayload;

typedef void (*LD2BRM_PvtVelNedPolled)(LD2BRM_PvtVelNedPolledPayload &payload);


/** @brief BRM-PVT-CBEE_STATUS (0x01 0x60)
*
*/
PACK(
typedef struct {
    U4 iTOW;
    U1 cbeeCfg;
    U1 status;
    U1 reserved1[10];
}) LD2BRM_PvtCbeeStatusPayload;

typedef void(*LD2BRM_PvtCbeeStatus)(LD2BRM_PvtCbeeStatusPayload &payload);


/** @brief BRM-ASC-PMREQ (0x02 0x41)
*
*/
PACK(
typedef struct {
    U1 version;
    U1 reserved1[3];
    U4 duration;
    X4 flags;
    X4 wakeupSources;
}) LD2BRM_AscPwrCycleCommandPayload;

/** @brief BRM-ASC-SUBFRAMES (0x02 0x13)
*
*/
PACK(
typedef struct {
    U1 gnssId;
    U1 svId;
    U1 reserved1;
    U1 freqId;
    U1 numWords;
    U1 reserved2;
    U1 version;
    U1 reserved3;
    U4 dataWords[LD2BRM_MAX_SUBFRAME_DATA_WORDS];
}) LD2BRM_AscSubframesPolledPayload;

typedef void(*LD2BRM_AscSubframesPolled)(LD2BRM_AscSubframesPolledPayload &payload);

/** @breif BRM-ASC-MEAS (0x02 0x15)
*
*/

PACK(
typedef struct {
    R8 prMes;
    R8 cpMes;
    R4 doMes;
    U1 gnssId;
    U1 svId;
    U1 sigId;
    U1 freqId;
    U2 locktime;
    U1 cn0;
    X1 prStdev;
    X1 cpStdev;
    X1 doStdev;
    X1 trkStat;
    U1 extra;
}) LD2BRM_RawMeas;

PACK(
typedef struct {
    R8 rcvTow;
    U2 week;
    I1 leapS;
    U1 numMeas;
    X1 recStat;
    U1 version;
    U1 reserved1[2];
    LD2BRM_RawMeas meas[LD2BRM_MAX_CHANNEL];
}) LD2BRM_AscMeasPolledPayload;

typedef void(*LD2BRM_AscMeasPolled)(LD2BRM_AscMeasPolledPayload &payload);

/** @brief BRM-ASC-AGC (0x02 0x80)
*
*/
PACK(
typedef struct {
    U1 gnssId;
    U1 svId;
    U1 sigId;
    U1 freqId;
    I2 AGCdB;
}) LD2BRM_AscAgcContent;

PACK(
typedef struct {
    U4 iTow;
    U1 numMeas;
    U1 reseved1[3];
    LD2BRM_AscAgcContent contentBlocks[LD2BRM_MAX_CHANNEL];
}) LD2BRM_AscAgcPolledPayload;

typedef void(*LD2BRM_AscAgcPolled)(LD2BRM_AscAgcPolledPayload &payload);


//////////////////////////////////////////////////////////////////
// BRM Class BUP
//   Nvmem backup control messages
//////////////////////////////////////////////////////////////////

/** @brief BRM-BUP (0x09 0x14)
*
*/

PACK(
typedef struct {

}) LD2BRM_BupPollPayload;

PACK(
typedef struct {
    U1 cmd;
    U1 reserved1[3];
}) LD2BRM_BupCommandPayload;

PACK(
typedef struct {
    U1 cmd;
    U1 reserved1[3];
    U1 response;
    U1 reserved2[3];
}) LD2BRM_BupPolledPayload;

typedef void(*LD2BRM_BupPolled)(LD2BRM_BupPolledPayload &payload);

#endif  // BREAM_H
