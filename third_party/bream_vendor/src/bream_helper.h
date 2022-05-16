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
/** @file bream_helper.h  
*============================================================================*/

#include "bream_handler.h"
#include "timegm.h"
#include "circ_buf.h"
#include <stdint.h>
#ifndef BREAM__HELPER_H
#define BREAM__HELPER_H

#ifdef SDK_OS_FREE_RTOS
#define NVMEM_FILE_NAME "/nvmem.dat"
#else
#define NVMEM_FILE_NAME "nvmem.dat"
#endif
enum RAW_MSMT_STATE {
    RAW_MSMT_IDLE,
    RAW_MSMT_START,
    RAW_MSMT_STORING,
    RAW_MSMT_STOP,
};

struct BreamAstData;
class BreamHelper
{
public:
    BreamHelper();

    static BreamHelper& GetInstance(void);
    void GnssStart(void);
    void GnssStop(void);

#define MAX_GNSS_ID 8
    int ReadLtoSlice(void *fd, LD2BRM_AstLtoInputPayload* p);

    void InjectLto(const char *path);
    void InjectTime(uint16_t year, uint16_t month, uint16_t day, uint16_t hour, uint16_t min, uint16_t sec, uint16_t accSec, uint32_t accNSec, uint8_t leapSec=18);
    void InjectRefLocation(double lat, double lon, double alt, double accuracy); //degree, degree, meter, meter
    void SetOsc(uint32_t freqHz, uint16_t calUncPpb, uint16_t uncalUncPpb);
    void SetMsgRate(uint8_t cls, uint8_t id, uint8_t rate);
    void SetAckAiding(bool enable);
    void SetLogging(uint8_t(&infMsgMask)[6]);
    void SetGnss(uint8_t enabledGnss, uint8_t maxChnNum, bool enableL5);
    void SetPowerMode(uint8_t value, uint16_t period = 0, uint16_t onTime = 0);
    void ResetOdometer(void);
    void EnterSuspend(uint32_t duration, uint32_t flags=0, uint32_t wakeupSources=0);
    void SetLteFilterEn(uint8_t en);
    void GetLteFilterEn(void);
    void SetL5Bias(uint32_t biasCm);
    void GetL5Bias(void);
    void SetBaudrate(uint32_t baudrate, uint8_t portid = 1, uint16_t outprotomask = 3);
    void PullNvmem(void);
    void PushNvmem(void);
    void CreateNvmemBackup(void);
    void ClearNvmemBackup(void);
    void RestoreNvmemFromBackup(void);
    const char* GetNameOfBreamPacket(uint8_t cls, uint8_t id);
    bool SetPowerModePreset(uint8_t rfMode, uint8_t pwrMode);
    void WaitAck(uint16_t clsid);
    void NotifyAckNack(int acknack, uint16_t clsid);
    void GetVer(void);
    void GnssReset(uint16_t navBbrMask);

    bool PushMultiAstData(BreamAstData &astData);
    void StartSavingRawMsmg();
    void StopSavingRawMsmg(void);
    void EnableBlindGalSearch(void);
    void AddRawdataCb(LD2BRM_RawData cb);
    void RemoveRawdataCb(LD2BRM_RawData cb);
    static uint16_t ClsId(uint8_t* buf);
    static uint16_t PayloadLen(uint8_t* buf);

    time_t m_refTime;
    int m_ackNack; //0 waiting 1 ack 2 nak
    uint16_t m_ackNackClsid;
    circ_buf m_respCircBuf;
    char m_respBuf[512];
    void* m_fdRawMsmt;
    bool m_bRawMsmt;
#define MAX_CALLBACK 3
    int m_numCbs;
    LD2BRM_RawData m_rawdataCallbacks[MAX_CALLBACK];
};

#endif  // BREAM__HELPER_H
