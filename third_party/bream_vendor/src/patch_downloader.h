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
/** @file lodi2_asic.h
*============================================================================*/

#ifndef LODI2_ASIC_H
#define LODI2_ASIC_H

#include "lodi2_os_glue.h"
#include "lodi2_os_glue.h"

#define DOWNLOAD_TIMEOUT_MS 10000

typedef void(*PatchDownloadProgressCb)(int percent);

bool Bream_LoadPatch(const char *patch, PatchDownloadProgressCb cb = 0, char *tty=nullptr);

#endif
