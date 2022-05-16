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
/** @file bream_endian.cpp
*============================================================================*/

#include "bream_endian.h"
#include <stdint.h>

static union {
    int l;
    char c[sizeof(int)];
} const x = {1,};


bool _LD2_isHostBigEndian(void)
{
	return (x.c[0]==0);
}


float _LD2_SwapFloat(float val)
{
    float ret=val;
    uint32_t *x = (uint32_t*)&ret;
    *x = _LD2_SWAP32(*x);
    return ret;
}


double _LD2_SwapDouble(double val)
{
    double ret=val;
    uint64_t *x = (uint64_t*)&ret;
    *x = _LD2_SWAP64(*x);
    return ret;
}


