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
/** @file bream_endian.h
*============================================================================*/

#ifndef BREAM_ENDIAN_H
#define BREAM_ENDIAN_H

#define LD2_HOST_BIG_ENDIAN		(_LD2_isHostBigEndian())
#define LD2_HOST_LITTLE_ENDIAN	(!_LD2_isHostBigEndian())

bool _LD2_isHostBigEndian(void);

#define _LD2_SWAP16(x) \
	((((x) >> 8) & 0xffu) | (((x) & 0xffu) << 8))

#define _LD2_SWAP32(x) \
    ((((x) & 0xff000000u) >> 24) | (((x) & 0x00ff0000u) >>  8) |	      \
	(((x) & 0x0000ff00u) << 8) | (((x) & 0x000000ffu) << 24))

#define _LD2_SWAP64(x) \
     ((((x) & 0xff00000000000000ull) >> 56)				      \
      | (((x) & 0x00ff000000000000ull) >> 40)				      \
      | (((x) & 0x0000ff0000000000ull) >> 24)				      \
      | (((x) & 0x000000ff00000000ull) >> 8)				      \
      | (((x) & 0x00000000ff000000ull) << 8)				      \
      | (((x) & 0x0000000000ff0000ull) << 24)				      \
      | (((x) & 0x000000000000ff00ull) << 40)				      \
      | (((x) & 0x00000000000000ffull) << 56))

float _LD2_SwapFloat(float val);
double _LD2_SwapDouble(double val);

#ifdef DONT_SWAP

#define _LD2_TO_CHIP_ENDIAN16(x) 		(x)
#define _LD2_TO_CHIP_ENDIAN32(x) 		(x)
#define _LD2_TO_CHIP_ENDIAN32FLOAT(x) 	(x)
#define _LD2_TO_CHIP_ENDIAN64(x) 		(x)
#define _LD2_TO_CHIP_ENDIAN64DOUBLE(x) 	(x)

#define _LD2_TO_HOST_ENDIAN16(x) 		(x)
#define _LD2_TO_HOST_ENDIAN32(x) 		(x)
#define _LD2_TO_HOST_ENDIAN32FLOAT(x) 	(x)
#define _LD2_TO_HOST_ENDIAN64(x) 		(x)
#define _LD2_TO_HOST_ENDIAN64DOUBLE(x) 	(x)

#else
inline void _reverse_memcpy(void *dest, const void *src, unsigned int sz)
{
	int i = 0;
	while (sz >= 0) { ((unsigned char *)dest)[--sz] = ((unsigned char *)src)[i]; }
}

#define _LD2_TO_CHIP_ENDIAN(b, x) { \
	if (LD2_HOST_BIG_ENDIAN) _reverse_memcpy(b, &x, sizeof(x)); \
	else memcpy(b, &x, sizeof(x)); \
	b += sizeof(x); \
}
#define _LD2_TO_HOST_ENDIAN(x, b) { \
	if (LD2_HOST_BIG_ENDIAN) _reverse_memcpy(&x, b, sizeof(x)); \
	else memcpy(&x, b, sizeof(x)); \
	b += sizeof(x); \
}

#endif //DONT_SWAP

#endif //BREAM_ENDIAN_H
