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
/** @file lodi2_ssi_spi.h  
*============================================================================*/

#ifndef LODI2_SSI_SPI_H
#define LODI2_SSI_SPI_H

#include <stdint.h>

#define SSI_MODE_STREAM			0x00
#define SSI_MODE_DEBUG			0x80

#define SSI_MODE_HALF_DUPLEX	0x00
#define SSI_MODE_FULL_DUPLEX	0x40

#define SSI_WRITE_TRANS			0x00
#define SSI_READ_TRANS			0x20

#define SSI_PCKT_1B_LENGTH        0
#define SSI_PCKT_2B_LENGTH        0x10

#define SSI_FLOW_CONTROL_DISABLED 0
#define SSI_FLOW_CONTROL_ENABLED  0x08

#define MAX_SPI_TX_FRAME_LEN 		(1024 * 4)
#define MAX_SPI_RX_FRAME_LEN 		(1024 * 2)

struct bcm_ssi_tx_frame {
	unsigned char cmd;
	unsigned char data[MAX_SPI_TX_FRAME_LEN-1];
} __attribute__((__packed__));

struct bcm_ssi_rx_frame {
	unsigned char status;
	unsigned char data[MAX_SPI_RX_FRAME_LEN-1];
} __attribute__((__packed__));

/*******************************************************************************
 * API
 ******************************************************************************/
bool LD2SSI_packTxBuff(uint8_t *in_buff, uint32_t in_length, uint8_t **out_buff, uint32_t *out_length);
uint32_t LD2SSI_packRxCtrl(uint8_t **tx_ssi_buff, uint8_t **rx_ssi_buff);
uint32_t LD2SSI_getRxLength(uint8_t *rx_buff);
uint32_t LD2SSI_unpackRxBuff(uint8_t *unpacked_buff, uint32_t read_len, uint8_t *rx_buff);

#endif // #define LODI2_SSI_SPI_H
