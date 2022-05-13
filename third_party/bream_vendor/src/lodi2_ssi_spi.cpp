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
/** @file lodi2_ssi_spi.cpp  
*============================================================================*/

#include <string.h>
#include "lodi2_ssi_spi.h"

static struct bcm_ssi_tx_frame priv_tx_buf;
static struct bcm_ssi_rx_frame priv_rx_buf;

bool LD2SSI_packTxBuff(uint8_t *in_buff, uint32_t in_length, uint8_t **out_buff, uint32_t *out_length)
{
	struct bcm_ssi_tx_frame *tx = &priv_tx_buf;
	uint16_t frame_len 			= MAX_SPI_TX_FRAME_LEN;
	uint8_t ctrl_byte 			= SSI_MODE_HALF_DUPLEX | SSI_MODE_STREAM | SSI_PCKT_2B_LENGTH | SSI_WRITE_TRANS | SSI_FLOW_CONTROL_DISABLED;
	int fc_len 					= 0;
	int ctrl_len 				= fc_len + 1; 	// 1 for tx cmd byte
	uint16_t frame_data_size 	= frame_len - ctrl_len;
	uint16_t bytes_to_write 	= (uint16_t)in_length;

	tx->cmd = ctrl_byte;

    if (bytes_to_write > frame_data_size)
    {
    	//TODO: Assert!
    	return false;
    }

	memcpy(&tx->data[0], in_buff, bytes_to_write);

	*out_buff = (uint8_t *)tx;
	*out_length = (bytes_to_write + ctrl_len);

    return true;
}

uint32_t LD2SSI_packRxCtrl(uint8_t **tx_ssi_buff, uint8_t **rx_ssi_buff)
{
	struct bcm_ssi_tx_frame *tx = &priv_tx_buf;
	struct bcm_ssi_rx_frame *rx = &priv_rx_buf;
	uint8_t ctrl_byte 			= SSI_MODE_HALF_DUPLEX | SSI_MODE_STREAM | SSI_PCKT_2B_LENGTH | SSI_READ_TRANS;
	int pckt_len 				= 2;						// 2 bytes length
	int fc_len 					= 0;
	int ctrl_len 				= pckt_len + fc_len + 1; 	// 1 for tx cmd byte

	tx->cmd = ctrl_byte;                       // SSI_READ_HD etc.

	tx->data[0] = 0;
	tx->data[1] = 0;

	*tx_ssi_buff = (uint8_t *)tx;
	*rx_ssi_buff = (uint8_t *)rx;

	return ctrl_len;
}

uint32_t LD2SSI_getRxLength(uint8_t *rx_buff)
{
	uint8_t ctrl_byte 	= SSI_MODE_HALF_DUPLEX | SSI_MODE_STREAM | SSI_PCKT_2B_LENGTH | SSI_READ_TRANS;
	uint32_t len = 0;

	// Case: CS8306443
	// Check if MISO floats high and then return error with 0.
	if (rx_buff[0] == 0xff && rx_buff[1] == 0xff && rx_buff[2] == 0xff)
	{
		return 0;
	}

	if ( ctrl_byte & SSI_PCKT_2B_LENGTH ) {
		len = ((uint16_t)rx_buff[1] + ((uint16_t)rx_buff[2] << 8));
	} else {
		len = (uint16_t)rx_buff[1];
	}

	return len;
}

uint32_t LD2SSI_unpackRxBuff(uint8_t *unpacked_buff, uint32_t read_len, uint8_t *rx_buff)
{
	int pckt_len 				= 2;						// 2 bytes length
	int fc_len 					= 0;
	int ctrl_len 				= pckt_len + fc_len + 1; 	// 1 for tx cmd byte
	uint32_t payload_len;
	uint32_t unpacked_len;

	payload_len = LD2SSI_getRxLength(rx_buff);

	unpacked_len = (payload_len > read_len) ? read_len : payload_len;
	memcpy(unpacked_buff, rx_buff+ctrl_len, unpacked_len);

	return unpacked_len;
}
