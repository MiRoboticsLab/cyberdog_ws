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
/** @file lodi2_os_glue_freertos.cpp
*============================================================================*/

#include "lodi2_os_glue.h"

#include <stdio.h>
#include <stdarg.h>
#include "FreeRTOS.h"
#include "fsl_debug_console.h"
#include "fsl_lpuart.h"
#include "semphr.h"
#include "task.h"
#include "board.h"
#include "fsl_gpio.h"
#include "fsl_lpuart.h"
#include "fsl_rtc.h"
#include "ff.h"
#include "string.h"
#include "spi_driver.h"
#include "host_req.h"
#ifdef SUPPORT_SPI
#include "lodi2_ssi_spi.h"
#endif

static void *g_fdLog = NULL;
static bool g_bEnabledCtsForMcuRdy = false;
static LoDi2SerialConnection g_connType = LODI2_SERIAL_UART;

class LogLock
{
public:
    LogLock()
    {
        //FIXME: bInit is not thread safe
        if (!bInit)
        {
            pLogMutex = xSemaphoreCreateRecursiveMutex();
            LD2_ASSERT(pLogMutex);
            bInit = true;
        }

        xSemaphoreTakeRecursive(pLogMutex, portMAX_DELAY);
    }
    ~LogLock()
    {
        xSemaphoreGiveRecursive(pLogMutex);
    }
    static bool bInit;
    static SemaphoreHandle_t pLogMutex;
};
bool LogLock::bInit = false;


#define LODI2_LPUART 			LPUART1
#define LODI2_UART_BAUDRATE 	921600
#define LODI2_LPUART_CLKSRC 	kCLOCK_Osc0ErClk
#define LODI2_LPUART_CLK_FREQ 	CLOCK_GetFreq(kCLOCK_Osc0ErClk)

#define LODI2_SPI_CLOCK 		25000000U	//<< 25Mhz SPI clock
#define LODI2_SPI_MASTER_PCS_FOR_INIT 		kDSPI_Pcs0
#define LODI2_SPI_PORT						SPI0
#define LODI2_SPI_MASTER_CLK_FREQ 			CLOCK_GetFreq(DSPI0_CLK_SRC)
#define LODI2_SPI_MASTER_PCS_FOR_TRANSFER 	kDSPI_MasterPcs0

#define RX_RING_BUFFER_SIZE 		64*1024
uint8_t g_rxRingBuffer[RX_RING_BUFFER_SIZE] = {0}; /* RX ring buffer. */
lpuart_handle_t g_lpuartHandle;

static SemaphoreHandle_t pMutex=NULL;
static SemaphoreHandle_t pSPIMutex=NULL;
SemaphoreHandle_t LogLock::pLogMutex=NULL;
enum { WAIT, RESET, TIMEOUT };
int g_rxOnGoing = WAIT;




void LD2OS_delay(uint32_t mSec)
{
	vTaskDelay(mSec/portTICK_PERIOD_MS);
}


uint32_t LD2OS_getTime(void)
{
    return (uint32_t) xTaskGetTickCount();
}

static bool bIsSerialOpened = false;

int fsl_overrun_cnt = 0;
int ring_overrun_cnt = 0;
void fsl_lpuart_callback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData)
{
	if (status == kStatus_LPUART_RxRingBufferOverrun)
	{
		ring_overrun_cnt++;
	}
	else if (status == kStatus_LPUART_RxHardwareOverrun)
	{
		fsl_overrun_cnt++;
	}
}

bool LD2OS_open(LoDi2SerialConnection connType, int port, int baudrate, const char *tty)
{
	if (bIsSerialOpened)
	{
		if (connType == LODI2_SERIAL_UART)
		{
#if defined(FSL_FEATURE_LPUART_HAS_FIFO) && FSL_FEATURE_LPUART_HAS_FIFO
			/* Wait tx FIFO send out*/
			while (0 != ((LODI2_LPUART->WATER & LPUART_WATER_TXCOUNT_MASK) >> LPUART_WATER_TXWATER_SHIFT))
			{
			}
#endif
			/* Wait last char shift out */
			while (0 == (LODI2_LPUART->STAT & LPUART_STAT_TC_MASK))
			{
			}

			LPUART_SetBaudRate(LODI2_LPUART, baudrate, LODI2_LPUART_CLK_FREQ);
		}
		return true;
	}

	bIsSerialOpened = true;

	g_connType = connType;

	gpio_pin_config_t gpio_output_config = {
		kGPIO_DigitalOutput, 0,
	};
	gpio_pin_config_t gpio_input_config = {
		kGPIO_DigitalInput, 0,
	};

	GPIO_PinInit(BOARD_LODI2_GPIO_PORT, LODI2_GPIO_MCU_REQ, &gpio_output_config);
	GPIO_PinInit(BOARD_LODI2_GPIO_PORT, LODI2_GPIO_MCU_RESP, &gpio_input_config);
	GPIO_PinInit(BOARD_LODI2_GPIO_PORT, LODI2_GPIO_HOST_REQ, &gpio_input_config);
	GPIO_PinInit(BOARD_LODI2_GPIO_PORT, LODI2_GPIO_NSTDBY, &gpio_output_config);

    if (connType == LODI2_SERIAL_UART)
    {
		lpuart_config_t uart_config;

		LPUART_GetDefaultConfig(&uart_config);
		uart_config.baudRate_Bps = baudrate;
		uart_config.enableTx = true;
		uart_config.enableRx = true;
		uart_config.enableRxRTS = true;
		uart_config.enableTxCTS = true;

		LPUART_Init(LODI2_LPUART, &uart_config, LODI2_LPUART_CLK_FREQ);
		LPUART_TransferCreateHandle(LODI2_LPUART, &g_lpuartHandle, fsl_lpuart_callback, NULL);
		LPUART_TransferStartRingBuffer(LODI2_LPUART, &g_lpuartHandle, g_rxRingBuffer, RX_RING_BUFFER_SIZE);
		return true;
    }
    else if (connType == LODI2_SERIAL_SPI)
    {
    	// initialize SPI Master
    	SPI_init();
    	HostReq_init(LODI2_GPIO_HOST_REQ);

    	pSPIMutex = xSemaphoreCreateRecursiveMutex();
    	return true;
    }
    else
    {
    	LD2_LOG("Error: cannot support the connection type = %d\n", connType);
    	return false;
    }
}

void LD2OS_emptySerialBuffer()
{
	lpuart_transfer_t receiveXfer;
	size_t rxBytes = 0;
	//status_t status;
	uint8_t buf[1024];

	if (g_connType == LODI2_SERIAL_SPI)
	{
		return;
	}

	receiveXfer.data = buf;
	receiveXfer.dataSize = sizeof(buf);

	LPUART_TransferReceiveFromRingbuffer(LODI2_LPUART, &g_lpuartHandle, &receiveXfer, &rxBytes);
}

void LD2OS_close(void)
{
	//We don't close
}

void LD2OS_enableCtsForMcuRdy(bool enable)
{
	if (g_connType != LODI2_SERIAL_UART)
	{
		LD2_LOG("Error: Not support CTS for MCU_RDY (connType = %d)\n", g_connType);
		LD2_ASSERT(0);
	}
	g_bEnabledCtsForMcuRdy = enable;
}

bool LD2OS_waitForHostReq(uint32_t timeout)
{
	return HostReq_wait(timeout);
}

uint8_t LD2OS_getMcuRdy(void)
{
	if(g_bEnabledCtsForMcuRdy == false)
	{
		return LD2OS_getGpio(LODI2_GPIO_MCU_RESP);
	}
	uint8_t pinState = 0x00;
	pinState = GPIO_PinRead(BOARD_LODI2_GPIO_PORT, LODI2_GPIO_CTS_MCU_RESP);
	return ~(pinState & 0x01);
}

void LD2OS_setGpio(LoDi2Gpio pin, uint8_t value)
{
    if (value)
    {
    	GPIO_PortSet(BOARD_LODI2_GPIO_PORT, 1u << pin);
    }
    else
    {
    	GPIO_PortClear(BOARD_LODI2_GPIO_PORT, 1u << pin);
    }
}


uint8_t LD2OS_getGpio(LoDi2Gpio pin)
{
    return GPIO_PinRead(BOARD_LODI2_GPIO_PORT, pin);
}

bool LD2OS_writeToSerial(const unsigned char *pcBuff, uint32_t ulLen)
{
    if (g_connType == LODI2_SERIAL_UART)
    {
    	LPUART_WriteBlocking(LODI2_LPUART, pcBuff, ulLen);
    }
#ifdef SUPPORT_SPI
    else if (g_connType == LODI2_SERIAL_SPI)
    {
    	uint8_t *ssi_tx_buf = NULL;
    	uint32_t ssi_tx_len = 0;

    	/* pack tx data for ssi */
    	LD2SSI_packTxBuff((unsigned char *)pcBuff, ulLen, &ssi_tx_buf, &ssi_tx_len);

    	/* send the data to 4775 */
    	xSemaphoreTakeRecursive(pSPIMutex, portMAX_DELAY);
    	SPI_sync(ssi_tx_buf, NULL, ssi_tx_len);
    	xSemaphoreGiveRecursive(pSPIMutex);

        return true;
    }
#endif
    else
    {
    	LD2_LOG("Error: not supported connType = %d\n", g_connType);
    	return false;
    }
    return true;
}

void LD2OS_cancelReadFromSerial(void)
{
    g_rxOnGoing = RESET;
}


bool LD2OS_readFromSerial(unsigned char *pcBuff, uint32_t *ulLen, uint32_t ulReqLen, uint32_t timeout)
{
    bool ret = true;
    uint32_t start = LD2OS_getTime();
    if (g_rxOnGoing != RESET) g_rxOnGoing = WAIT;
    if (g_connType == LODI2_SERIAL_UART)
    {
		lpuart_transfer_t receiveXfer;
		size_t rxBytes = 0;
		status_t status;

		receiveXfer.data = pcBuff;
		receiveXfer.dataSize = ulReqLen;

		do
		{
			*ulLen = 0;
			status = LPUART_TransferReceiveFromRingbuffer(LODI2_LPUART, &g_lpuartHandle, &receiveXfer, &rxBytes);

			if (status != kStatus_Success)
			{
				ret = false;
				break;
			}

			if (rxBytes > 0)
			{
				*ulLen = rxBytes;
				break;
			}

			if (rxBytes == 0 && LD2OS_getTime() - start < timeout)
			{
				LD2OS_delay(1);
			}
		} while(LD2OS_getTime() - start < timeout && g_rxOnGoing != RESET);
    }
#ifdef SUPPORT_SPI
    else if (g_connType == LODI2_SERIAL_SPI)
    {
    	uint8_t *tx_ssi_buff = NULL;
    	uint8_t *rx_ssi_buff = NULL;
    	uint32_t ctrl_len = 0;
    	uint32_t payload_len = 0;

    	if (LD2OS_getGpio(LODI2_GPIO_HOST_REQ) == 0)
    	{
    		bool ret = LD2OS_waitForHostReq(timeout);
    		return false;
    	}

    	do {
			/* get payload length from 4775 first
			 *    - tx_ssi_buff & rx_ssi_buff are buffer pointer in ssi driver
			 */
			ctrl_len = LD2SSI_packRxCtrl(&tx_ssi_buff, &rx_ssi_buff);
			SPI_sync(tx_ssi_buff, rx_ssi_buff, ctrl_len);

			/* get payload length from rx data */
			payload_len = LD2SSI_getRxLength(rx_ssi_buff);
			if (payload_len == 0 && LD2OS_getTime() - start < timeout)
			{
				LD2OS_delay(1);
			}
			else
			{
				uint32_t readLen = MIN(ulReqLen, payload_len);

				xSemaphoreTakeRecursive(pSPIMutex, portMAX_DELAY);
				SPI_sync(tx_ssi_buff, rx_ssi_buff, ctrl_len + readLen);
				xSemaphoreGiveRecursive(pSPIMutex);

				*ulLen = LD2SSI_unpackRxBuff(pcBuff, readLen, rx_ssi_buff);
				break;
			}
    	} while (LD2OS_getTime() - start < timeout && g_rxOnGoing != RESET);
    }
#endif
    else
    {
    	LD2_LOG("Error: Not support the connection type = %d\n", g_connType);
    	ret = false;
    }

	g_rxOnGoing = TIMEOUT;
    return ret;
}


void LD2OS_initMutex(void)
{
	if (pMutex==NULL)
	{
		pMutex = xSemaphoreCreateRecursiveMutex();
	}
}


void LD2OS_deinitMutex(void)
{
	if (pMutex!=NULL)
	{
		xSemaphoreGiveRecursive(pMutex);
		vSemaphoreDelete(pMutex);
		pMutex = NULL;
	}
}


void LD2OS_lockMutex(void)
{
	xSemaphoreTakeRecursive(pMutex, portMAX_DELAY);
}


void LD2OS_unlockMutex(void)
{
	xSemaphoreGiveRecursive(pMutex);
}


void* LD2OS_openFile(const char *path, uint16_t flags)
{
    
    LogLock fileLock;
	LD2_ASSERT(path);
    
	FIL *fd = (FIL*)pvPortMalloc(sizeof(FIL));
	LD2_ASSERT(fd);

	BYTE mode = 0;
	if (flags & LD2OS_O_RDONLY)	{ mode |= FA_READ;}
	if (flags & LD2OS_O_WRONLY) { mode |= FA_WRITE;}
	if (flags & LD2OS_O_RDWR) 	{ mode |= FA_READ | FA_WRITE;}
	if (flags & LD2OS_O_CREAT)	{ mode |= FA_CREATE_ALWAYS;}

	FRESULT error = f_open(fd, path, mode);
	if (error !=FR_OK && error != FR_EXIST)
	{
		PRINTF("%s (%s,%u) error %d\n", __FUNCTION__, path, flags, error);
		vPortFree(fd);
		fd = NULL;
	}

	return fd;
}


void LD2OS_closeFile(void* fd)
{
    
    LogLock fileLock;
	if (fd == NULL)
		return;

	FRESULT error = f_close((FIL*)fd);
	LD2_ASSERT(error==FR_OK);
	vPortFree(fd);
}

int32_t LD2OS_readFile(const void* fd, void *buf, int32_t size)
{
    
    LogLock fileLock;
	LD2_ASSERT(fd);
	LD2_ASSERT(size>0);

	UINT bytesRead=0;
    
	FRESULT error = f_read((FIL*)fd, buf, size, &bytesRead);
	LD2_ASSERT(error==FR_OK);
	return bytesRead;
}

int32_t LD2OS_writeFile(const void* fd, void *buf, int32_t size)
{
    
    LogLock fileLock;
    LD2_ASSERT(fd);
    LD2_ASSERT(buf);
    LD2_ASSERT(size>0);

	UINT bytesWritten=0;
    FRESULT error = f_write((FIL*)fd, buf, size, &bytesWritten);
    if (error != FR_OK) {
    	PRINTF("f_write error %d\r\n", error);
    	LD2_ASSERT(0);
    }
    return bytesWritten;
}

int32_t LD2OS_seekFile(const void* fd, int32_t offset)
{
    LD2_ASSERT(fd);
    LD2_ASSERT(offset>=0);

    LogLock fileLock;
    FRESULT error = f_lseek((FIL*)fd, (FSIZE_t)offset);
    if (error != FR_OK)
	{
        PRINTF("f_lseek error %d\r\n", error);
        LD2_ASSERT(0);
        return -1;
    }
    return offset;
}

int32_t LD2OS_statFile(const char* path, LD2OSFileInfo* info)
{
    LD2_ASSERT(path);
    LD2_ASSERT(info);
    
    LogLock fileLock;
    FILINFO fno = {};
    FRESULT error = f_stat(path, &fno);
    if (error != FR_OK)
	{
        PRINTF("Error: f_stat error %d\r\n", error);
        LD2_ASSERT(0);
        return -1;
    }

    info->fsize = (int64_t)fno.fsize;
    return 0;
}

void* LD2OS_openLog()
{
    LogLock lock;
	if (g_fdLog)
	{
		return g_fdLog;
	}

	char fname[128];
    rtc_datetime_t date;
    RTC_GetDatetime(RTC, &date);
	sprintf(fname, "/log/gl-%04d-%02d-%02d-%02d-%02d-%02d.txt", date.year, date.month, date.day, date.hour, date.minute, date.second);
	g_fdLog = LD2OS_openFile(fname, LD2OS_O_RDWR|LD2OS_O_CREAT);
	LD2_ASSERT(g_fdLog);
	return g_fdLog;
}

void LD2OS_closeLog()
{
    LogLock lock;

	LD2OS_closeFile((void*)g_fdLog);
	g_fdLog = NULL;
}


void* LD2OS_openFailSafe()
{
	char fname[128];
    rtc_datetime_t date;
    RTC_GetDatetime(RTC, &date);
	sprintf(fname, "%sfailsafe-%04d-%02d-%02d-%02d-%02d-%02d.bin", LD2_FAILSAFE_DIR, date.year, date.month, date.day, date.hour, date.minute, date.second);
	void *fd = LD2OS_openFile(fname, LD2OS_O_RDWR|LD2OS_O_CREAT);
	LD2_ASSERT(fd);
	return fd;
}

void LD2OS_assert(const char *format, ...)
{
    char buf[256];
    char *pbuf = buf;
    rtc_datetime_t date;
    RTC_GetDatetime(RTC, &date);
    pbuf += sprintf(pbuf, "[%04d-%02d-%02d %02d:%02d:%02d][%10u] ASSERT : ",
    		date.year, date.month, date.day, date.hour, date.minute, date.second, (unsigned int)LD2OS_getTime());
    va_list arg_ptr;
    va_start(arg_ptr, format);
    pbuf += vsprintf(pbuf, format, arg_ptr);
    va_end(arg_ptr);
    PRINTF("%s\n", buf);

    //LD2OS_writeFile creates recursive call here. Therefore, directly call f_write
    {
        LogLock lock;
        UINT bytesWritten = 0;
        FRESULT error = f_write((FIL*)g_fdLog, buf, strlen(buf), &bytesWritten);
        if (error != FR_OK)
        {
            PRINTF("Error: f_write error %d\r\n", error);
        }
    }

    LD2OS_closeLog();
    while(true);
    //TODO: rename suffix into *.a.log
}

void LD2OS_log(bool bFileOnly, const char *format, ...)
{
    LogLock lock;

	char buf[4096];
	char *pbuf=buf;
    rtc_datetime_t date;
    RTC_GetDatetime(RTC, &date);
    pbuf += sprintf(pbuf, "[%04d-%02d-%02d %02d:%02d:%02d][%10u] ",
    		date.year, date.month, date.day, date.hour, date.minute, date.second, (unsigned int)LD2OS_getTime());
	va_list arg_ptr;
	va_start(arg_ptr, format);
	pbuf += vsprintf(pbuf, format, arg_ptr);
	va_end(arg_ptr);
	if (!bFileOnly)
	{
		PRINTF("%s",buf);
	}
	LD2OS_writeFile(g_fdLog, buf, strlen(buf));
}

void LD2OS_dump(const uint8_t *from, const uint32_t size)
{
	for (uint32_t i=0; i<size; i++)
	{
		PRINTF("%02X ", from[i]);
	}
	PRINTF("\n");
}


