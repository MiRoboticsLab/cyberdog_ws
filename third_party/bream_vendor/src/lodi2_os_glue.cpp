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
/** @file lodi2_os_glue.cpp
*============================================================================*/

#include "lodi2_os_glue.h"

#include <windows.h>
#include <stdio.h>
#include <time.h>
#include <fcntl.h>  
#include <sys/types.h>  
#include <sys/stat.h> 
#include <io.h>
#include <errno.h>
#include <string>
#include <mutex>
#include "uart_hal_winxp.h"
#include "tcp_hal_winxp.h"
#include "ftdi_spi_hal_winxp.h"
#include "ftdi_enum.h"
#include "ftdi_gpio.h"

static void *g_fdLog = NULL;
static bool g_bEnabledCtsForMcuRdy = false;
static LoDi2SerialConnection g_connType = LODI2_SERIAL_UART;
static GpioType g_gpioType = GpioType::FTDI;
static bool g_bResetFdti = false;

static std::recursive_mutex fileLock;
static std::recursive_mutex logLock;

static IHalSerial* g_serial = nullptr;
static IHalGpio* g_gpio = nullptr;

void LD2OS_delay(uint32_t mSec)
{
	Sleep(mSec);
}

uint32_t LD2OS_getTime(void)
{
	static uint64_t timeOrigin = 0;

	FILETIME time;
	GetSystemTimeAsFileTime(&time);
	uint64_t time_ms = (((LONGLONG)(time.dwHighDateTime) << 32LL) + time.dwLowDateTime) / 10000;
	if(!timeOrigin) 
	{
		timeOrigin = time_ms;
	}

	return (uint32_t)(time_ms - timeOrigin);
}

GpioType LD2OS_GetGpioType()
{
    return g_gpioType;
}

void LD2OS_SetGpioType(GpioType type)
{
    g_gpioType = type;
}

void LD2OS_SetFtdiReset(bool reset)
{
    g_bResetFdti = reset;
}

bool LD2OS_open(LoDi2SerialConnection connType, int port, int baudrate, const char *tty)
{
    if (g_serial && connType == LODI2_SERIAL_SPI)
    {
        // Skip re-open in SPI connect type
        return true;
    }

    LD2_LOG("%s() \n", __FUNCTION__);
    if (g_serial)
    {
        delete g_serial;
        g_serial = nullptr;
    }
    int retryCount = 0;
retry:
    switch (g_gpioType)
    {
    case GpioType::FTDI:
        if (g_bResetFdti)
        {
            FtdiEnum::GetInst().Reset();
            g_bResetFdti = false;
        }
        port = FtdiEnum::GetInst().SelectDevice(port);
        if (port < 0)
        {
            LD2_ASSERT(false);
            return false;
        }
        g_gpio = FtdiGpio::GetInst();
        break;
    case GpioType::NONE:
        {
            static IHalGpio s_emptyGpio;
            g_gpio = &s_emptyGpio;
        }
        break;
    }

    switch (connType)
    {
    case LODI2_SERIAL_UART:
        g_serial = new UartHal(port, baudrate, g_gpioType);
        break;
    case LODI2_SERIAL_TCP:
        g_serial = new TcpHal(port);
        break;
    case LODI2_SERIAL_SPI:
        g_serial = new FtdiSpiHal(port, 25000000, g_gpioType);
        break;
    default:
        LD2_LOG("%s() Not supported connection type:%d\n",
            __FUNCTION__, connType);
        LD2_ASSERT(false);
        return false;
    }

    g_connType = connType;
    if (!g_serial->Open())
    {
        delete g_serial;
        if (retryCount++ < 3)
        {
            g_bResetFdti = true;
            LD2_LOG("%s() Retry opening UART %d/3\n", __FUNCTION__, retryCount);
            goto retry;
        }
        LD2_ASSERT(false);
        return false;
    }
    return true;
}

void LD2OS_emptySerialBuffer()
{
    LD2_ASSERT(g_serial);
    g_serial->EmptyBuffer();
}

void LD2OS_close(void)
{
    if (g_serial)
    {
        delete g_serial;
        g_serial = nullptr;
    }
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
	LD2_LOG("Not supported waiting HOST_REQ\n");
	LD2_ASSERT(0);
    return false;
}

uint8_t LD2OS_getMcuRdy(void)
{
    LD2_ASSERT(g_gpio);
	if(g_bEnabledCtsForMcuRdy == false)
	{
        return g_gpio->Get(LODI2_GPIO_MCU_RESP);
	}
    else
    {
        return g_gpio->Get(LODI2_GPIO_CTS_MCU_RESP);
    }
}

void LD2OS_setGpio(LoDi2Gpio pin, uint8_t value)
{
    LD2_ASSERT(g_gpio);
    g_gpio->Set(pin, value);
}

uint8_t LD2OS_getGpio(LoDi2Gpio pin)
{
    LD2_ASSERT(g_gpio);
    return g_gpio->Get(pin);
}

bool LD2OS_writeToSerial(const unsigned char *pcBuff, uint32_t ulLen)
{
	if ((g_connType != LODI2_SERIAL_UART) && (g_connType != LODI2_SERIAL_SPI))
	{
		LD2_LOG("Error: not supported connType = %d\n", g_connType);
		return false;
	}
	//LD2_LOG("TX : ");
	//LD2_DUMP(pcBuff, ulLen);

    return g_serial->Send(pcBuff, ulLen);
}

void LD2OS_cancelReadFromSerial(void)
{
}

bool LD2OS_readFromSerial(unsigned char *pcBuff, uint32_t *ulLen, uint32_t ulReqLen, uint32_t timeout)
{
	LD2_ASSERT(pcBuff);
	LD2_ASSERT(ulLen);

    if ((g_connType != LODI2_SERIAL_UART) && (g_connType != LODI2_SERIAL_SPI))
	{
		LD2_LOG("Error: not supported connType = %d\n", g_connType);
		return false;
	}
    long receved = g_serial->Recv(pcBuff, ulReqLen);
    if (receved < 0)
    {
        return false;
    }
    *ulLen = receved;
    return true;
}

void* LD2OS_openFile(const char *path, uint16_t flags)
{
    std::lock_guard<std::recursive_mutex> lock(fileLock);
	LD2_ASSERT(path);
	const char* modes[] = {"rb", "wb"};
	const char* mode = (flags == LD2OS_O_RDONLY) ? modes[0] : modes[1];
	FILE* fd = fopen(path, mode);
	LD2_ASSERT(fd);
	return (void*)fd;
}

void LD2OS_closeFile(void* fd)
{
    std::lock_guard<std::recursive_mutex> lock(fileLock);
	LD2_ASSERT(fd);
	fclose((FILE*)fd);
}

int32_t LD2OS_readFile(const void* fd, void *buf, int32_t size)
{
    std::lock_guard<std::recursive_mutex> lock(fileLock);
	LD2_ASSERT(fd);
	LD2_ASSERT(size>0);

	size_t bytesRead = fread(buf, 1, size, (FILE*)fd);
	return (int32_t)bytesRead;
}

int32_t LD2OS_writeFile(const void* fd, void *buf, int32_t size)
{
    std::lock_guard<std::recursive_mutex> lock(fileLock);
    LD2_ASSERT(fd);
    LD2_ASSERT(buf);
    LD2_ASSERT(size>0);

	size_t bytesWritten = fwrite(buf, 1, size, (FILE*)fd);
	return (int32_t)bytesWritten;
}

int32_t LD2OS_seekFile(const void* fd, int32_t offset)
{
    std::lock_guard<std::recursive_mutex> lock(fileLock);
    LD2_ASSERT(fd);
    LD2_ASSERT(offset>=0);

    int32_t ret = fseek((FILE*)fd, offset, SEEK_SET);
	LD2_ASSERT(ret>0);
	return ret;
}

int32_t LD2OS_statFile(const char* path, LD2OSFileInfo* info)
{
    std::lock_guard<std::recursive_mutex> lock(fileLock);
	struct _stat sb;
    int ret = _stat(path, &sb);
    info->fsize = (int64_t)sb.st_size;
	return ret;
}

void* LD2OS_openLog()
{
    printf("sensor_manager 01");

    std::lock_guard<std::recursive_mutex> lock(logLock);
	if (g_fdLog)
	{
		return g_fdLog;
	}

	char fname[128];
    time_t rawtime;
    struct tm * ptm;
  
    time(&rawtime);
    ptm = localtime(&rawtime);
	sprintf(fname, "%sgl-%04d-%02d-%02d-%02d-%02d-%02d.log", LD2_LOG_DIR,
        ptm->tm_year + 1900, ptm->tm_mon + 1, ptm->tm_mday,
        ptm->tm_hour, ptm->tm_min, ptm->tm_sec);
    printf("sensor_manager 02");
    printf("sensor_manager 02 %s",fname);

	g_fdLog = LD2OS_openFile(fname, LD2OS_O_RDWR|LD2OS_O_CREAT);
	LD2_ASSERT(g_fdLog);
    printf("sensor_manager 03");

	return g_fdLog;
}

void LD2OS_closeLog()
{
    std::lock_guard<std::recursive_mutex> lock(logLock);
    if (g_fdLog)
    {
        LD2OS_closeFile((void*)g_fdLog);
        g_fdLog = 0;
    }
}

void* LD2OS_openFailSafe()
{
	char fname[128];
    time_t rawtime;
    struct tm * ptm;
  
    time(&rawtime);
    ptm = localtime(&rawtime);
	sprintf(fname, "%sfailsafe-%04d-%02d-%02d-%02d-%02d-%02d.bin", LD2_FAILSAFE_DIR,
        ptm->tm_year + 1900, ptm->tm_mon + 1, ptm->tm_mday,
        ptm->tm_hour, ptm->tm_min, ptm->tm_sec);
	void *fd = LD2OS_openFile(fname, LD2OS_O_RDWR|LD2OS_O_CREAT);
	LD2_ASSERT(fd);
	return fd;
}

typedef void(*messageCb)(const char *msg);
static messageCb OnAssert = nullptr;
void LD2OS_setAssertCb(messageCb callback)
{
    OnAssert = callback;
}

void LD2OS_assert(const char *format, ...)
{
    char buf[256];
    char *pbuf = buf;
    pbuf += sprintf(pbuf, "[%10u] ASSERT : ", LD2OS_getTime());
    va_list arg_ptr;
    va_start(arg_ptr, format);
    pbuf += vsprintf(pbuf, format, arg_ptr);
    va_end(arg_ptr);
    printf("%s\n", buf);
    fflush(stdout);

    if (g_fdLog)
    {
        std::lock_guard<std::recursive_mutex> lock(logLock);
        size_t ret = fwrite(buf, 1, strlen(buf), (FILE*)g_fdLog);
        if (ret == 0)
        {
            printf("write error %d\r\n", errno);
        }
        LD2OS_closeLog();
    }
    if (OnAssert)
        OnAssert(buf);
    else
        exit(-1);
    //TODO: rename suffix into *.a.log
}

static messageCb OnHalLog = nullptr;
void LD2OS_setHalLogCb(messageCb callback)
{
    OnHalLog = callback;
}
void LD2OS_log(bool bFileOnly, const char *format, ...)
{
    char buf[4096];
    char *pbuf = buf;
    if (!OnHalLog)
        pbuf += sprintf(pbuf, "[%10u] ", LD2OS_getTime());
    va_list arg_ptr;
    va_start(arg_ptr, format);
    pbuf += vsprintf(pbuf, format, arg_ptr);
    va_end(arg_ptr);
    if (OnHalLog)
    {
        OnHalLog(buf);
        return;
    }
    if (!bFileOnly)
    {
        printf("%s", buf);
        fflush(stdout);
    }

    std::lock_guard<std::recursive_mutex> lock(logLock);
    if (!g_fdLog) return;
    size_t ret = fwrite(buf, 1, strlen(buf), (FILE*)g_fdLog);
    if (ret == 0)
    {
    	printf("Error: write error %d\r\n", errno);
    }
}

void LD2OS_dump(const uint8_t *from, const uint32_t size)
{
	for (uint32_t i=0; i<size; i++) 
	{
		printf("%02X ", from[i]);
	}
	printf("\n");
	fflush(stdout); 
}
