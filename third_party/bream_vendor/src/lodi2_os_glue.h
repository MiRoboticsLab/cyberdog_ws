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
/** @file lodi2_os_glue.h
*============================================================================*/

#ifndef LODI2_OS_GLUE_LAYER_H
#define LODI2_OS_GLUE_LAYER_H

#include <stdint.h>
#include <stdlib.h>
/**  @addtogroup control
 *   @{
 *   @enum LoDi2SerialConnection Serial connection between user MCU and LoDi2 board
 *  4775 chip automatically detects which serial interface is being used.
 *  In case UART is detected, no special handling is required.
 *  However if SPI or I2C is detected, 4775 interrupts MCU to request read.
 *  Also, 4775 engages framing format called SSI so that MCU know how much to read.
 */
typedef enum {
	LODI2_SERIAL_UART,      ///< UART
	LODI2_SERIAL_SPI,       ///< SPI
	LODI2_SERIAL_I2C,       ///< I2C
    LODI2_SERIAL_TCP,
}LoDi2SerialConnection;

/*! @brief Structure is used to hold the date and time */
typedef struct _Lodi2DateTime
{
    uint16_t year;  /*!< Range from 1970 to 2099.*/
    uint8_t month;  /*!< Range from 1 to 12.*/
    uint8_t day;    /*!< Range from 1 to 31 (depending on month).*/
    uint8_t hour;   /*!< Range from 0 to 23.*/
    uint8_t minute; /*!< Range from 0 to 59.*/
    uint8_t second; /*!< Range from 0 to 59.*/
} Lodi2DateTime;

#define LD2_ASSERT(x)		if(!(x)) {LD2OS_assert(" %s %u", __FILE__, __LINE__);}
#define LD2_LOG(...)		LD2OS_log(false, __VA_ARGS__)
#define LD2_FLOG(...)		LD2OS_log(true, __VA_ARGS__)
#define LD2_DUMP(x,size)	LD2OS_dump((x),(size))

#if defined(_WIN32)
#define LD2_LOG_DIR  ".\\\\"
#define LD2_FAILSAFE_DIR ".\\\\"
#elif defined(__unix__) || defined (__APPLE__)
#define LD2_LOG_DIR  "./"
#define LD2_FAILSAFE_DIR "./failsafe/"
#elif defined(SDK_OS_FREE_RTOS)
#define LD2_LOG_DIR "/log/"
#define LD2_FAILSAFE_DIR "/failsafe/"
#elif defined(AM_PART_APOLLO3)
#define LD2_NO_FILESYSTEM
#endif

#if defined(_MSC_VER)
#define PACK( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#else
#define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#endif

/** @defgroup gpioConfig GPIO configuration
    Four GPIO pins are used for LoDi2 board and their pin number should be defined in lodi2_os_glue.h
    
    @verbatim
    LODI2_GPIO_MCU_REQ  : MCU sets high this pin to wake up LoDi2 board. MCU sets low this pin to let LoDi2 sleep.
    LODI2_GPIO_MCU_RESP : LoDi2 board sets high this pin when it's awake. It sets this pin low before entering sleep.
    LODI2_GPIO_HOST_REQ : LoDi2 board sets high this pin when it has data bytes to send to MCU. It sens this pin low when there's no more data bytes for MCU.
    LODI2_GPIO_NSTDBY   : MCU sets high this pin to power on LoDi2 board
    @endverbatim

    In lodi2_os_glue.h, above pins should be defined as following example according to your HW configuration.
    
    @verbatim
    typedef enum{
        LODI2_GPIO_MCU_REQ = 0,
        LODI2_GPIO_MCU_RESP = 1,
        LODI2_GPIO_HOST_REQ = 2,
        LODI2_GPIO_NSTDBY = 7
    }LoDi2Gpio;
    @endverbatim
 */
#if defined(_WIN32)

/** \brief 4775EVK GPIO definition
 *
 * 4775EVK has FTDI4232 quad port USB serial interface where CBUS pins are connected with 4775's gpio pins.
 * 
 *   CBUS pin0 : MCU_REQ  (output from FTDI4232, input to 4775)
 *   CBUS pin1 : MCU_RESP (input to FTDI4232, output from 4775)
 *   CBUS pin2 : HOST_REQ (input to FTDI4232, output from 4775)
 *   CBUS pin5 : DBG2     (output from FTDI4232, input to 4775)
 *   CBUS pin6 : DBG1     (output from FTDI4232, input to 4775)
 *   CBUS pin7 : NSTDBY   (output from FTDI4232, input to 4775)
 *
 */
typedef enum{
	LODI2_GPIO_MCU_REQ = 0,
	LODI2_GPIO_MCU_RESP = 1,
	LODI2_GPIO_HOST_REQ = 2,
	LODI2_GPIO_CTS_MCU_RESP = 3,
	LODI2_GPIO_TEST = 4,
    LODI2_GPIO_DBG2 = 5,
    LODI2_GPIO_DBG1 = 6,
	LODI2_GPIO_NSTDBY = 7
}LoDi2Gpio;

#elif defined(__ICCARM__)

/* @brief 4775 GPIO definition
 * FOLLOWING GPIO NUMBER IS NOT REAL. PLEASE FIX IT ACCORDING TO YOUR HW GPIO CONFIGURATION
 */
typedef enum{
#if defined(AM_PART_APOLLO3)
    LODI2_GPIO_TEST = 0, // NOTE: TEST pin is not used for Apollo board but declared here to pass build.
    LODI2_GPIO_HOST_REQ = 17,
    LODI2_GPIO_NSTDBY = 14,
    LODI2_GPIO_MCU_REQ = 15,
    LODI2_GPIO_MCU_RESP = 30
#else
	LODI2_GPIO_MCU_REQ = 5,
	LODI2_GPIO_MCU_RESP = 6,
	LODI2_GPIO_HOST_REQ = 1,
	LODI2_GPIO_NSTDBY = 2
#endif
}LoDi2Gpio;

#elif defined(__unix__) || defined(__APPLE__)

/* @brief 4775 GPIO definition
 * FOLLOWING GPIO NUMBER IS NOT REAL. PLEASE FIX IT ACCORDING TO YOUR HW GPIO CONFIGURATION
 */
typedef enum{
	LODI2_GPIO_MCU_REQ = 0,
	LODI2_GPIO_MCU_RESP = 1,
	LODI2_GPIO_HOST_REQ = 2,
	LODI2_GPIO_NSTDBY = 7,
    LODI2_GPIO_TEST = 12, // NOTE: TEST pin is not used for NXP board but declared here to pass build.
}LoDi2Gpio;

#elif defined(SDK_OS_FREE_RTOS)

/* @brief 4775 GPIO definition
 * FOLLOWING GPIO NUMBER IS NOT REAL. PLEASE FIX IT ACCORDING TO YOUR HW GPIO CONFIGURATION
 */
typedef enum{
	LODI2_GPIO_CTS_MCU_RESP = 11,
	LODI2_GPIO_MCU_REQ = 12,
	LODI2_GPIO_TEST = 12, // NOTE: TEST pin is not used for NXP board but declared here to pass build.
	LODI2_GPIO_MCU_RESP = 13,
	LODI2_GPIO_HOST_REQ = 14,
	LODI2_GPIO_NSTDBY = 15
}LoDi2Gpio;

#else

/* @brief 4775 GPIO definition
 * FOLLOWING GPIO NUMBER IS NOT REAL. PLEASE FIX IT ACCORDING TO YOUR HW GPIO CONFIGURATION
 */
typedef enum{
    LODI2_GPIO_MCU_REQ = 0,
    LODI2_GPIO_MCU_RESP = 1,
    LODI2_GPIO_HOST_REQ = 2,
    LODI2_GPIO_NSTDBY = 7
}LoDi2Gpio;

#endif

#define LODI2_GPIO_OUT_MASK		((1<<LODI2_GPIO_NSTDBY) | (1<<LODI2_GPIO_MCU_REQ))
#define LODI2_GPIO_IN_MASK		((1<<LODI2_GPIO_HOST_REQ) | (1<<LODI2_GPIO_MCU_RESP))


/** @} */ // end of gpioConfig







/** @defgroup OsGlue  OS Abstraction Layer Functions
 *  @brief LoDi2 calls common system functions like open, read, delay, etc. Following functions abstracts OS dependencies for LoDi2.
 *  @{
 */

/** @brief Delays current task
 *  @param[in] mSec time in msec 
 */
void LD2OS_delay(uint32_t mSec);


/** @brief Get current monolithic system time.
 *  @return time in msec
 */
uint32_t LD2OS_getTime(void);

/*!
 * @brief Gets the RTC time and stores it in the given time structure.
 *
 * @param datetime Pointer to the structure where the date and time details are stored.
 */
void LD2OS_GetDatetime(Lodi2DateTime *datetime);

/*!
 * @brief Sets the RTC date and time according to the given time structure.
 *
 * The RTC counter must be stopped prior to calling this function because writes to the RTC
 * seconds register fail if the RTC counter is running.
 *
 * @param datetime Pointer to the structure where the date and time details are stored.
 *
 * @return true: Success in setting the time and starting the RTC
 *         false: Error because the datetime format is incorrect
 */
bool LD2OS_SetDatetime(Lodi2DateTime *datetime);

/** @brief Open serial connection with LoDi2 board. Depending on HW configuration, the connection can be UART or SPI.
 *  @param[in] connType         Serial connection with LoDi2 board
 *  @return true for success. false for error.
 */
bool LD2OS_open(LoDi2SerialConnection connType, int port, int baudrate=115200, const char *tty=nullptr);


/** @brief Close erial connection with LoDi2 board.
 */
void LD2OS_close(void);

/** @brief enable checking HOST_UART_nCTS pin instead of MCU_RDY pin
 *  @param[in] enable true or false 
 */
void LD2OS_enableCtsForMcuRdy(bool enable);

/** @brief wait HOST_REQ pin to high level
 *  @param[in] timeout Timeout to give up waiting host_req 
 *  @return true for success. false for error.
 */
bool LD2OS_waitForHostReq(uint32_t timeout);

/** @brief Get MCU Ready status
 *  @return level 0 for not ready(MCU sleep), 1 for ready (MCU wake-up)
 */
uint8_t LD2OS_getMcuRdy(void);

/** @brief Set high/low specific gpio pin which is used for LoDi2 board. For more information of gpio pin, please refer HW introduction
 *  @param[in] pin Gpio pin
 *  @param[in] value 0 for low, 1 for high
 */
void LD2OS_setGpio(LoDi2Gpio pin, uint8_t value);


/** @brief Get gpio pin level which is used for LoDi2 board
 *  @param[in] pin Gpio pin
 *  @return level 0 for low, 1 for high
 */
uint8_t LD2OS_getGpio(LoDi2Gpio pin);


/** @brief Write to LoDi2 board via serial connection. Depending on HW configuration, the connection can be UART or SPI.
 *  @param[in] pucBuff Pointer to write buffer
 *  @param[in] ulLen Write size
 *  @return true for success. false for error.
 */
bool LD2OS_writeToSerial(const unsigned char *pucBuff, uint32_t ulLen);


/** @brief Read from LoDi2 board via serial connection. Depending on HW configuration, the connection can be UART or SPI.
 *  @param[in] pucBuff Pointer to read buffer
 *  @param[out] ulLen Pointer to return actual read size
 *  @param[in] ulReqLen Requested read size 
 *  @param[in] timeout Timeout to give up reading more data
 *  @return true for success. false for error.
 */
bool LD2OS_readFromSerial(unsigned char *pucBuff, uint32_t *ulLen, uint32_t ulReqLen, uint32_t timeout);


/** @brief Cancel the waiting in LD2OS_readFromSerial API.
 */
void LD2OS_cancelReadFromSerial(void);

/** @brief Assert called by LoDi2 when it find any anomalies.
 */
void LD2OS_assert(const char *format, ...);


/** @brief Print log lines from LoDi2
 */
void LD2OS_log(bool bFileOnly, const char *format, ...);

/** @brief Print hex bytes
 *  @param[in] from Pointer to hex bytes
 *  @param[in] size to print
 */
void LD2OS_dump(const uint8_t *from, const uint32_t size);


#define	LD2OS_O_RDONLY		1
#define LD2OS_O_WRONLY		2
#define LD2OS_O_RDWR		4
#define LD2OS_O_DIRECTORY	010
#define LD2OS_O_CREAT		0100
#define LD2OS_O_EXCL		0200
#define LD2OS_O_TRUNC		01000
#define LD2OS_O_APPEND		02000

/** @brief Open file
 *  @param[in] path file path
 *  @param[in] flags LD2OS_O_XXX
 *  @return pointer to file descriptor. If fails, returns null.
 */
void* LD2OS_openFile(const char *path, uint16_t flags);

/** @brief Close file
 *  @param[in] fd  Pointer to file descriptor
  */
void LD2OS_closeFile(void* fd);

/** @brief Empty serial data in the driver in previous run.
 */
void LD2OS_emptySerialBuffer();

/** @brief Read file
 *  @param[in] fd  pointer to file descriptor
 *  @param[in] buf buffer where to read into
 *  @param[in] size length to read
 *  @return actual length read
 */
int32_t LD2OS_readFile(const void* fd, void *buf, int32_t size);

/** @brief Write file
 *  @param[in] fd  pointer to file descriptor
 *  @param[in] buf buffer where to write from
 *  @param[in] size length to write
 *  @return actual length written
 */
int32_t LD2OS_writeFile(const void* fd, void *buf, int32_t size);


/** @brief Move file pointer
 *  @param[in] fd  pointer to file descriptor
 *  @param[in] offset file pointer from beginning of the file
 *  @return file pointer moved
 */
int32_t LD2OS_seekFile(const void* fd, int32_t offset);

typedef struct LD2OSFileInfo
{
    int64_t fsize;
} LD2OSFileInfo;

int32_t LD2OS_statFile(const char* path, LD2OSFileInfo* info);

void* LD2OS_openLog();

void LD2OS_closeLog();

/** @brief Open failsafe file to dump asic memory
 *  @return pointer to file descriptor. If fails, returns null.
 */
void* LD2OS_openFailSafe();

bool OpenTTY(const char* pcPortName, long lBaudRate);
bool OpenSPITTY(const char *device, long speed);
int Transfer_spi_buffers(int fd, void *tx_buffer, void *rx_buffer, size_t length);

/** @} */ // end of OsGlue
#endif //LODI2_OS_GLUE_LAYER_H
