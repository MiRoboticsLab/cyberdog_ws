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

#include <sys/time.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <unistd.h>
#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <map>
#include <string>
#include <algorithm>
#include <mutex>
#include <syslog.h>

static void * g_fdLog = NULL;
static bool g_bEnabledCtsForMcuRdy = false;
static LoDi2SerialConnection g_connType = LODI2_SERIAL_UART;
/** Serial port */
int iPortFd = -1;

struct FtdiDevice
{
  int port;
  char bBusSn[16];
  char cBusSn[16];
};

using FtdiMap = std::map<std::string, FtdiDevice>;
FtdiMap g_ftdiMap;

static std::recursive_mutex LogLock;

void LD2OS_delay(uint32_t mSec)
{
  timespec tv;
  tv.tv_sec = 0;
  tv.tv_nsec = mSec * 1e6;
  nanosleep(&tv, NULL);
}

uint32_t LD2OS_getTime(void)
{
  static uint64_t timeOrigin = 0;

  struct timeval tv;
  gettimeofday(&tv, NULL);
  uint64_t time_ms = tv.tv_sec * 1000 + tv.tv_usec / 1000;
  if (!timeOrigin) {
    timeOrigin = time_ms;
  }

  return (uint32_t)(time_ms - timeOrigin);
}

#ifndef __APPLE__
#include <termio.h>
bool OpenTTY(const char * pcPortName, long lBaudRate)
{
  LD2_LOG("%s(\"%s\",%ld)\n", __FUNCTION__, pcPortName, lBaudRate);

  int iPortHandler;
  struct termios termios;
  speed_t speed;
  int openFlags = O_RDWR;
  openFlags |= O_NOCTTY;

  switch (lBaudRate) {
    case 9600: speed = B9600; break;
    case 19200: speed = B19200; break;
    case 38400: speed = B38400; break;
    case 57600: speed = B57600; break;
    case 115200: speed = B115200; break;
#ifdef B230400
    case 230400: speed = B230400; break;
#endif
#ifdef B460800
    case 460800: speed = B460800; break;
#endif
#ifdef B921600
    case 921600: speed = B921600; break;
#endif
#ifdef B1000000
    case 1000000: speed = B1000000; break;
#endif
#ifdef B1152000
    case 1152000: speed = B1152000; break;
#endif
#ifdef B1500000
    case 1500000: speed = B1500000; break;
#endif
#ifdef B2000000
    case 2000000: speed = B2000000; break;
#endif
#ifdef B2500000
    case 2500000: speed = B2500000; break;
#endif
#ifdef B3000000
    case 3000000: speed = B3000000; break;
#endif
#ifdef B3500000
    case 3500000: speed = B3500000; break;
#endif
#ifdef B4000000
    case 4000000: speed = B4000000; break;
#endif
    default:
      LD2_LOG("Error identifying port speed.\n");
      return false;
  }

  iPortHandler = open(pcPortName, openFlags);
  if (-1 == iPortHandler) {
    LD2_LOG("Open error %d %s\n", errno, strerror(errno));
    return false;
  }

  // close on exec
  int fd_flags = fcntl(iPortHandler, F_GETFL);
  if (fd_flags == -1) {
    LD2_LOG("Cannot get the file descriptor's flags, errno=%d.\n", errno);
    return false;
  }

  if (fcntl(iPortHandler, F_SETFL, fd_flags | FD_CLOEXEC) == -1) {
    LD2_LOG("Cannot set FD_CLOEXEC option, errno=%d.\n", errno);
    return false;
  }

  memset(&termios, 0, sizeof(termios));
  if (tcgetattr(iPortHandler, &termios) == -1) {
    LD2_LOG("Error reading GPS serial port config");
    close(iPortHandler);
    return false;
  }

  termios.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
  termios.c_oflag &= ~OPOST;
  termios.c_cflag &= ~(CSIZE | PARENB | HUPCL);

  termios.c_cflag |= CS8 | CREAD | CLOCAL | CRTSCTS;

  termios.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  termios.c_cc[VMIN] = 0;
  termios.c_cc[VTIME] = 1;
  cfsetispeed(&termios, speed);
  cfsetospeed(&termios, speed);
  tcflush(iPortHandler, TCIOFLUSH);

  if (tcsetattr(iPortHandler, TCSANOW, &termios) == -1) {
    LD2_LOG("Error configuring the GPS serial port\n");
    close(iPortHandler);
    return false;
  }

  tcflush(iPortHandler, TCIOFLUSH);

  if (tcgetattr(iPortHandler, &termios) == -1) {
    LD2_LOG("Error reading back the port configuration.\n");
  }


  // Set DTR
  int iFlags;

  // turn on DTR
  iFlags = TIOCM_DTR;
  ioctl(iPortHandler, TIOCMBIS, &iFlags);


#if 0
  LD2_LOG(
    "TTY <== i=0x%08X o=0x%08X c=0x%08X l=0x%08X m%d t%d %d %d\n",
    termios.c_iflag, termios.c_oflag,
    termios.c_cflag, termios.c_lflag,
    termios.c_cc[VMIN], termios.c_cc[VTIME],
    cfgetispeed(&termios), cfgetospeed(&termios));
  LD2_LOG("opened fd %d\n", iPortHandler);
#endif

  iPortFd = iPortHandler;

  LD2_LOG("%s(GPS: OPEN: %d)\n", __FUNCTION__, iPortFd);

  return true;
}


#else

#include <IOKit/serial/ioss.h>
#include <sys/ioctl.h>
bool OpenTTY(const char * pcPortName, long lBaudRate)
{

  struct              termios termios;
  unsigned long mics = 1;                                   // Receive latency, in microseconds
  speed_t speed = (speed_t)lBaudRate;
  int iPortHandler;
  int openFlags = O_RDWR | O_NOCTTY;

  iPortHandler = open(pcPortName, openFlags);

  if (-1 == iPortHandler) {
    LD2_LOG("Open %s failed.   error=%d %s\n", pcPortName, errno, strerror(errno));
    return false;
  }

  // GpsHalSetCloexecFlag(iPortHandler);

  /*  NOTE:  This initialization sequence is specific to OS X.  It was transcribed from
   *         code in:
   *
   *    http://opensource.apple.com/source/IOSerialFamily/IOSerialFamily-74/tests/IOSerialTestLib.c?txt
   *
   */

  //   prevent additional opens on the device, except from a root-owned process
  if (ioctl(iPortHandler, TIOCEXCL) == -1) {
    LD2_LOG("ERROR :  %s(): ioctl TIOCEXCL failed\n", __FUNCTION__);
    return false;
  }

  // snapshot the current terminal state in originalOptions
  if (tcgetattr(iPortHandler, &termios) == -1) {
    LD2_LOG("ERROR %s: tcgetattr failed\n", __FUNCTION__);
    return false;
  }

  // Set raw input (non-canonical) mode
  cfmakeraw(&termios);

  termios.c_cc[VMIN] = 0;
  termios.c_cc[VTIME] = 10;


  // Set 19200 baud as a default.  We will change it later
  if (cfsetspeed(&termios, B19200) == -1) {
    LD2_LOG("ERROR %s: cfsetspeed failed\n", __FUNCTION__);
    return false;
  }

  // Use 8 bit words, Parity is disabled, CTS and RTS enabled
  tcflag_t cflag_mask = CS8 | CLOCAL | CCTS_OFLOW | CRTS_IFLOW;

  termios.c_cflag = cflag_mask;

  // Cause the new options to take effect immediately.
  if (tcsetattr(iPortHandler, TCSANOW, &termios) == -1) {
    LD2_LOG("ERROR %s: tcsetattr failed\n", __FUNCTION__);
    return false;
  }

  // check that the tcsetattr worked properly
  if (tcgetattr(iPortHandler, &termios) == -1) {
    LD2_LOG("ERROR %s: tcgetattr failed\n", __FUNCTION__);
    return false;
  }


  if ((termios.c_cflag & (CS8 | CLOCAL | CCTS_OFLOW | CRTS_IFLOW)) != cflag_mask) {
    LD2_LOG("ERROR %s: tcsetattr/tcgetattr failed\n", __FUNCTION__);
    return false;
    //<rdar://problem/38829793> Resume UART flow control check
  }


  // Check that speed is 19200 baud
  if (cfgetispeed(&termios) != B19200 || cfgetospeed(&termios) != B19200) {
    LD2_LOG("ERROR %s: cfsetspeed failed\n", __FUNCTION__);
    return false;
  }

  // Set the receive latency in microseconds
  if (ioctl(iPortHandler, IOSSDATALAT, &mics) == -1) {
    LD2_LOG("error %s: ioctl IOSSDATALAT failed\n", __FUNCTION__);
    return false;
  }


  int handshake;

  // Assert Data Terminal Ready (DTR)
  if (ioctl(iPortHandler, TIOCSDTR) == -1) {
    LD2_LOG("ERROR %s: ioctl TIOCSDTR failed\n", __FUNCTION__);
    return false;
  }
  // Clear Data Terminal Ready (DTR)
  if (ioctl(iPortHandler, TIOCCDTR) == -1) {
    LD2_LOG("ERROR %s: ioctl TIOCCDTR failed\n", __FUNCTION__);
    return false;
  }

  // Set the modem lines depending on the bits set in handshake
  handshake = TIOCM_DTR | TIOCM_RTS | TIOCM_CTS | TIOCM_DSR;
  if (ioctl(iPortHandler, TIOCMSET, &handshake) == -1) {
    LD2_LOG("ERROR %s: ioctl TIOCMSET failed\n", __FUNCTION__);
    return false;
  }

  // Store the state of the modem lines in handshake
  if (ioctl(iPortHandler, TIOCMGET, &handshake) == -1) {
    LD2_LOG("ERROR %s: ioctl TIOCMGET failed\n", __FUNCTION__);
    return false;
  }

  // The IOSSIOSPEED ioctl can be used to set arbitrary baud rates other than
  // those specified by POSIX. The driver for the underlying serial hardware
  // ultimately determines which baud rates can be used. This ioctl sets both
  // the input and output speed.
  if (ioctl(iPortHandler, IOSSIOSPEED, &speed) == -1) {
    LD2_LOG("ERROR %s: unable to set desired baud rate of %lu).\n", __FUNCTION__, speed);
    return false;
  }

  // Check that speed is properly modified
  if (tcgetattr(iPortHandler, &termios) == -1) {
    LD2_LOG("ERROR %s: tcgetattr failed\n", __FUNCTION__);
    return false;
  }

  if (cfgetispeed(&termios) != speed || cfgetospeed(&termios) != speed) {
    LD2_LOG("ERROR %s: cfsetspeed failed, %lu, %lu.\n", __FUNCTION__, speed, cfgetispeed(&termios));
    return false;
  }


  iPortFd = iPortHandler;

  LD2_LOG("%s(GPS: OPEN: %d)\n", __FUNCTION__, iPortFd);


  return true;
}
#endif

bool LD2OS_open(LoDi2SerialConnection connType, int port, int baudrate, const char * tty_str)
{

  if (iPortFd >= 0) {
    LD2OS_close();
  }

  if (connType != LODI2_SERIAL_UART) {
    LD2_LOG("Error: cannot support the connection type = %d\n", connType);
    return false;
  }

  return OpenTTY((const char *) tty_str, (long) baudrate);
}

void LD2OS_emptySerialBuffer()
{
  // Flush away any bytes previously read or written.
  int result = tcflush(iPortFd, TCIOFLUSH);
  if (result) {
    LD2_LOG("tcflush failed");
  }
}

void LD2OS_close(void)
{
  close(iPortFd);
}

void LD2OS_enableCtsForMcuRdy(bool enable)
{
  if (g_connType != LODI2_SERIAL_UART) {
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
  return 1;
}

void LD2OS_setGpio(LoDi2Gpio pin, uint8_t value)
{
}

uint8_t LD2OS_getGpio(LoDi2Gpio pin)
{
  return 1;
}

bool LD2OS_writeToSerial(const unsigned char * pcBuff, uint32_t ulLen)
{
  ssize_t bytes = 0;
  uint32_t writtenBytes = 0;

  while (writtenBytes < (ssize_t) ulLen) {
    const unsigned char * buff = pcBuff + writtenBytes;
    bytes = write(iPortFd, buff, ulLen - writtenBytes);
    if (bytes == -1) {
      return false;
    }
    writtenBytes += bytes;
  }
  return writtenBytes == ulLen;
}

void LD2OS_cancelReadFromSerial(void)
{
  // SetEvent(hEvent);
}

bool LD2OS_readFromSerial(
  unsigned char * pcBuff, uint32_t * ulLen, uint32_t ulReqLen,
  uint32_t timeout)
{

  LD2_ASSERT(pcBuff);
  LD2_ASSERT(ulLen);

  if (g_connType != LODI2_SERIAL_UART) {
    LD2_LOG("Error: not supported connType = %d\n", g_connType);
    return false;
  }

  *ulLen = 0;

  size_t ulRead = read(iPortFd, pcBuff, ulReqLen);
  if (ulRead == -1) {
    return false;
  }
  *ulLen = ulRead;

  return true;
}

void * LD2OS_openFile(const char * path, uint16_t flags)
{
  std::lock_guard<std::recursive_mutex> lk(LogLock);
  LD2_ASSERT(path);
  const char * modes[] = {"rb", "wb"};
  const char * mode = (flags == LD2OS_O_RDONLY) ? modes[0] : modes[1];
  FILE * fd = fopen(path, mode);
  LD2_ASSERT(fd);
  return (void *)fd;
}

void LD2OS_closeFile(void * fd)
{
  std::lock_guard<std::recursive_mutex> lk(LogLock);
  LD2_ASSERT(fd);
  fclose((FILE *)fd);
}

int32_t LD2OS_readFile(const void * fd, void * buf, int32_t size)
{

  std::lock_guard<std::recursive_mutex> lk(LogLock);
  LD2_ASSERT(fd);
  LD2_ASSERT(size > 0);

  size_t bytesRead = fread(buf, 1, size, (FILE *)fd);
  return (int32_t)bytesRead;
}

int32_t LD2OS_writeFile(const void * fd, void * buf, int32_t size)
{
  std::lock_guard<std::recursive_mutex> lk(LogLock);
  LD2_ASSERT(fd);
  LD2_ASSERT(buf);
  LD2_ASSERT(size > 0);

  size_t bytesWritten = fwrite(buf, 1, size, (FILE *)fd);
  return (int32_t)bytesWritten;
}

int32_t LD2OS_seekFile(const void * fd, int32_t offset)
{
  std::lock_guard<std::recursive_mutex> lk(LogLock);
  LD2_ASSERT(fd);
  LD2_ASSERT(offset >= 0);

  int32_t ret = fseek((FILE *)fd, offset, SEEK_SET);
  LD2_ASSERT(ret > 0);
  return ret;
}

int32_t LD2OS_statFile(const char * path, LD2OSFileInfo * info)
{
  return 1;
}

void * LD2OS_openLog(std::string log_path)
{
  // std::lock_guard<std::recursive_mutex> lk(LogLock);
  if (g_fdLog) {
    return g_fdLog;
  }

  char fname[128];
  time_t rawtime;
  struct tm * ptm;

  time(&rawtime);
  ptm = localtime(&rawtime);
  sprintf(
    fname, "%sgl-%04d-%02d-%02d-%02d-%02d-%02d.log", log_path.c_str(), ptm->tm_year + 1900,
    ptm->tm_mon + 1, ptm->tm_mday, ptm->tm_hour, ptm->tm_min, ptm->tm_sec);
  g_fdLog = LD2OS_openFile(fname, LD2OS_O_RDWR | LD2OS_O_CREAT);
  LD2_ASSERT(g_fdLog);
  return g_fdLog;
}

void LD2OS_closeLog()
{
  if (g_fdLog) {
    LD2OS_closeFile((void *)g_fdLog);
    g_fdLog = 0;
  }
}

void * LD2OS_openFailSafe()
{
  char fname[128];
  time_t rawtime;
  struct tm * ptm;

  time(&rawtime);
  ptm = localtime(&rawtime);
  sprintf(
    fname, "%sfailsafe-%04d-%02d-%02d-%02d-%02d-%02d.bin", LD2_FAILSAFE_DIR,
    ptm->tm_year + 1900, ptm->tm_mon + 1, ptm->tm_mday, ptm->tm_hour, ptm->tm_min,
    ptm->tm_sec);
  void * fd = LD2OS_openFile(fname, LD2OS_O_RDWR | LD2OS_O_CREAT);
  LD2_ASSERT(fd);
  return fd;
}

typedef void (* assertcb)(const char * msg);
static assertcb OnAssert = nullptr;
void LD2OS_setAssertCb(assertcb callback)
{
  OnAssert = callback;
}

void LD2OS_assert(const char * format, ...)
{
  char buf[256];
  char * pbuf = buf;
  pbuf += sprintf(pbuf, "[%10u] ASSERT : ", LD2OS_getTime());
  va_list arg_ptr;
  va_start(arg_ptr, format);
  pbuf += vsprintf(pbuf, format, arg_ptr);
  va_end(arg_ptr);
  //printf("%s\n", buf);
  syslog(LOG_NOTICE, "%s", buf);
  fflush(stdout);

  if (g_fdLog) {
    std::lock_guard<std::recursive_mutex> lk(LogLock);
    size_t ret = fwrite(buf, 1, strlen(buf), (FILE *)g_fdLog);
    if (ret == 0) {
      //printf("write error %d\r\n", errno);
      syslog(LOG_NOTICE, "write error %d\r\n", errno);
    }
    LD2OS_closeLog();
  }
  if (OnAssert) {
    OnAssert(buf);
  } else {
    exit(-1);
  }
  //TODO: rename suffix into *.a.log
}

void LD2OS_log(bool bFileOnly, const char * format, ...)
{
  std::lock_guard<std::recursive_mutex> lk(LogLock);

  char buf[4096];
  char * pbuf = buf;
  pbuf += sprintf(pbuf, "[%10u] ", LD2OS_getTime());
  va_list arg_ptr;
  va_start(arg_ptr, format);
  pbuf += vsprintf(pbuf, format, arg_ptr);
  va_end(arg_ptr);
  if (!bFileOnly) {
    //printf("%s", buf);
    //syslog(LOG_NOTICE,"%s", buf);
    fflush(stdout);
  }

  if (!g_fdLog) {return;}
  size_t ret = fwrite(buf, 1, strlen(buf), (FILE *)g_fdLog);
  if (ret == 0) {
    //printf("Error: write error %d\r\n", errno);
    syslog(LOG_NOTICE, "Error: write error %d\r\n", errno);
  }
}

void LD2OS_dump(const uint8_t * from, const uint32_t size)
{
  for (uint32_t i = 0; i < size; i++) {
    //printf("%02X ", from[i]);
    syslog(LOG_NOTICE, "%02X ", from[i]);
  }
  //printf("\n");
  fflush(stdout);
}
