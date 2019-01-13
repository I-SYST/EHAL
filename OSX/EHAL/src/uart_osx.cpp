/*--------------------------------------------------------------------------
 File   : uart_osx.cpp
 
 Author : Hoang Nguyen Hoan          Sep. 3, 2016
 
 Desc   :  uart OSX implementation
 
 Copyright (c) 2016, I-SYST inc., all rights reserved
 
 Permission to use, copy, modify, and distribute this software for any purpose
 with or without fee is hereby granted, provided that the above copyright
 notice and this permission notice appear in all copies, and none of the
 names : I-SYST or its contributors may be used to endorse or
 promote products derived from this software without specific prior written
 permission.
 
 For info or contributing contact : hnhoan at i-syst dot com
 
 THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
 EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 
 ----------------------------------------------------------------------------
 Modified by          Date              Description
 
 ----------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <errno.h>
//#include <IOKit/IOKitLib.h>
#include <IOKit/serial/IOSerialKeys.h>
#include <IOKit/serial/ioss.h>
#include <IOKit/IOBSD.h>

#include "uart_osx.h"
/*
static OSXUARTDEV s_UartDev[] = {
    
};
*/
void UARTSetCtrlLineState(UARTDEV *pDev, uint32_t LineState)
{
    
}

int OsxUARTGetRate(DEVINTRF *pDev)
{
    return 0;
}

int OsxUARTSetRate(DEVINTRF *pDev, int Rate)
{
    return 0;
}

static inline bool OsxUARTStartRx(DEVINTRF *pSerDev, int DevAddr) {
    return true;
}

int OsxUARTRxData(DEVINTRF *pDev, uint8_t *pBuff, int Bufflen)
{
    int cnt = 0;
    
    if (pDev)
    {
        OSXUARTDEV *dev = (OSXUARTDEV *)pDev->pDevData;
        if (dev)
        	cnt = (int)read(dev->hDevFile, pBuff, Bufflen);
/*		printf("OsxUartRxData : ")
		;		for (int i = 0; i < cnt; i++)
		{
			printf("%02x ", pBuff[i]);
		}
		printf("\n");*/
    }
    
/*    if (cnt < 0)
    {
        printf("error reading\n");
    }*/
    return cnt;
}

static inline void OsxUARTStopRx(DEVINTRF *pSerDev) {}

static inline bool OsxUARTStartTx(DEVINTRF *pDev, int DevAddr) {
    return true;
}

int OsxUARTTxData(DEVINTRF *pDev, uint8_t *pData, int Datalen)
{
    int cnt = 0;
    
    if (pDev)
    {
        OSXUARTDEV *dev = (OSXUARTDEV *)pDev->pDevData;
        if (dev)
    		cnt = (int)write(dev->hDevFile, pData, Datalen);
    }
    
    return cnt;
}

static inline void OsxUARTStopTx(DEVINTRF *pDev) {}

void OsxUARTDisable(DEVINTRF *pDev)
{
    if (pDev)
    {
        OSXUARTDEV *dev = (OSXUARTDEV *)pDev->pDevData;
        if (dev)
        {
            close(dev->hDevFile);
            dev->hDevFile = -1;
        }
    }
}

void OsxUARTEnable(DEVINTRF *pDev)
{
    if (pDev)
    {
        OSXUARTDEV *dev = (OSXUARTDEV *)pDev->pDevData;
        if (dev)
        {
    		dev->hDevFile = open(dev->DevPath, O_RDWR | O_NOCTTY | O_NONBLOCK);
        }
    }
}



bool UARTInit(UARTDEV *pDev, const UARTCFG *pCfgData)
{
	if (pDev == NULL || pCfgData == NULL)
        return false;
    
    struct termios OrigTTYAttrs;
    char *pathname = (char *)pCfgData->pIoMap;
    int hdev = open(pathname, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (hdev == -1)
        return false;
    
    // Note that open() follows POSIX semantics: multiple open() calls to the same file will succeed
    // unless the TIOCEXCL ioctl is issued. This will prevent additional opens except by root-owned
    // processes.
    // See tty(4) <x-man-page//4/tty> and ioctl(2) <x-man-page//2/ioctl> for details.
    
    if (ioctl(hdev, TIOCEXCL) == -1)
    {
        close(hdev);
        return false;
    }
    
    // Now that the device is open, clear the O_NONBLOCK flag so subsequent I/O will block.
    // See fcntl(2) <x-man-page//2/fcntl> for details.
    
    if (fcntl(hdev, F_SETFL, 0) == -1)
    {
        close(hdev);
        return false;
    }
    
    // Get the current options and save them so we can restore the default settings later.
    if (tcgetattr(hdev, &OrigTTYAttrs) == -1)
    {
        close(hdev);
        return false;
    }
    
    // The serial port attributes such as timeouts and baud rate are set by modifying the termios
    // structure and then calling tcsetattr() to cause the changes to take effect. Note that the
    // changes will not become effective without the tcsetattr() call.
    // See tcsetattr(4) <x-man-page://4/tcsetattr> for details.
    
    struct termios options = OrigTTYAttrs;
    
    // Set raw input (non-canonical) mode, with reads blocking until either a single character
    // has been received or a one second timeout expires.
    // See tcsetattr(4) <x-man-page://4/tcsetattr> and termios(4) <x-man-page://4/termios> for details.
    
    cfmakeraw(&options);
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 10;
    
    // The baud rate, word length, and handshake options can be set as follows:
    
    cfsetspeed(&options, pCfgData->Rate);
	options.c_cflag |= CS8;
    if (pCfgData->FlowControl == UART_FLWCTRL_HW)
	{
		options.c_cflag |= CCTS_OFLOW | CRTS_IFLOW;
	}
	else
	{
		options.c_cflag &= ~(CCTS_OFLOW | CRTS_IFLOW);
	}
    // Print the new input and output baud rates. Note that the IOSSIOSPEED ioctl interacts with the serial driver
    // directly bypassing the termios struct. This means that the following two calls will not be able to read
    // the current baud rate if the IOSSIOSPEED ioctl was used but will instead return the speed set by the last call
    // to cfsetspeed.
    
    //printf("Input baud rate changed to %d\n", (int) cfgetispeed(&options));
    //printf("Output baud rate changed to %d\n", (int) cfgetospeed(&options));
    
    // Cause the new options to take effect immediately.
 /*   if (tcsetattr(hdev, TCSANOW, &options) == -1) {
        printf("Error calling ioctl(..., IOSSIOSPEED, ...) %s - %s(%d).\n",
               pathname, strerror(errno), errno);
        close(hdev);
        return false;
    }
   */
    // The IOSSIOSPEED ioctl can be used to set arbitrary baud rates
    // other than those specified by POSIX. The driver for the underlying serial hardware
    // ultimately determines which baud rates can be used. This ioctl sets both the input
    // and output speed.
    
    speed_t speed = pCfgData->Rate; // Set 14400 baud
    if (ioctl(hdev, IOSSIOSPEED, &speed) == -1) {
    }
    
    // To set the modem handshake lines, use the following ioctls.
    // See tty(4) <x-man-page//4/tty> and ioctl(2) <x-man-page//2/ioctl> for details.
    
    // Assert Data Terminal Ready (DTR)
    if (ioctl(hdev, TIOCSDTR) == -1) {
    }
    
    // Clear Data Terminal Ready (DTR)
    if (ioctl(hdev, TIOCCDTR) == -1) {
    }
    
    // Set the modem lines depending on the bits set in handshake
    int handshake = TIOCM_DTR | TIOCM_RTS | TIOCM_CTS | TIOCM_DSR;
    if (ioctl(hdev, TIOCMSET, &handshake) == -1) {
    }
    
    // To read the state of the modem lines, use the following ioctl.
    // See tty(4) <x-man-page//4/tty> and ioctl(2) <x-man-page//2/ioctl> for details.
    
    // Store the state of the modem lines in handshake
    if (ioctl(hdev, TIOCMGET, &handshake) == -1) {
        printf("Error getting handshake lines %s - %s(%d).\n",
               pathname, strerror(errno), errno);
    }
    
    //printf("Handshake lines currently set to %d\n", handshake);
    
    unsigned long mics = 1UL;
    
    // Set the receive latency in microseconds. Serial drivers use this value to determine how often to
    // dequeue characters received by the hardware. Most applications don't need to set this value: if an
    // app reads lines of characters, the app can't do anything until the line termination character has been
    // received anyway. The most common applications which are sensitive to read latency are MIDI and IrDA
    // applications.
    
    if (ioctl(hdev, IOSSDATALAT, &mics) == -1) {
        // set latency to 1 microsecond
        printf("Error setting read latency %s - %s(%d).\n",
               pathname, strerror(errno), errno);
        close(hdev);
        return false;
    }
    
    OSXUARTDEV *osxdev = new OSXUARTDEV;
    
    osxdev->DevNo = 0;
    strcpy(osxdev->DevPath, (char*)pCfgData->pIoMap);
    osxdev->hDevFile = hdev;
    
    pDev->DevIntrf.pDevData = osxdev;
    osxdev->pUartDev = pDev;
    osxdev->OrigTTYAttrs = OrigTTYAttrs;
    
    pDev->EvtCallback = pCfgData->EvtCallback;
    pDev->DevIntrf.Disable = OsxUARTDisable;
    pDev->DevIntrf.Enable = OsxUARTEnable;
    pDev->DevIntrf.GetRate = OsxUARTGetRate;
    pDev->DevIntrf.SetRate = OsxUARTSetRate;
    pDev->DevIntrf.StartRx = OsxUARTStartRx;
    pDev->DevIntrf.RxData = OsxUARTRxData;
    pDev->DevIntrf.StopRx = OsxUARTStopRx;
    pDev->DevIntrf.StartTx = OsxUARTStartTx;
    pDev->DevIntrf.TxData = OsxUARTTxData;
    pDev->DevIntrf.StopTx = OsxUARTStopTx;
    pDev->DevIntrf.bBusy = false;
    
    return true;
}
