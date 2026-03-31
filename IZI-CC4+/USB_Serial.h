/*
 * MIT License
 * Copyright (C) 2020 Brian Piccioni brian@documenteddesigns.com
 * @author Brian Piccioni <brian@documenteddesigns.com>
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
*/

#ifndef USB_SERIAL_H
#define USB_SERIAL_H

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#include "usb/class/cdc/device/cdcdf_acm.h"
#include "usb/class/cdc/device/cdcdf_acm_desc.h"

#define ACM_TIMEOUT -1
#define DTR_TIMEOUT -2
#define VBUS_NO_HOST -3

#define DTR_ON              1   //USB attached and ready to recieve
#define DTR_OFF             0   //USB may (see DTE) not ready to recieve
#define MODEM_NO_CONNECT    0   //USB port disabled PC side
#define MODEM_DTE_ABSENT    2   //USB port enabled, not attached
#define MODEM_DTE_PRESENT   3   //USB port enabled, PC attached

extern  volatile bool      USB_FLOW_CONTROL;    //Enable flow control (only partly working)
extern  volatile uint8_t   USB_DTR;             //Data Terminal Ready
extern  volatile uint8_t   USB_DCE;             //Modem DCE


void    usb_init(void);
int     USBSerial_Init( uint32_t aTimeout );
bool    CheckUSBOutAvailable( void );       //Check if output can be written 
void    USBFlush( void );                   //Flush the output buffer (send to terminal)
int     CheckUSBSerialRXAvailable( void );  //Any data in (like UART RX Ready)
int     USBSerialRead( );                   //Wait for input from USB Serial and return with it
void    USBSerialWrite( int c );            //Write to USB Serial and flush the buffer
void USBSerialWriteCached( int c );
void USBSerialWriteUpdate();
void    USBSerialWriteBfr( uint8_t *bfr, uint16_t len );
void    USBSerialFlushBfr();
bool    USBSerialAttached( void );          //True if USB Serial is attached and ready
void    USBSerialWaitDTR( void );           //If USB_FLOW_CONTROL true wait for DTR 
//
typedef void (*usb_cb_func_rxptr)(unsigned char c);
void USBSerial_set_rx_callback(usb_cb_func_rxptr ptr);
// stdio getchar() in the standard library does not work as it expects a newline.
// user USBSerialRead instead. printf, scanf, putchar() all work
//
int     _read (int fd, const void *buf, size_t count);  // stdio used by scanf, etc
int     _write( int fd, const void *buf, size_t count ); // stdio used by printf, getchr

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // USB_SERIAL_H
