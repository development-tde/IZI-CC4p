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

#include "atmel_start.h"
#include "atmel_start_pins.h"
#include <stdio.h>
//#include "board.h"
#include "utils_ringbuffer.h"
#include "hpl_usb_device.h"
#include "usb/class/cdc/device/cdcdf_acm.h"

#include "USB_Serial.h"

void    strout( char *astring )
{
    while( *astring != '\0')
        USBSerialWrite( (int) *astring++ );
}

int main(void)
{
	atmel_start_init();             //Set up various stacks, IO, etc
    USB_FLOW_CONTROL = true;        //Set to false or comment out if no flow control desired
//
// On a SAMD51 @ 120MHz, startup takes less than 300mSec if USB is attached
// 
int initrv = USBSerial_Init( 600 );            
    //printf("\n*** Start ret code %d\n", initrv );
    
    while( true )
    {
int inp;        
            //printf( "Type characters uses getchar. End with return %d \n", (int) SECONDS );
            
            do 
            {
                inp = getchar(); 
                USBSerialWrite( inp );
            } while (( inp != '\n') && ( inp != '\r' ));

            //printf("\n\n");
    }    
}
