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
 * Note:
 * Because of the nature of the ASF4 code I had to copy and modify the ASF4 file
 * usb_start.c and usb_start.h, which this USB_Serial.c and USB_Serial.h replace.
 *  There is no copyright information on those ASF4 files but whatever copyright applies
 * to them applies here as well.
 */

/*
* A working USBSerial implementation using only ASF4. I have tested this on a SAMD51
* ( Adafruit Metro M4 Express) but it should work on pretty much any ASF4 project with
* USB support.
*
* V0.01 December 20, 2020
*
* ************ Important: read the README
*
*/

#include "includes.h"
#include "atmel_start.h"
#include "USB_Serial.h"
#include "usb/class/cdc/device/cdcdf_acm.h"
//#include "board.h"
#include <utils_ringbuffer.h>
#include <stdio.h>

volatile uint8_t   ENDPOINT = 0;
char               USB_RING_BUFFER[ 16*4 ];
struct ringbuffer  USB_RING_BUFFER_STRUCT;

volatile bool      USB_FLOW_CONTROL = false;      //Enable flow control (only partly working)
volatile uint8_t   USB_DTR = DTR_OFF;             //Data Terminal Ready
volatile uint8_t   USB_DCE = MODEM_NO_CONNECT;    //Modem DCE
//
//volatile uint8_t USB_RTS;     //Since this is write I need a pointer to rs232 to set it
//                              //See the readme
//


#if CONF_USBD_HS_SP
// Device descriptors and Configuration descriptors list. 
static uint8_t single_desc_bytes[] = {
    CDCD_ACM_HS_DESCES_LS_FS};

// Device descriptors and Configuration descriptors list.
static uint8_t single_desc_bytes_hs[] = {
    CDCD_ACM_HS_DESCES_HS};
#define CDCD_ECHO_BUF_SIZ CONF_USB_CDCD_ACM_DATA_BULKIN_MAXPKSZ_HS

#else
// Device descriptors and Configuration descriptors list.
static uint8_t single_desc_bytes[] = {
    CDCD_ACM_DESCES_LS_FS};
#define CDCD_ECHO_BUF_SIZ CONF_USB_CDCD_ACM_DATA_BULKIN_MAXPKSZ

#endif

static struct usbd_descriptors single_desc[]
    = {{single_desc_bytes, single_desc_bytes + sizeof(single_desc_bytes)}

#if CONF_USBD_HS_SP
       ,
       {single_desc_bytes_hs, single_desc_bytes_hs + sizeof(single_desc_bytes_hs)}
#endif
};

// Buffer to receive and echo the communication bytes.
static uint32_t USB_DATA_BUFFER[CDCD_ECHO_BUF_SIZ / 4];

// Ctrl endpoint buffer 
static uint8_t ctrl_buffer[64];
static usb_cb_func_rxptr usb_tx_cb = NULL;

void USBSerial_set_rx_callback(usb_cb_func_rxptr ptr)
{
	usb_tx_cb = ptr;	
}

//
// Callback invoked when data recieved from PC (RX Data)
//
static bool USBSerialRecieveBlock(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count)
{
    ENDPOINT = ep;          //Capture the endpoint (used to avoid output overwrite)
    for( int i = 0; i < count; i++ )    //Stuff RX data into FIFO
	{
        ringbuffer_put( &USB_RING_BUFFER_STRUCT, *((uint8_t *)USB_DATA_BUFFER + i) );
		if(usb_tx_cb != NULL)
			usb_tx_cb(*((uint8_t *)USB_DATA_BUFFER + i));
	}

//Set to read again	
    cdcdf_acm_read((uint8_t *)USB_DATA_BUFFER, sizeof(USB_DATA_BUFFER)); 
	return false;
}

//
// Callback invoked after data sent 
// transmit    
static bool USBSerialTransmitBlock(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count)
{
    ENDPOINT = ep;          //Capture the endpoint (used to avoid output overwrite)
//Set to read again
	cdcdf_acm_read((uint8_t *)USB_DATA_BUFFER, sizeof(USB_DATA_BUFFER));
	return false;
}

//
// Callback invoked when Line State Change
// to do implement flow control
// https://www.ftdichip.com/Support/FAQs.htm
//
// 
//
static bool USBSerialStateChange(usb_cdc_control_signal_t state)
{
        USB_DTR = state.rs232.DTR;
        USB_DCE = state.modem.dte_present;

	if( state.rs232.DTR == DTR_ON ) {  //Only becomes true if PC connected and ready
                            //i.e. terminal emulator running on PC and USB cable connected
		cdcdf_acm_register_callback(CDCDF_ACM_CB_READ, (FUNC_PTR)USBSerialRecieveBlock);
		cdcdf_acm_register_callback(CDCDF_ACM_CB_WRITE, (FUNC_PTR)USBSerialTransmitBlock);
//
// Have to get the ball rolling with a dummy write then set up to read with call to
// cdcdf_acm_read()
//
        cdcdf_acm_write((uint8_t *)USB_DATA_BUFFER, 0);             
        cdcdf_acm_read((uint8_t *)USB_DATA_BUFFER, sizeof(USB_DATA_BUFFER));
	}
	return false;
}

//
// if 

//
// This is a stub called by atmel_start_init()
// Because it doesn't pass arguments, I made USBSerial_Init()
//
void usb_init(void)
{
    return;
}
//
// Initalize CDC USB Stack
// Returns a TIMEOUT code (a negative value) if times out
//
int USBSerial_Init( uint32_t aTimeout )
{
	if(!gpio_get_pin_level(USB_VBUS))
		return (VBUS_NO_HOST);
		
	usbdc_init(ctrl_buffer);        //Initialize the USB stack
	cdcdf_acm_init();               //brief Initialize USB CDC ACM Function Driver
	usbdc_start(single_desc);       //This is probably what needs to change for multiple endpoints
	usbdc_attach();
    //board_init();                   //Set up board specific stuff (clocks, etc)

    ringbuffer_init( &USB_RING_BUFFER_STRUCT, USB_RING_BUFFER, sizeof( USB_RING_BUFFER) );
			
	TickType_t ticks = xTaskGetTickCount();
 
    while(!cdcdf_acm_is_enabled())
    {
        if((xTaskGetTickCount() - ticks ) > aTimeout )
            return( ACM_TIMEOUT );           
		vTaskDelay(10); 
    }  
    //
    // Wait for the USB line state change (check DTR)
    //
    cdcdf_acm_register_callback(CDCDF_ACM_CB_STATE_C, (FUNC_PTR)USBSerialStateChange );

    while( ENDPOINT == 0 )
    {
        if(( xTaskGetTickCount() - ticks ) > aTimeout )
            return( DTR_TIMEOUT );            
		vTaskDelay(10);
    };       //Wait for the endpoint to be captured

	setbuf(stdin, NULL);        //No buffering on input (makes getchar work)
    return( 0 );
}

//
// Check if the USB output port is available
// Used to avoid overruns
//
bool    CheckUSBOutAvailable( void )
{
    struct usb_d_trans_status usbstatus;
    _usb_d_dev_ep_get_status( ENDPOINT | 0x80, &usbstatus);
    return( usbstatus.busy == 0 );
}

//
// Flush virtual serial port (otherwise waits for newline)
//
void    USBFlush( void )
{
    int time = 0;
    if( ENDPOINT != 0 )
    do
    {
        if( CheckUSBOutAvailable()) 
			return;
        vTaskDelay(1);
    } while( time++ < 100 );
}

//
// Check if there are input characters available
//
int CheckUSBSerialRXAvailable( void )
{
    return (int) ringbuffer_num( &USB_RING_BUFFER_STRUCT );
}

//
// If ENDPOINT = 0; USB is not attached
//
bool USBSerialAttached( void )
{
    return(( ENDPOINT != 0 ) && gpio_get_pin_level(USB_VBUS));//( USB_DCE == MODEM_DTE_PRESENT));
}

//
// If using flow control and USB is connected
//
void        USBSerialWaitDTR( void )
{
    if( USB_FLOW_CONTROL && USBSerialAttached())         //If USB is connected
        while( USB_DTR == DTR_OFF ){};  //Flow control to PC 
}

//
// Wait for input from USBSerial port
// to do: implement RTS flor control
//
int     USBSerialRead( )
{
uint8_t   retval = 0;

    while( ringbuffer_num( &USB_RING_BUFFER_STRUCT ) == 0 );
    ringbuffer_get( &USB_RING_BUFFER_STRUCT, &retval );
    return (int) retval;
}


//
// Write to the USBSerial port and flush it
// 
void USBSerialWrite( int c )
{
    USBSerialWaitDTR();     //If USB attached wait for DTR (flow control to PC)
    cdcdf_acm_write( (uint8_t*) &c, 1 );
    USBFlush();
}

#define USB_TX_CACHE_BFRS		16		// 16 * 300 = 4800
#define USB_TX_CACHE_SIZE		300		// 280 of protocol + some overhead
char usb_tx_cached[USB_TX_CACHE_BFRS][USB_TX_CACHE_SIZE];
uint16_t usb_tx_ready[USB_TX_CACHE_BFRS] = { 0, 0, 0, 0, 0, 0, 0, 0 };
uint8_t usb_tx_write_index = 0, usb_tx_read_index = 0;
int usb_tx_cached_idx = 0, usb_tx_cached_overflow = 0;
//
// Write to the USBSerial port and flush it
// 
void USBSerialWriteCached(int c)
{
    usb_tx_cached[usb_tx_write_index][usb_tx_cached_idx++] = c;
	if(usb_tx_cached_idx >= USB_TX_CACHE_SIZE || c == '\n')
	{
		if(usb_tx_ready[usb_tx_write_index] > 0)
			usb_tx_cached_overflow++;
			
		usb_tx_ready[usb_tx_write_index] = usb_tx_cached_idx;
		usb_tx_cached_idx = 0;
		if(++usb_tx_write_index >= USB_TX_CACHE_BFRS)
			usb_tx_write_index = 0;
	}
}

void USBSerialWriteUpdate()
{
	for(int i = 0; i < USB_TX_CACHE_BFRS; i++)
	{
		if(USBSerialAttached())
		{
			if(usb_tx_ready[usb_tx_read_index] > 0)
			{
				USBSerialWriteBfr((uint8_t *)usb_tx_cached[usb_tx_read_index], usb_tx_ready[usb_tx_read_index]);
				usb_tx_ready[usb_tx_read_index] = 0;
				if(++usb_tx_read_index >= USB_TX_CACHE_BFRS)
					usb_tx_read_index = 0;
			}
			else
				break;
		}
	}
}

//
// Write to the USBSerial port and flush it
// 
void USBSerialWriteBfr( uint8_t *bfr, uint16_t len )
{
	uint8_t send_len;
	for(int i = 0; i < len; i += 64)
    {
		send_len = (len - i) > 64 ? 64 : (len - i);
		USBSerialWaitDTR();     //If USB attached wait for DTR (flow control to PC)
		cdcdf_acm_write( (uint8_t*)(bfr + i), send_len);
		USBFlush();
	}
}

//
// Send and FLUSH
//
void USBSerialFlushBfr(  )
{
	if(usb_tx_cached_idx > 0)
	{
		//USBSerialWriteBfr(usb_tx_cached, usb_tx_cached_idx);
		if(usb_tx_ready[usb_tx_write_index] == 0)
		{
			usb_tx_ready[usb_tx_write_index] = usb_tx_cached_idx;
			if(++usb_tx_write_index >= USB_TX_CACHE_BFRS)
				usb_tx_write_index = 0;
			usb_tx_cached_idx = 0;
		}
	}
}

//
// stdio read function  
// this works for scanf however, getchar does not work as it expects a newline. 
// Therefore, instead of reverse engineering getchar (macros on macros on macros)
// I make USBSerialRead() (returns when a character is entered)
// and   CheckUSBSerialAvail(return nz when a character present)
//
int _read (int fd, const void *buf, size_t count)
{
int         receivedcount = 0;
uint8_t *   outbuf = ( uint8_t *) buf; //Basically recast buf

    for( size_t i = 0; i < count; i++ )
    {
            while( ringbuffer_num( &USB_RING_BUFFER_STRUCT ) == 0 ); //Wait for something available
        ++receivedcount;
        ringbuffer_get( &USB_RING_BUFFER_STRUCT, outbuf );
        if(( *outbuf == '\r' ) || (*outbuf == '\n'))
        {
            *outbuf = '\n';     //Because CRLF is a thing
            break;
        } 
        outbuf++;       
    }
    return receivedcount;
}

//
// stdio write function used by putchar and printf
//
int _write( int fd, const void *buf, size_t count )
{
    USBSerialWaitDTR();             //If USB attached wait for DTR (flow control to PC)
    cdcdf_acm_write( (uint8_t *) buf, count );
    USBFlush();
    return(( int ) count );
}


    