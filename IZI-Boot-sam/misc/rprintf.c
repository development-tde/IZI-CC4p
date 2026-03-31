/*---------------------------------------------------*/
/* Public Domain version of printf                   */
/*                                                   */
/* Rud Merriam, Compsult, Inc. Houston, Tx.          */
/*                                                   */
/* For Embedded Systems Programming, 1991            */
/*                                                   */
/*---------------------------------------------------*/

/*---------------------------------------------------*/
/* Example of usage:                                 */
/* void putc(unsigned char c);  //function prototype */
/*                                                   */
/* esp_printf(putc, "val=%02x %ld kilo\n",a,b);      */
/*                                                   */
/*---------------------------------------------------*/

#include "includes.h"
#include <stdio.h>
#include <ctype.h>

/*---------------------------------------------------*/
/* The purpose of this routine is to output data the */
/* same as the standard printf function without the  */
/* overhead most run-time libraries involve. Usually */
/* the printf brings in many kilobytes of code and   */
/* that is unacceptable in most embedded systems.    */
/*---------------------------------------------------*/

static struct
{
	char f_left:	1;
	char f_long:	1;
	char f_pad: 	1;
	char f_dot: 	1;
	char f_neg: 	1;
} flags;

static func_ptr out_char;
static int pad_len;
static int num2;
static char pad_character;

static char outnum_buf[16];

const char digits[] = "0123456789ABCDEF";

/*---------------------------------------------------*/
/*                                                   */
/* This routine puts pad characters into the output  */
/* buffer.                                           */
/*                                                   */
static void padding(int num_pad_chars)
{
	if(flags.f_pad)
	{
		while(num_pad_chars > 0)
		{
			out_char(pad_character);
			num_pad_chars--;
		}
	}
}

/*---------------------------------------------------*/
/*                                                   */
/* This routine moves a string to the output buffer  */
/* as directed by the padding and positioning flags. */
/*                                                   */
static void outs(charptr lp)
{
	int len;
	
	/* pad on left if needed                          */
	len = strlen(lp);
	if(!flags.f_left) {
		padding(pad_len - len);
	}

	/* Move string to the buffer                      */
	while (*lp && num2--) {
		out_char( *lp++);
	}

	/* Pad on right if needed                         */
	if(flags.f_left) {
		padding(pad_len - len);
	}
}

/*---------------------------------------------------*/
/*                                                   */
/* This routine moves a number to the output buffer  */
/* as directed by the padding and positioning flags. */
/*                                                   */
static void outnum( long num, int base)
{
	int len;
	charptr cp;

	/* Check if number is negative (only when base == -10) */
	flags.f_neg = 0;
	if (base == -10) {
		base = -base;
		if (num < 0L) {
			flags.f_neg = 1;
			num = -num;
		}
	}

	/* Build number (backwards) in outnum_buf             */
	cp = outnum_buf;
	do
	{
		*cp++ = digits[(int)((unsigned long)num % base)];
		num = (unsigned long)num / base;
	} while (num > 0);

	if (flags.f_neg) {
		*cp++ = '-';
	}
	*cp-- = 0;

	/* Move the converted number to the buffer and    */
	/* add in the padding where needed.               */
	len = strlen(outnum_buf);
	if(!flags.f_left) {
		padding(pad_len - len);
	}
	while (cp >= outnum_buf) {
		out_char(*cp--);
	}
	if(flags.f_left) {
		padding(pad_len - len);
	}
}

/*---------------------------------------------------*/
/*                                                   */
/* This routine gets a number from the format        */
/* string.                                           */
/*                                                   */
static int getnum(charptr *linep)
{
	int n = 0;

	while (isdigit(**linep))
	{
		n = n*10 + ((**linep) - '0');
		(*linep)++;
	}
	
	return(n);
}

/*---------------------------------------------------*/
/*                                                   */
/* This routine operates just like a printf/sprintf  */
/* routine. It outputs a set of data under the       */
/* control of a formatting string. Not all of the    */
/* standard C format control are supported. The ones */
/* provided are primarily those needed for embedded  */
/* systems work. Primarily the floaing point         */
/* routines are omitted. Other formats could be      */
/* added easily by following the examples shown for  */
/* the supported formats.                            */
/*                                                   */

void esp_printf(const func_ptr f_ptr, charptr src, charptr ctrl, ...)
{
	va_list argp;

	va_start( argp, ctrl);
	esp_printf_arg(f_ptr, src, ctrl, argp);
	va_end (argp);
}

//
//In stead of using esp_printf for 'standard debug printing', the esp_printf_arg
//can also be called directly, in the same way as esp_printf does.
//This can be used for passing the variable argument list from other functions.
//For instance: mdm_dgb_print("Modem state is %d\r", mdm_state); where
//mdm_dbg_print is the function for all modem debug printing. In this function
//a trailer can be printed, like [MDM] and a test can be added if debug printing
//is enabled for the modem.
//
void esp_printf_arg(const func_ptr f_ptr, charptr src, charptr ctrl, va_list argp)
{
	char ch;

	out_char = f_ptr;

	for ( ; *src; src++)
	{
		if(*src == 0)
			break;
		out_char(*src);
	}
	for ( ; *ctrl; ctrl++) {

		/* move format string chars to buffer until a  */
		/* format control is found.                    */
		if (*ctrl != '%') {
#ifdef RPRINTF_ADD_CR2LF
			if(*ctrl == '\n') {
				out_char('\r');
			}
#endif
			out_char(*ctrl);
			continue;
		}

		/* initialize all the flags for this format.   */
		*(char *)&flags = 0;
		pad_character = ' ';
		num2 = 32767;

try_next:
		ch = *(++ctrl);

		if (isdigit(ch)) {
			if (flags.f_dot) {
				num2 = getnum(&ctrl);
			}
			else {
				if (ch == '0') {
					pad_character = '0';
				}
				pad_len = getnum(&ctrl);
				flags.f_pad = 1;
			}
			ctrl--;
			goto try_next;
		}

		switch (tolower(ch)) {
			case '%':
				out_char( '%');
				continue;

			case '-':
				flags.f_left = 1;
				break;

			case '.':
				flags.f_dot = 1;
				break;

			case 'l':
				flags.f_long = 1;
				break;

			case 'u':
				if (flags.f_long) {
					outnum( va_arg(argp, unsigned long), 10);
					continue;
				}
				else {
					outnum( va_arg(argp, unsigned int), 10);
					continue;
				}
			case 'd':
				if (flags.f_long || ch == 'D') {
					outnum( va_arg(argp, long), -10);
					continue;
				}
				else {
					outnum( va_arg(argp, int), -10);
					continue;
				}
			case 'x':
				if (flags.f_long) {
					outnum( (unsigned long)va_arg(argp, unsigned long), 16);
				} else {
					outnum( (unsigned long)va_arg(argp, unsigned int), 16);
				}
				continue;

			case 's':
				outs( va_arg( argp, charptr));
				continue;

			case 'c':
				out_char( va_arg( argp, int));
				continue;

//			case '\\':
//				switch (*ctrl) {
//					case 'a':
//						out_char( 0x07);
//						break;
//					case 'h':
//						out_char( 0x08);
//						break;
//					case 'r':
//						out_char( 0x0D);
//						break;
//					case 'n':
//						out_char( 0x0D);
//						out_char( 0x0A);
//						break;
//					default:
//						out_char( *ctrl);
//						break;
//				}
//				ctrl++;
//				break;

			default:
				continue;
		}
		goto try_next;
	}
}


/*---------------------------------------------------*/
