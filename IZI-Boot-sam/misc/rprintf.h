/*
 * rprintf.h
 *
 *  Created on: 3 aug. 2011
 *      Author: ZeveringM
 */

#ifndef RPRINTF_H_
#define RPRINTF_H_

#include <stdarg.h>

typedef char* charptr;
typedef void (*func_ptr)(unsigned char c);

// Proto
void esp_printf(const func_ptr f_ptr, charptr src, charptr ctrl, ...);
void esp_printf_arg(const func_ptr f_ptr, charptr src, charptr ctrl, va_list argp);

#endif /* RPRINTF_H_ */
