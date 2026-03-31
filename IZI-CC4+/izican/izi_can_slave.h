/*
 * izi_can_slave.h
 *
 *  Created on: 8 jul. 2020
 *      Author: Milo
 */

#ifndef IZI_CAN_SLAVE_H_
#define IZI_CAN_SLAVE_H_

#include "logger_data.h"

#define LOGWRITE_CRITICAL(errcode, text, log_source)	izican_log_write(errcode, text, log_critical, 0, log_source)
#define LOGWRITE_ERROR(errcode, text, log_source)		izican_log_write(errcode, text, log_error, 0, log_source)
#define LOGWRITE_WARNING(errcode, text, log_source)		izican_log_write(errcode, text, log_warning, 0, log_source)
#define LOGWRITE_INFO(errcode, text, log_source)		izican_log_write(errcode, text, log_info, 0, log_source)
#define LOGWRITE_TRACE(errcode, text, log_source)		izican_log_write(errcode, text, log_trace, 0, log_source)

#define LOGWRITE_CRITICAL_S(errcode, text, log_source, serial)		izican_log_write(errcode, text, log_critical, serial, log_source)
#define LOGWRITE_ERROR_S(errcode, text, log_source, serial)		izican_log_write(errcode, text, log_error, serial, log_source)
#define LOGWRITE_WARNING_S(errcode, text, log_source, serial)		izican_log_write(errcode, text, log_warning, serial, log_source)
#define LOGWRITE_INFO_S(errcode, text, log_source, serial)		izican_log_write(errcode, text, log_info, serial, log_source)

// Proto
void izican_slave_init(void);
void izican_log_write(uint32_t errorcode, char *text, log_severity_t severity, uint32_t serial, log_source_t log_source);

void izican_dbg_init();
void izican_slave_send();
void izican_slave_putc(char c);
bool izican_slave_getc(char *c);
bool izican_slave_nodata();
void izican_slave_fastrequest();

#endif /* IZI_CAN_SLAVE_H_ */
