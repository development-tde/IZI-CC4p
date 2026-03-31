/*
 * izican_slave_handler.h
 *
 *  Created on: 9 jul. 2020
 *      Author: Milo
 */

#ifndef IZICAN_SLAVE_HANDLER_H_
#define IZICAN_SLAVE_HANDLER_H_

#include "izi_module.h"
#include "izi_can_frame.h"
#include "logger_data.h"

#define RAM_APP_PROG_SIZE		(32 * 1024)		// Max 32k prog data

void izican_slave_notification_handler(izi_can_extid_u id, izican_notification_data_t *data, uint16_t length);
void izican_slave_request_handler(izi_can_extid_u id, izican_request_data_t *data, uint16_t length);

bool izican_slave_reportmodule(uint16_t index, uint16_t total, izi_module_base_t *module, uint8_t *channels, uint8_t *config_amount, uint8_t *monitor_amount, uint8_t *monitor_status_idx, uint8_t *monitor_input_idx, uint8_t *monitor_encoder_idx);
bool izican_slave_notifymodule(uint16_t index, uint16_t total, izi_module_base_t *module);
bool izican_slave_state(uint16_t state, uint16_t total, uint8_t pl_freq, uint8_t pl_rate, uint16_t dmx_per_sec, uint8_t izi_req_per_sec, uint8_t izi_rsp_quality, uint8_t scan_amount, uint8_t channels, uint8_t *scan_res);
bool izican_slave_base_state(uint16_t state, uint32_t errors, uint32_t warnings, uint32_t start_info, char *txt);
bool izican_slave_power_report(uint8_t minute, uint8_t offset, uint8_t amount, uint16_t *data, uint32_t energy);
bool izican_slave_monitor_report(uint8_t *values, uint8_t length);
bool izican_slave_notifymonitor(uint16_t index, izi_module_var_t *module);
bool izican_slave_reportmonitor(uint16_t index, izi_module_t *module);
bool izican_slave_reportlog(logitem_t *logitem);
bool izican_slave_notifylog(logitem_t *logitem);
bool izican_master_present(void);
uint16_t izican_slave_get_linkstate();
int16_t izican_slave_update_state(bool report, bool force);
bool izican_slave_update_monitor(uint8_t *values, uint8_t length, bool force);

extern uint8_t can_slave_id;
extern uint16_t can_slave_state;

#endif /* IZICAN_SLAVE_HANDLER_H_ */
