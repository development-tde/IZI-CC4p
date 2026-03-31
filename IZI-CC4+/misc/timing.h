/*
 * timing.h
 *
 * Created: 15-2-2025 16:25:41
 *  Author: Milo
 */ 


#ifndef TIMING_H_
#define TIMING_H_

void timing_init(void);
uint16_t timing_get_count(void);
void timing_set_callback0(uint16_t us, void (*callback)(void));
void timing_set_callback1(uint16_t us, void (*callback)(void));


#endif /* TIMING_H_ */