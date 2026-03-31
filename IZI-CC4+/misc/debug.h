/*
 * debug.h
 *
 *  Created on: 18 mei 2020
 *      Author: Milo
 */

#ifndef DEBUG_H_
#define DEBUG_H_

void Debug_Init();
void Debug_Putc(uint8_t c);
void Debug_Recv(uint8_t c);
void Debug_Off(bool off);

#endif /* DEBUG_H_ */
