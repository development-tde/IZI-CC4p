/*
 * USART4.h
 *
 * Created: 19-8-2021 09:35:59
 *  Author: Milo
 */ 


#ifndef USART4_H_
#define USART4_H_

void USART4_Init(void);
void USART4_Putc(char c);
bool USART4_Getc(char *c);
void USART4_Send();

#endif /* USART4_H_ */