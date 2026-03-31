/*
 * USART5.h
 *
 * Created: 19-8-2021 09:35:59
 *  Author: Milo
 */ 


#ifndef USART5_H_
#define USART5_H_

void USART1_Init(void);
void USART1_Putc(char c);
bool USART1_Getc(char *c);
void USART1_Send();

extern struct usart_async_descriptor USART_1;

#endif /* USART5_H_ */