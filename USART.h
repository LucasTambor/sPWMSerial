#ifndef USART_H
#define USART_H

#include <inttypes.h>
#include <avr/io.h>

#define F_OSC	8000000

extern void USART_init(uint32_t baud);

extern unsigned char USART_Get(void);
 
extern void USART_Send(unsigned char data);

extern uint8_t USART_Available(void);

#endif
