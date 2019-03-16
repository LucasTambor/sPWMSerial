#include "USART.h"


void USART_init(uint32_t baud)
{
    UBRR1 = (F_OSC/(16*baud)) - 1;                    // define baud rate e habilita a USART
    UCSR1B = (1<<TXEN1) | (1<<RXEN1);
}
 
 
unsigned char USART_Get(void)
{
    while( !(UCSR1A & (1<<RXC1)) ); //aguarda receber um  byte
    return UDR1;                    // retorna byte recebido
}
 
 
void USART_Send( unsigned char data )
{
    while( !(UCSR1A & (1<<UDRE1)) ); // aguarda buffer de transmissão vazio
    UDR1 = data;                     // envia o byte
}
 
uint8_t USART_Available(void)
{
	return (UCSR1A & (1<<RXC1));
}
