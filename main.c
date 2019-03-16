/*
 * 	
 *  Author: Lucas Tamborrino
 *  2019
 */ 


#include <avr/io.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include "lcd_usb.h"
#include "defines.h"
#include <math.h>
#include "USART.h"

//************************************************************************

#define F_OSC	8000000
#define PWM_F_OSC	10000
#define MAX_ADC_VALUE	1023

//************************************************************************


volatile uint16_t Lut_Size = 166;
uint16_t Freq_Osc_Sin = 60;

uint16_t adcValue = 0;
double adcNorm = 0;

uint16_t ICR1_value = 0;
uint16_t LUT_size = 0;

volatile uint8_t idx = 0;
double lookUpTable[255];


int16_t OCR1C_Table[255];

volatile uint16_t timerCounter = 0;


//************************************************************************

uint16_t calculate_sin_values()
{	
	double sinVal = 0;
	// Calcula valor ICR1
	ICR1_value = (uint16_t) F_OSC/PWM_F_OSC;


	for(uint16_t i = 0; i<Lut_Size; i++)
	{
		sinVal = sin(i*2*M_PI/Lut_Size);
		lookUpTable[i] = sinVal;
		
	}

	return ICR1_value;
	
}

void setFreq(uint8_t freq)
{
	cli();

	double sinVal = 0;

	Lut_Size = PWM_F_OSC / freq;
	
	for(uint16_t i = 0; i<Lut_Size; i++)
	{
		sinVal = sin(i*2*M_PI/Lut_Size);
		lookUpTable[i] = sinVal;
		
	}

	sei();
}


void Adc_init(void){

    //ADC Multiplexer Selection Register = ADMUX
	ADMUX = (1<<MUX0)| (1<<REFS0);  // Set reference to AVcc,set chanel 1

	// ADCSRA - ADC Control and Status Register A
	ADCSRA = (1<<ADEN);  // Enable ADC
	ADCSRA = (1<<ADATE)|(1<<ADEN)|(1<<ADIF);  // Auto Trigger Enable  (default auto-trigger is free-running mode ADCSRB)
	ADCSRA = (1<<ADPS2)|(1<<ADPS1)|(1<<ADEN)|(1<<ADSC)|(1<<ADIE);
	// Disable digital input buffer on the ADC pin (reduces power consumption)
	DIDR0=(1<<5)|(1<<4)|(1<<1)|(1<<0);
}

//Analog Read ISR
ISR(ADC_vect)
{
	adcValue = ADC;

	//new reading 
	ADCSRA|= (1<<ADSC);

}



void Timer0_init(void) // Aciona contador externo
{

	TCNT0 = 0;
	TCCR0A =0;
	TCCR0B = (1<< CS02)| (0<< CS01)|(0<< CS00); /// subida em T0 === D2 === PD7
	TIMSK0 = (1<<TOIE0);
}



void Timer1_init(void)
{

	//TCNT1 = 217;
	TCCR1A = 0;
	TCCR1B =  (1<< CS11)|(1<< CS10);
	TIMSK1 = (1<<TOIE1);
}

// Interrupção Timer 1
ISR(TIMER1_OVF_vect)
{

	//Atualiza valor de OCR1C
	
	OCR1C = OCR1C_Table[idx] ;

	idx++;

	if(idx == Lut_Size)
	{
	 	idx = 0;
	}
	
}

// Interrupção Timer 0
ISR(TIMER0_OVF_vect)
{
	
	timerCounter++;

}

void init_pwm1C(void)
{
	ICR1 = 0x0190;
   TCCR1A |= (1<<COM1C1);
   TCCR1B = (1<< WGM13)|(0<<CS11)|(1<< CS10); //NO preescaler
   TCCR1C = 0;
   TIMSK1 = (1<<TOIE1);
}

void initIO(void)
{
	MCUCR |= (1<<JTD);
	MCUCR |= (1<<JTD);
    MCUCR |= (1<<JTD); // Desabilita JTAG wr 2x
	// Inicializa portas de IO do processador
	DDRB = (1<<7)|(1<<6)|(1<<5)|(1<<4)|(1<<3)|(1<<2)|(1<<1); // Port B Bits 7 ao 1 como sa�da
	DDRC = (1<<6)|(1<<7); // Port C Bits 7 e 6 como saida
	DDRD = (1<<4)|(1<<6)|(1<<7); // Port D Bits 4, 6 e 7 como saida
	DDRE = 0; // Port E entrada
	DDRF = 0; // Port F entrada
	
	PORTB = 0; 
	PORTC = 0;
	PORTD = (1<<5)|(1<<1)|(1<<0); // Liga pullup
	PORTE = (1<<2); // Liga pullup
	PORTF = (1<<7)|(1<<6); // Liga pullup

}







void updateOCR(void)
{
	OCR1C_Table[idx] = (int16_t)((lookUpTable[idx]*200*adcValue)/MAX_ADC_VALUE) + 200;

}


void serialListener(void)
{
	uint8_t inputBuffer = 0;
	char buffer[10];
	

	if(USART_Available())
	{
		inputBuffer = USART_Get();
		
		if(inputBuffer != 'A' || inputBuffer != 'B')
		{
			lcd_gotoxy(0, 1);
			sprintf(buffer, "Atualizando... ");
			lcd_puts(buffer);
			
		}

		switch(inputBuffer)
		{
			case 'A':
				Liga_Feed();
			break;
			case 'B':
				Desliga_Feed();
			break;
		
			case 50:
				setFreq(50);
			break;
			case 60:
				setFreq(60);
			break;
			case 80:
				setFreq(80);
			break;
			default:
				setFreq(inputBuffer);
			break;

		
		}
		lcd_clrscr();
		lcd_gotoxy(0, 0);
		lcd_puts_p(PSTR("sPWM Generator"));
		sei();
		
	}
	
}

void btnListener(void)
{
	if(Ch_busy_lig)
	{
		USART_Send(65);
	}

	if(Ch_ack_lig)
	{
		USART_Send(66);

	}
	

		
	if(timerCounter >= 120)
	{
		uint8_t adcVolt = (uint8_t) (adcValue * 5 /1023);
		
		Toggle_Init();
		timerCounter = 0;
		
				

		USART_Send(adcVolt);
	}
	

}



int main(void)
{
	
	initIO();

	Timer0_init();
	
	Adc_init();

	//init LCD
	lcd_init(LCD_DISP_ON);
	lcd_clrscr();
	lcd_puts_p(PSTR("sPWM Generator"));
	lcd_gotoxy(0, 1);
	lcd_puts_p(PSTR("ADC: "));
	

	init_pwm1C();  // inicia pwm

	setFreq(60);  // define valor


	USART_init(9600);
		


	sei(); // interrupcao geral
    
	
	
    while(1)
    {
		updateOCR();

		//updateLCD(); 

		serialListener();

		btnListener();
    }
}

