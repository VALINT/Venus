/*
 * _1905_Prj_VENUS.c
 *
 * Created: 25.05.2019 15:24:46
 *  Author: VAL
 */ 
#define F_CPU 8000000UL

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <math.h>

#define STALL_TIME		75
#define BAR_MOVE_DELAY	6
#define MOVE_TIME		12

#define h_bridge_PIN1	5
#define h_bridge_PIN2	6
#define h_bridge_PORT	PORTD	

#define data1_PORT		PORTD
#define data1_PIN		2
#define data2_PORT		PORTD
#define data2_PIN		0
#define clk_PORT		PORTD
#define clk_PIN			1

#define net1_PORT		PORTD
#define net1_PIN		4
#define net2_PORT		PORTD
#define net2_PIN		3

#define h_bridge_1pos	h_bridge_PORT &=~ (1 << h_bridge_PIN2); asm("nop"); h_bridge_PORT |= (1 << h_bridge_PIN1)
#define h_bridge_2pos	h_bridge_PORT &=~ (1 << h_bridge_PIN1); asm("nop"); h_bridge_PORT |= (1 << h_bridge_PIN2)
#define clk				clk_PORT &=~ (1 << clk_PIN); clk_PORT |= (1 << clk_PIN);
#define set_data_1_h	data1_PORT |= (1 << data1_PIN)
#define set_data_2_h	data2_PORT |= (1 << data2_PIN)
#define set_data_1_l	data1_PORT &=~ (1 << data1_PIN)
#define set_data_2_l	data2_PORT &=~ (1 << data2_PIN)
#define show_net1		net2_PORT &=~ (1 << net2_PIN); asm("nop"); net1_PORT |= (1 << net1_PIN)
#define show_net2		net1_PORT &=~ (1 << net1_PIN); asm("nop"); net2_PORT |= (1 << net2_PIN)
#define clear_screen	net1_PORT &=~ ((1 << net1_PIN)|(1 << net2_PIN))

const uint16_t LogDivArr[16] PROGMEM = {1000, 100, 20, 10, 625, 455, 357, 316, 282, 250, 224, 200, 179, 159, 140, 99};
const uint16_t LogMulArr[16] PROGMEM = {1,  1,  1,  1, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};	
	
const uint16_t LinDivArr[16] PROGMEM = {16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16};
const uint16_t LinMulArr[16] PROGMEM = {1,   2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16};
	
uint16_t LogScale[16]  = {5, 10, 50, 100, 160, 220, 280, 320, 360, 400, 450, 510, 580, 640, 730, 1000};
uint16_t LineScale[16]  = {35, 70, 140, 210, 280, 350, 420, 490, 560, 630, 700, 770, 840, 910, 980, 1020};

typedef struct BAR 
{
	uint16_t raw;
	uint16_t peak;
	uint16_t last_peak;
	uint16_t bar;
	uint16_t last_bar;
	uint8_t  bar_fall_counter;
	uint8_t  peak_stall_counter;
	uint8_t  peak_fall_counter;
} BAR;

BAR left_channel = {0,0,0,0,0,0,0};
BAR right_channel = {0,0,0,0,0,0,0};

uint8_t frame_even_odd = 0;
uint16_t calibration_value = 0;
uint16_t last_cal_val = 0;

void _fill_buffer(uint16_t);
void S_BuildBar(BAR *data);
void S_Amortization(BAR *data);

ISR (TIMER0_OVF_vect)
{
	if(!frame_even_odd)
	{
		h_bridge_1pos;
		frame_even_odd++;
	}	
	else
	{
		h_bridge_2pos;
		frame_even_odd = 0;
	}		
}

int main(void)
{
	uint16_t buff_d = 0,buff_m = 0;
	
//Initialization:
	// I/O ports initialization
	DDRD = 0xff;
	DDRB = 0x00;
	PORTB = 0b00000011;
	DDRC = 0b00000000; 		
	// Timers initialization						
	TCCR0 = (1<<CS02);					
	TIMSK = (1<<TOIE0);
	// Scale initialization
	// Need to initialized ADC, measure reference value and build scale.
	ADMUX = 0x42;
	ADCSRA = 0x83;
	ADCSRA |= (1 << ADSC);
	while(!(ADCSRA & (1 <<ADIF)));
	
	// calibration	
	calibration_value = ADCW;
	if(calibration_value < 100) calibration_value = 100;
	last_cal_val = calibration_value;
	
	// build linear scale 	
	for(uint8_t i = 0; i < 15; i++)
	{
		buff_d = pgm_read_word(&(LinDivArr[i]));
		buff_m = pgm_read_word(&(LinMulArr[i]));
		LineScale[i] = ((calibration_value*buff_m)/buff_d);
	}
	
	// build logarithmic scale
	for(uint8_t i = 0; i < 15; i++)
	{
		buff_d = pgm_read_word(&(LogDivArr[i]));
		buff_m = pgm_read_word(&(LogMulArr[i]));
		LogScale[i] = ((calibration_value*buff_m)/buff_d);
	}						
	sei();
	
    while(1)
    {
			ADMUX = 0x42;
			ADCSRA |= (1 << ADSC);
			while(!(ADCSRA & (1 <<ADIF))){};
		
			calibration_value = ADCW;
			if(calibration_value < 100)
				calibration_value = 100;
				
			if(abs(last_cal_val - calibration_value) > 10);
			{
				last_cal_val = calibration_value;
				//build line scale
				for(uint8_t i = 0; i < 15; i++)
				{
					buff_d = pgm_read_word(&(LinDivArr[i]));
					buff_m = pgm_read_word(&(LinMulArr[i]));
					LineScale[i] = (uint32_t)((calibration_value*buff_m)/buff_d);
				}
				//build log scale
				for(uint8_t i = 0; i < 15; i++)
				{
					buff_d = pgm_read_word(&(LogDivArr[i]));
					buff_m = pgm_read_word(&(LogMulArr[i]));
					LogScale[i] = (uint32_t)((calibration_value*buff_m)/buff_d);
				}
			}

		clear_screen;
		S_BuildBar(&right_channel);
		S_Amortization(&right_channel);
		_fill_buffer(right_channel.bar);
		
		//Show right bar
		show_net1;
		
		//Waite to next screen/measure left channel bar
		right_channel.raw = 0;
		
		ADMUX = 0x41;
		ADCSRA |= (1 << ADSC);
		while(!(ADCSRA & (1 <<ADIF)));
		
		for(uint8_t i = 0; i < 99; i++)
		{
			ADMUX = 0x41;	
			ADCSRA |= (1 << ADSC);
			while(!(ADCSRA & (1 <<ADIF)));
			if(left_channel.raw < ADCW)
				left_channel.raw = ADCW;
			ADMUX = 0x40;
			ADCSRA |= (1 << ADSC);
			while(!(ADCSRA & (1 <<ADIF)));
			if(right_channel.raw < ADCW)
				right_channel.raw = ADCW;
		}
		
		clear_screen;
		
		//Prepare left channel bar
		S_BuildBar(&left_channel);
		S_Amortization(&left_channel);
		
		//Write bar in shift register		
		_fill_buffer(left_channel.bar);

		//Show left bar
		show_net2;
		
		//Waite to next screen/measure right channel bar
		left_channel.raw = 0;
		
		ADMUX = 0x40;
		ADCSRA |= (1 << ADSC);
		while(!(ADCSRA & (1 <<ADIF)));
		
		for(uint8_t i = 0; i < 100; i++)
		{
			ADMUX = 0x40;
			ADCSRA |= (1 << ADSC);
			while(!(ADCSRA & (1 <<ADIF)));
			if(right_channel.raw < ADCW)
			right_channel.raw = ADCW;
			ADMUX = 0x41;
			ADCSRA |= (1 << ADSC);
			while(!(ADCSRA & (1 <<ADIF)));
			if(left_channel.raw < ADCW)
			left_channel.raw = ADCW;
		}
	}
}

void _fill_buffer(uint16_t bar)
{
	uint8_t first_part = bar;
	uint8_t second_part = bar >> 8;
	
	for(uint8_t i = 0; i < 8; i++)
	{
		if(first_part & (1 << i))
			set_data_1_h;
		else
			set_data_1_l;
		if(second_part &(1 << i))
			set_data_2_h;
		else
			set_data_2_l;
		clk;
	}
}

void S_BuildBar(BAR *data)														// This function build bar used raw data from ADC 
{
	data->bar = 0;
	data->peak = 0;
	for(uint8_t i = 0; i <= 16; i++)
	{
		if(PINB & (1 << 0))														//if jumper 1 set use linear scale
		{
			if(data->raw >= LogScale[i])										//if jumper 1 absent use logarithmic scale 
			{																	//P.S. (if jumper absent, on PB0 high state)
				data->bar = (data->bar << 1)+1;
				if(!(PINB & (1 << 1)))											//if jumper 2 set draw peak point
					{
						if(data->peak == 0) {data->peak = 1;}
						else {data->peak = (data->peak << 1);}
					}
				else	
					data->last_peak = 0;					
			}
		}	
		else																	
			{
				if(data->raw >= LineScale[i]) 
				{
					data->bar = (data->bar << 1)+1;
					if(!(PINB & (1 << 1)))											//if jumper 2 set draw peak point
					{
						if(data->peak == 0) {data->peak = 1;}
						else {data->peak = (data->peak << 1);}
					}
				}
			}
	}
}

void S_Amortization(BAR *data)
{
	if(data->bar < data->last_bar)
	{
		if(data->bar_fall_counter >= BAR_MOVE_DELAY)
		{
			data->bar = data->last_bar >> 1;
			data->bar_fall_counter = 0;
		}
		else
		{
			data->bar = data->last_bar;
			data->bar_fall_counter++;
		}		
	}	
	else
	{
		data->bar_fall_counter = 0;
	}
	
	if(!(PINB & (1 << 1)))
	{
		if(data->peak < data->last_peak)
		{
			if(data->peak_stall_counter >= STALL_TIME)
			{
				if(data->peak_fall_counter >= MOVE_TIME)
				{
					data->last_peak = data->last_peak >> 1;
					data->peak_fall_counter = 0;
				}
				else
				{
					data->peak_fall_counter++;
				}
			}
			else
			{
				if(data->peak_stall_counter < STALL_TIME)
					data->peak_stall_counter++;
			}
			
		}
		else
		{
			data->last_peak = data->peak;
			data->peak_fall_counter = 0;
			data->peak_stall_counter = 0;
		}
	}	
	data->last_bar = data->bar;
	data->bar |= data->last_peak;
}
