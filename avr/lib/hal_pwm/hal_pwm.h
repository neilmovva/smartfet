#pragma once

#include <stdint.h>
#include <avr/io.h>

static const uint8_t  HAL_MAX_PWR_LEVEL	=  100;
static const uint8_t  PWR_TO_PWM_MULT 		=  2;
static const uint16_t PWM_PERIOD_CYCLES 	=  200;
#define NUM_CHANNELS 	3

// OC1A/B @ PB1/2 
#define DDR_PWM 		DDRB
#define PORT_PWM_CH1    PORTB
#define PORT_PWM_CH2    PORTB
#define PIN_PWM_CH1     1
#define PIN_PWM_CH2     2

// OC0A @ PD6 
#define DDR_PWM_CH3 	DDRD
#define PORT_PWM_CH3    PORTD
#define PIN_PWM_CH3     6	

#define BAUDRATE 				38400
#define ENTER_CHAR 				0x0D

// #define DRV_INVERTED

//helper macros
#define SET(x,y) (x |= (1<<y))
#define FLP(x,y) (x ^= (1<<y))
#define CLR(x,y) (x &= (~(1<<y)))

void pwm_init();
void pwm_update_ch(uint8_t powerlevel, uint8_t channel);
void pwm_ch_disable(uint8_t channel);
void pwm_ch_enable(uint8_t channel);
