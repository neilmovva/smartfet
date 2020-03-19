// Smartfet Firmware, written for ATmega328P.
// nmovva 2018

#include <Arduino.h>
#include "mini-printf.h"
#include "sssp.h"

#include "sinetable.h"
#define SINETABLE sin_a255_p4096

//helper macros
#define SET(x,y) (x |= (1<<y))
#define FLP(x,y) (x ^= (1<<y))
#define CLR(x,y) (x &= (~(1<<y)))

inline int MAX(int a, int b) {
    if (a > b)
        return a;
    return b;
}

// #define DRV_INVERTED

const uint16_t PWM_PERIOD_CYCLES 	=  200;
const uint16_t PWM_VALUE_MAX 		=  PWM_PERIOD_CYCLES;
const uint8_t  SSSP_MAX_PWR_LEVEL  	=  100;
const uint8_t  PWR_TO_PWM_MULT 		=  2;

#ifdef DRV_INVERTED
const uint16_t  REST_PWMLEVEL		=  PWM_VALUE_MAX;
#else 
const uint8_t REST_PWMLEVEL =  0;
#endif

// #define REVB

#ifdef REVB
#define DDR_LED    	DDRD
#define PORT_LED    PORTD
#define PIN_LED_R   4
#else
#define DDR_LED    	DDRC
#define PORT_LED    PORTC
#define PIN_LED_R   0
#endif

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


// HAL function, verify against hardware!
void pwm_ch_disable(uint8_t channel) {
	switch(channel) {
		case 1:
		CLR(TCCR1A, COM1A1);
		OCR1A = 0;
		break;

		case 2:
		CLR(TCCR1A, COM1B1);
		OCR1B = 0;
		break;

		case 3:
		CLR(TCCR0A, COM0A1);
		OCR0A = 0;
		break;
	}
}

// HAL function, verify against hardware!
void pwm_ch_enable(uint8_t channel) {
	switch(channel) {
		case 1:
		SET(TCCR1A, COM1A1);
		OCR1A = 0;
		break;

		case 2:
		SET(TCCR1A, COM1B1);
		OCR1B = 0;
		break;

		case 3:
		SET(TCCR0A, COM0A1);
		OCR0A = 0;
		break;
	}
}

void pwm_update_raw8(uint8_t pwm_level, uint8_t channel) {
	#ifdef DRV_INVERTED
	pwm_level = PWM_VALUE_MAX - pwm_level;
	#endif

	if(pwm_level == 0) {
		pwm_ch_disable(channel);
		return;
	} 
	pwm_ch_enable(channel);

	switch(channel) {
		case 1:
		OCR1A = pwm_level;
		break;

		case 2:
		OCR1B = pwm_level;
		break;

		case 3:
		OCR0A = pwm_level;
		break;
	}
}

void pwm_update_ch(uint8_t powerlevel, uint8_t channel) {
	uint16_t pwm_level = powerlevel * PWR_TO_PWM_MULT;
	if(pwm_level > PWM_VALUE_MAX) {
		pwm_level = PWM_VALUE_MAX;
	}
	pwm_update_raw8(pwm_level, channel);
}

void sssp_process_packet_pwm(sssp_packet_pwm_t* pkt) {
	//unpack hdr and ftr strings
	char str_hdr[sizeof(pkt->hdr) + 1] = {};
	char str_ftr[sizeof(pkt->ftr) + 1] = {};
	strncat(str_hdr, pkt->hdr, sizeof(pkt->hdr));
	strncat(str_ftr, pkt->ftr, sizeof(pkt->ftr));
	//check against protocol-defined magic data
	int cmp_hdr = strncmp(str_hdr, BOOKEND_HDR, sizeof(pkt->hdr));
	int cmp_ftr = strncmp(str_ftr, BOOKEND_FTR, sizeof(pkt->ftr));
	if(cmp_hdr != 0 || cmp_ftr != 0){
		//print error
		const int bufsz = 64;
		char receipt[bufsz];
		mini_snprintf(receipt, bufsz, 
			"invalid -- hdr: %s  ftr: %s  BUF: %s", 
			str_hdr, str_ftr, pkt);
		Serial.println(receipt);
		return; //drop packet
	}

	//unpack payload strings
	char str_powerlevel[sizeof(pkt->powerlevel) + 1] = {}; 
	char str_channel[sizeof(pkt->channel) + 1] = {}; 
	strncat(str_powerlevel, pkt->powerlevel, 
		sizeof(pkt->powerlevel));
	strncat(str_channel, pkt->channel, 
		sizeof(pkt->channel));
	//parse text data
	uint8_t powerlevel = parse_powerlevel(str_powerlevel);
	uint8_t channel = parse_channel(str_channel);
	bool command_is_sane = true;

	if(powerlevel > SSSP_MAX_PWR_LEVEL) {
 		Serial.println("error   -- PWM value set too high, ignoring");
		command_is_sane = false;
	} 
	if (channel > NUM_CHANNELS) {
		Serial.println("error   -- target channel not found, ignoring");
		command_is_sane = false;
	}

	//if a valid command, commit payload data to HW
	if(command_is_sane) {
		pwm_update_ch(powerlevel, channel);
		Serial.println("committed");
		Serial.print("    "); 	//indent sucessful commands
	}
	
	//send acknowledgement string
	const uint8_t bufsz = 64;
	char receipt[bufsz];
	mini_snprintf(receipt, bufsz, "CH: %u  PWR: %u", 
		channel, powerlevel);
	Serial.println(receipt);
}


void sssp_receive_loop() {
	char buffer_rx[sizeof(sssp_packet_pwm_t) + 1];
	buffer_rx[sizeof(sssp_packet_pwm_t)] = '\0';
	uint8_t buffer_idx = 0;

	while(buffer_idx < sizeof(sssp_packet_pwm_t)) {
		//blocking busywait for serial data
		while(Serial.available() < 1) {};
		
		char input_char = Serial.read();

		if(char_is_alphanumeric(input_char)) {
			//echo char
			Serial.print(input_char);
			//append to buffer
			buffer_rx[buffer_idx] = input_char;
			buffer_idx++;

		} else if(input_char == ENTER_CHAR) {
			//visually end user input with NL+CR
			Serial.println();
			//return to reset input buffer, align to cmd start
			return;
		}
	}
	//once buffer is full, ship pkt downstream
	sssp_packet_pwm_t pkt[1];
	memcpy(pkt, buffer_rx, sizeof(sssp_packet_pwm_t));
	Serial.println();
	sssp_process_packet_pwm(pkt);
}


void phase_test_loop() {
	pwm_update_ch(25, 3);
	//phase1
	pwm_update_ch(10, 1);
	pwm_update_ch(60, 2);
	//endphase
	_delay_ms(10000);

}


void triangle() {

	for(uint16_t step = 0; step < PWM_VALUE_MAX; step++) {
		uint8_t pwr_a = step;
		uint8_t pwr_b = (step + 127) % PWM_VALUE_MAX;

		pwm_update_raw8(pwr_a, 1);
		pwm_update_raw8(pwr_b, 2);
		_delay_ms(20);
	}

	for(uint16_t step = PWM_VALUE_MAX; step > 0 ; step--) {
		uint8_t pwr_a = step;
		uint8_t pwr_b = (step - 127) % PWM_VALUE_MAX;

		pwm_update_raw8(pwr_a, 1);
		pwm_update_raw8(pwr_b, 2);
		_delay_ms(20);
	}

	// turn off both channels
	pwm_update_ch(0, 1);
	pwm_update_ch(0, 2);
}

void sinewave_allphase() {
	FLP(PORT_LED, PIN_LED_R);

	const uint16_t table_pts = sizeof(SINETABLE);
	const uint16_t phase_stride = table_pts / NUM_CHANNELS;

	for(uint16_t theta = 0; theta < table_pts; theta++) {
		for(uint8_t idx_ch = 0; idx_ch < NUM_CHANNELS; idx_ch++) {
			uint16_t table_idx = (theta + idx_ch * phase_stride) % table_pts;
			uint8_t pwr_ch =  MAX(pgm_read_byte(&SINETABLE[table_idx]), 5);
			pwm_update_raw8(pwr_ch, idx_ch + 1);
		}
		_delay_ms(1);
	}

	// turn off all channels
	pwm_update_ch(0, 1);
	pwm_update_ch(0, 2);
	pwm_update_ch(0, 3);
}


void setup() {
	// assert signal pins immediately
	SET(DDR_PWM, PIN_PWM_CH1);
	SET(DDR_PWM, PIN_PWM_CH2);
	SET(DDR_PWM_CH3, PIN_PWM_CH3);

	#ifdef DRV_INVERTED
	SET(PORT_PWM_CH1, PIN_PWM_CH1);
	SET(PORT_PWM_CH2, PIN_PWM_CH2);
	SET(PORT_IO_CH3, PIN_IO_CH3);
	#else 
	CLR(PORT_PWM_CH1, PIN_PWM_CH1);
	CLR(PORT_PWM_CH2, PIN_PWM_CH2);
	CLR(PORT_PWM_CH3, PIN_PWM_CH3);
	#endif

	// flash initialization pattern
	SET(DDR_LED, PIN_LED_R);
	const uint8_t blink_seconds = 2;
    for(uint8_t iter_blink = 0; iter_blink < (blink_seconds * 5); iter_blink++) {
		SET(PORT_LED, PIN_LED_R);
		_delay_ms(100);
		CLR(PORT_LED, PIN_LED_R);
		_delay_ms(100);
    }

	Serial.begin(BAUDRATE);
	Serial.println("hello world");

	// Timer1 setup (CH A and B)
	// WGM 14, FastPWM, TOP=ICR1, 1x prescaler
	TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);     
	TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);
	ICR1 = PWM_PERIOD_CYCLES;
	TCNT1  = 0x0000;

	// Timer0 setup (CH C)
	// WGM 03, FastPWM, TOP=255, 1x prescaler, out OC0A
	TCCR0A = _BV(COM0A1) | _BV(WGM00) | _BV(WGM01);     
	TCCR0B = _BV(CS00);
	OCR0A = 0;
	TCNT0  = 0x00;

	// turn off all PWM channels
	pwm_update_ch(0, 1);
	pwm_update_ch(0, 2);
	pwm_update_ch(0, 3);
	
}


void loop() {

	// sssp_receive_loop();
	// pwm_ch_enable(1);
	// pwm_update_ch(100, 1);
	phase_test_loop();
	// sinewave_allphase();
	// triangle();
}
