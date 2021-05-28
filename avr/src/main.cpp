// Smartfet Firmware, written for ATmega328P.
// nmovva 2018

extern "C" {
	#include <avr/sleep.h>
	#include <avr/interrupt.h>
	#include <util/delay.h>
	#include "hal_pwm.h"
}

#define DDR_LED    	DDRC
#define PORT_LED    PORTC
#define PIN_LED_R   0

#include <Arduino.h>
#include "sssp.h"
#include "mini-printf.h"

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

	if(powerlevel > HAL_MAX_PWR_LEVEL) {
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


int main() {
	
	pwm_init();

	// flash initialization pattern
	SET(DDR_LED, PIN_LED_R);
	const uint8_t blink_seconds = 2;
    for(uint8_t iter_blink = 0; iter_blink < (blink_seconds * 10); iter_blink++) {
		SET(PORT_LED, PIN_LED_R);
		_delay_ms(10);
		CLR(PORT_LED, PIN_LED_R);
		_delay_ms(90);
    }


	pwm_ch_enable(3);
	pwm_update_ch(50, 3);

	pwm_ch_enable(1);
	pwm_ch_enable(2);
	pwm_update_ch(100, 1);
	pwm_update_ch(100, 2);
	

	set_sleep_mode(SLEEP_MODE_IDLE);
	while (1) {
		cli();
		sleep_enable();
		sei();
		sleep_cpu();			//go to sleep
		sleep_disable();	//resume execution after ISR
	}
}