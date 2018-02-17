#include <Arduino.h>
#include "mini-printf.h"

const uint8_t MAX_PWM_LEVEL  =  100;
#define DRV_INVERTED

#define DDR_LED    	DDRC
#define PORT_LED    PORTC
#define PIN_LED_R   0

#define PORT_PWM_CH1    PORTB
#define PORT_PWM_CH2    PORTB
#define PIN_PWM_CH1     1
#define PIN_PWM_CH2     2

#define PORT_IO_CH3    	PORTD
#define PIN_IO_CH3     	5
#define NUM_CHANNELS 	3

//helper macros
#define SET(x,y) (x |= (1<<y))
#define CLR(x,y) (x &= (~(1<<y)))

//SSSP = Sail Simple Serial Protocol
struct sssp_packet_pwm {
	char hdr[4];
	char pwm_level[3];
	char pwm_channel[1];
	char ftr[4];
};
typedef sssp_packet_pwm sssp_packet_pwm_t; 

const char* BOOKEND_HDR 	=	"sail";
const char* BOOKEND_FTR  	=	"rsch";

#define BAUDRATE 				38400
#define ENTER_CHAR 				0x0D

bool char_is_alphanumeric(const char c) {
	bool is_num = (c >= 48U) && (c <= 57U);
	bool is_uppercase = (c >= 65U) && (c <= 90U);
	bool is_lowercase = (c >= 97U) && (c <= 122U);
	return (is_num || is_uppercase || is_lowercase);
}

uint8_t parse_pwm_level(const char* str_pwmlevel) {
    uint8_t d_hun = str_pwmlevel[0] - '0'; 
	uint8_t d_ten = str_pwmlevel[1] - '0'; 
	uint8_t d_one = str_pwmlevel[2] - '0';

	uint8_t result = d_hun*100 + d_ten*10 + d_one*1;
	return result;
}

uint8_t parse_pwm_ch(const char* str_pwmch) {
	uint8_t d_one = str_pwmch[0] - '0';

	uint8_t result = d_one*1;
	return result;
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
	char str_pwm_level[sizeof(pkt->pwm_level) + 1] = {}; 
	char str_pwm_channel[sizeof(pkt->pwm_channel) + 1] = {}; 
	strncat(str_pwm_level, pkt->pwm_level, 
		sizeof(pkt->pwm_level));
	strncat(str_pwm_channel, pkt->pwm_channel, 
		sizeof(pkt->pwm_channel));
	//parse text data
	uint8_t pwm_level = parse_pwm_level(str_pwm_level);
	uint8_t pwm_channel = parse_pwm_ch(str_pwm_channel);
	bool command_is_sane = true;

	if(pwm_level > MAX_PWM_LEVEL) {
 		Serial.println("error   -- PWM value set too high, ignoring");
		command_is_sane = false;
	} 
	if (pwm_channel > NUM_CHANNELS) {
		Serial.println("error   -- target channel not found, ignoring");
		command_is_sane = false;
	}

	//if a valid command, commit payload data to HW
	if(command_is_sane) {
		// TODO pwm_update_ch(pwm_level, pwm_channel);
		Serial.println("committed");
		Serial.print("    "); 	//indent sucessful commands
	}
	
	//send acknowledgement string
	const uint8_t bufsz = 64;
	char receipt[bufsz];
	mini_snprintf(receipt, bufsz, "CH: %u  PWR: %u", 
		pwm_channel, pwm_level);
	Serial.println(receipt);
}


void sssp_receive_loop() {
	char buffer_rx[sizeof(sssp_packet_pwm_t) + 1];
	buffer_rx[sizeof(sssp_packet_pwm_t)] = '\0';
	uint8_t buffer_idx = 0;

	uint8_t state_current = 0;
	while(1) {
		//blocking busywait for serial data
		while(Serial.available() < 1) {};
		
		char input_char = Serial.read();
		
		bool should_accept_char = false;
		bool should_reset_state = false;

		if(char_is_alphanumeric(input_char)) {
			//echo char
			Serial.print(input_char);

			//watch for header sequence
			if(state_current < strlen(BOOKEND_HDR)) {
				char input_advance_state = BOOKEND_HDR[state_current];
				if(input_char == input_advance_state) {
					state_current++;
					should_accept_char = true;
				} else {
					should_reset_state = true;
				}
			//after header, stream chars to end of buffer
			} else if(state_current == strlen(BOOKEND_HDR)) {
				//fill buffer
				if(buffer_idx < sizeof(sssp_packet_pwm_t)) {
					should_accept_char = true;
				}
			}

		} else if(input_char == ENTER_CHAR) {
			//visually end user input with NL+CR
			Serial.println();
			should_reset_state = true;
		}

		if(should_accept_char) {
			SET(PORT_LED, PIN_LED_R);
			buffer_rx[buffer_idx] = input_char;
			buffer_idx++;
			//once buffer is full, ship pkt downstream
			if(buffer_idx >= sizeof(sssp_packet_pwm_t)) {
				sssp_packet_pwm_t pkt[1];
				memcpy(pkt, buffer_rx, sizeof(sssp_packet_pwm_t));
				Serial.println();
				sssp_process_packet_pwm(pkt);
				should_reset_state = true;
			}
		}

		if(should_reset_state) {
			CLR(PORT_LED, PIN_LED_R);
			//reset state and buffer
			state_current = 0;
			buffer_idx = 0;
			memset(buffer_rx, 0, sizeof(sssp_packet_pwm_t));
		}


	}
}

void setup() {
	SET(DDR_LED, PIN_LED_R);

    for(int iter_blink = 0; iter_blink < (4 * 5); iter_blink++) {
		SET(PORT_LED, PIN_LED_R);
		_delay_ms(100);
		CLR(PORT_LED, PIN_LED_R);
		_delay_ms(100);
    }

	Serial.begin(BAUDRATE);
	Serial.println("hello world");
}

void loop() {
	sssp_receive_loop();
}
