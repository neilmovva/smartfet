#include <Arduino.h>
#include <TimerOne.h>
#include "mini-printf.h"

#define DRV_INVERTED

const uint16_t PWM_PERIOD 	  		=  100;
const uint8_t  SSSP_MAX_PWR_LEVEL  	=  100;
#ifdef DRV_INVERTED
const uint16_t  REST_PWM_LEVEL 		=  1023;
#else 
const uint8_t REST_PWM_LEVEL =  0;
#endif

#define DDR_LED    	DDRC
#define PORT_LED    PORTC
#define PIN_LED_R   0

#define NUM_CHANNELS 	3

#define PORT_PWM_CH1    PORTB
#define PORT_PWM_CH2    PORTB
#define PIN_PWM_CH1     1
#define PIN_PWM_CH2     2
#define ARDPIN_PWM_CH1  9
#define ARDPIN_PWM_CH2  10

#define DDR_IO_CH3    	DDRD
#define PORT_IO_CH3    	PORTD
#define PIN_IO_CH3     	5


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

void pwm_update_ch(uint8_t power_level, uint8_t pwm_channel) {
	uint16_t pwm_level = 10 * power_level;
	#ifdef DRV_INVERTED
	pwm_level = 1023 - pwm_level;
	#endif

	switch(pwm_channel) {
		case 1:
		Timer1.setPwmDuty(ARDPIN_PWM_CH1, pwm_level);
		break;

		case 2:
		Timer1.setPwmDuty(ARDPIN_PWM_CH2, pwm_level);
		break;

		case 3:
		if(power_level > 50) {
			CLR(PORT_IO_CH3, PIN_IO_CH3);
		} else {
			SET(PORT_IO_CH3, PIN_IO_CH3);
		}
		break;
	}
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

	if(pwm_level > SSSP_MAX_PWR_LEVEL) {
 		Serial.println("error   -- PWM value set too high, ignoring");
		command_is_sane = false;
	} 
	if (pwm_channel > NUM_CHANNELS) {
		Serial.println("error   -- target channel not found, ignoring");
		command_is_sane = false;
	}

	//if a valid command, commit payload data to HW
	if(command_is_sane) {
		pwm_update_ch(pwm_level, pwm_channel);
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
			
		}
	}
	//once buffer is full, ship pkt downstream
	sssp_packet_pwm_t pkt[1];
	memcpy(pkt, buffer_rx, sizeof(sssp_packet_pwm_t));
	Serial.println();
	sssp_process_packet_pwm(pkt);
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

	Timer1.initialize(PWM_PERIOD);
	Timer1.pwm(ARDPIN_PWM_CH1, REST_PWM_LEVEL); 
	Timer1.pwm(ARDPIN_PWM_CH2, REST_PWM_LEVEL);

	SET(DDR_IO_CH3, PIN_IO_CH3);
	#ifdef DRV_INVERTED
	SET(PORT_IO_CH3, PIN_IO_CH3);
	#else 
	CLR(PORT_IO_CH3, PIN_IO_CH3);
	#endif
	

}

void loop() {
	sssp_receive_loop();
}
