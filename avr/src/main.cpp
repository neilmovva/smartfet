#include <Arduino.h>

const uint32_t PERIOD_VALUE  =  (256 - 1);
#define DRV_INVERTED

#define PORT_LED    PORTC
#define PIN_LED_R   0

#define PORT_PWM_CH1    PORTB
#define PORT_PWM_CH2    PORTB
#define PIN_PWM_CH1     1
#define PIN_PWM_CH2     2

#define PORT_IO_CH3    PORTD
#define PIN_IO_CH3     5
#define NUM_CHANNELS 	3


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
	uint32_t pwm_level = myAtoi(str_pwm_level);
	uint32_t pwm_channel = myAtoi(str_pwm_channel);
	bool command_is_sane = true;

	if(pwm_level > PERIOD_VALUE) {
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
	const int bufsz = 64;
	char receipt[bufsz];
	mini_snprintf(receipt, bufsz, "CH: %u  PWR: %u", 
		pwm_channel, pwm_level);
	Serial.println(receipt);
}


void sssp_receive_loop() {
	char buffer_rx[sizeof(sssp_packet_pwm_t) + 1];
	buffer_rx[sizeof(sssp_packet_pwm_t)] = '\0';
	uint32_t buffer_idx = 0;

	uint32_t state_current = 0;
	while(1) {
		char input_char = usart_recv_blocking(USART1);
		
		bool should_accept_char = false;
		bool should_reset_state = false;

		if(char_is_alphanumeric(input_char)) {
			//echo char
			usart_send_blocking(USART1, input_char);

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
			usart_newln();
			should_reset_state = true;
		}

		if(should_accept_char) {
			gpio_set(PORT_LED, PIN_LED);
			buffer_rx[buffer_idx] = input_char;
			buffer_idx++;
			//once buffer is full, ship pkt downstream
			if(buffer_idx >= sizeof(sssp_packet_pwm_t)) {
				sssp_packet_pwm_t pkt[1];
				memcpy(pkt, buffer_rx, sizeof(sssp_packet_pwm_t));
				usart_newln();
				sssp_process_packet_pwm(pkt);
				should_reset_state = true;
			}
		}

		if(should_reset_state) {
			gpio_clear(PORT_LED, PIN_LED);
			//reset state and buffer
			state_current = 0;
			buffer_idx = 0;
			memset(buffer_rx, 0, sizeof(sssp_packet_pwm_t));
		}


	}
}

int main(void) {
    rcc_clock_setup_in_hsi_out_48mhz();
    gpio_setup();
	pwm_setup();
	usart_setup();

    for(int iter_blink = 0; iter_blink < 8; iter_blink++) {
		gpio_toggle(PORT_LED, PIN_LED);
		busywait_ms(25);
    }
	
	sssp_receive_loop();

	return 0;
}



void setup() {
    // put your setup code here, to run once:
}

void loop() {
    // put your main code here, to run repeatedly:
}