#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>
#include <string.h>
#include <mini-printf.h>


const uint32_t PERIOD_VALUE  =  (256 - 1);
const uint32_t MAX_PWM_VALUE =  (PERIOD_VALUE * 0.75);
const uint32_t MIN_PWM_VALUE =  (0);
#define DRV_INVERTED

#define PORT_LED    GPIOB
#define PIN_LED     GPIO1

#define PORT_PWM    GPIOA
#define PIN_CH1     GPIO6
#define PIN_CH2     GPIO7
#define PIN_CH3     GPIO9
#define NUM_PWM_CHANNELS 	3

#define PORT_UART 	GPIOA
#define PIN_TX 		GPIO2
#define PIN_RX 		GPIO3

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

#define ENTER_CHAR 				0x0D


//implemented in console.cpp
void usart_send_str(const char* str);
void usart_print(const char* str);
void usart_newln();
bool char_is_alphanumeric(const char c);
int myAtoi(const char *str);



static void inline busywait_ms(int ms) {
	for (int i = 0; i < 6000 * ms; i++) {
		__asm__("nop");
	}
}

static void gpio_setup(void) {
	/* Enable GPIOB clock. */
	rcc_periph_clock_enable(RCC_GPIOB);
	gpio_mode_setup(PORT_LED, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN_LED);
}

static void usart_setup(void) {
	rcc_periph_clock_enable(RCC_USART1);
	rcc_periph_clock_enable(RCC_GPIOA);

	/* Setup USART1 parameters. for 8-N-1 at 115200 baud*/
	usart_set_baudrate(USART1, 115200);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, 1);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

	// USART1 GPIO setup
    gpio_mode_setup(PORT_UART, GPIO_MODE_AF, GPIO_PUPD_NONE, 
        PIN_TX | PIN_RX);
    gpio_set_output_options(PORT_UART, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, 
        PIN_TX | PIN_RX);
	// from datasheet, AF1 is the USART function
    gpio_set_af(PORT_UART, GPIO_AF1, 
        PIN_TX | PIN_RX);

	usart_enable(USART1);
}

static void pwm_setup(void) {
	rcc_periph_clock_enable(RCC_TIM3); //ch 1, 2
	rcc_periph_clock_enable(RCC_TIM1); //ch 3
	rcc_periph_clock_enable(RCC_GPIOA); //all pwm pins are on port A
	
    gpio_mode_setup(PORT_PWM, GPIO_MODE_AF, GPIO_PUPD_NONE, 
        PIN_CH1 | PIN_CH2 | PIN_CH3);
    gpio_set_output_options(PORT_PWM, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, 
        PIN_CH1 | PIN_CH2 | PIN_CH3);

    gpio_set_af(PORT_PWM, GPIO_AF1, //ch 1, 2 use AF mode 1
		PIN_CH1 | PIN_CH2);
	gpio_set_af(PORT_PWM, GPIO_AF2, //ch3 uses AF mode 2
		PIN_CH3);

	// Reset the timer configuration and then set it up to use the CPU clock,
	// center-aligned PWM, and an increasing rate.
	timer_reset(TIM3);
	timer_reset(TIM1);
	timer_set_prescaler(TIM3, 0);
	timer_set_prescaler(TIM1, 0);
	timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_CENTER_1, TIM_CR1_DIR_UP);
	timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_CENTER_1, TIM_CR1_DIR_UP);
	timer_set_period(TIM3, PERIOD_VALUE);
	timer_set_period(TIM1, PERIOD_VALUE);
	//needed for advanced timers?
	timer_enable_break_main_output(TIM1);

    // Enable the channel PWM output.
    timer_enable_oc_output(TIM3, TIM_OC1); 	//ch1
	timer_enable_oc_output(TIM3, TIM_OC2);	//ch2
	timer_enable_oc_output(TIM1, TIM_OC2);	//ch3

	// PWM mode 2 inverts drive signals. [mode 1 == standard]
	#ifdef DRV_INVERTED
	#define PWM_TIMER_MODE TIM_OCM_PWM2
	#else
	#define PWM_TIMER_MODE TIM_OCM_PWM1
    #endif
    
    timer_set_oc_mode(TIM3, TIM_OC1, PWM_TIMER_MODE);
	timer_set_oc_mode(TIM3, TIM_OC2, PWM_TIMER_MODE);
	timer_set_oc_mode(TIM1, TIM_OC2, PWM_TIMER_MODE);
	timer_enable_counter(TIM3);
	timer_enable_counter(TIM1);
    timer_set_oc_value(TIM3, TIM_OC1, 0);
	timer_set_oc_value(TIM3, TIM_OC2, 0);
	timer_set_oc_value(TIM1, TIM_OC2, 0);
}

static void pwm_update_ch(uint32_t timer_threshold, int channel) {
	//discard input outside range
	if(timer_threshold >= PERIOD_VALUE)
		return;
	
	switch(channel) {
		case 1:
		timer_set_oc_value(TIM3, TIM_OC1, timer_threshold); break;
		case 2:
		timer_set_oc_value(TIM3, TIM_OC2, timer_threshold); break;
		case 3:
		timer_set_oc_value(TIM1, TIM_OC2, timer_threshold); break;
	}
	
}

static void pwm_update_ch_pct(float duty_cycle, int channel) {
	uint32_t timer_threshold = PERIOD_VALUE * duty_cycle;
	pwm_update_ch(timer_threshold, channel);
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
		usart_print(receipt);
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
		usart_print("error   -- PWM value set too high, ignoring");
		command_is_sane = false;
	} 
	if (pwm_channel > NUM_PWM_CHANNELS) {
		usart_print("error   -- target channel not found, ignoring");
		command_is_sane = false;
	}

	//if a valid command, commit payload data to HW
	if(command_is_sane) {
		pwm_update_ch(pwm_level, pwm_channel);
		usart_print("committed");
		usart_send_str("\t"); 	//indent sucessful commands
	}
	
	//send acknowledgement string
	const int bufsz = 64;
	char receipt[bufsz];
	mini_snprintf(receipt, bufsz, "CH: %u  PWR: %u", 
		pwm_channel, pwm_level);
	usart_print(receipt);
}


void sssp_receive_loop() {
	char buffer_rx[sizeof(sssp_packet_pwm_t) + 1];
	buffer_rx[sizeof(sssp_packet_pwm_t)] = '\0';
	uint32_t buffer_idx = 0;

	while(1) {
		char input_char = usart_recv_blocking(USART1);

		gpio_set(PORT_LED, PIN_LED);
		if(char_is_alphanumeric(input_char)) {
			//echo char, append to buffer if space available
			usart_send_blocking(USART1, input_char);
			if(buffer_idx < sizeof(sssp_packet_pwm_t)){
				buffer_rx[buffer_idx] = input_char;
				buffer_idx++;
			}
		} else if(input_char == ENTER_CHAR) {
			//visually end user input with NL+CR
			usart_newln();
			//forward packet for further processing
			sssp_process_packet_pwm((sssp_packet_pwm_t*) buffer_rx);
			//clear buffered string and reset write position
			memset(buffer_rx, '\0', sizeof(sssp_packet_pwm_t));
			buffer_idx = 0;
		}
		gpio_clear(PORT_LED, PIN_LED);
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