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
const char* STR_NEWLINE 	= 	"\n\r";
#define ENTER_CHAR 				0x0D

static void inline busywait_ms(int ms) {
	for (int i = 0; i < 6000 * ms; i++) {	/* Wait a bit. */
		__asm__("nop");
	}
}

static void usart_send_str(const char* str) {
	for(uint32_t idx_c = 0; idx_c < strlen(str); idx_c++ ){
		usart_send_blocking(USART1, str[idx_c]);
	}
}

static void usart_print(const char* str) {
	usart_send_str(str);
	usart_send_str(STR_NEWLINE);
}

static bool char_is_alphanumeric(char c) {
	bool is_num = (c >= 48U) && (c <= 57U);
	bool is_uppercase = (c >= 65U) && (c <= 90U);
	bool is_lowercase = (c >= 97U) && (c <= 122U);
	return (is_num || is_uppercase || is_lowercase);
}

static int myAtoi(const char *str) {
    int res = 0; // Initialize result
    for (int i = 0; str[i] != '\0'; ++i)
        res = res*10 + str[i] - '0';
    return res;
}


static void gpio_setup(void) {
	/* Enable GPIOB clock. */
	rcc_periph_clock_enable(RCC_GPIOB);
	gpio_mode_setup(PORT_LED, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN_LED);
}

static void usart_setup(void) {
	rcc_periph_clock_enable(RCC_USART1);
	rcc_periph_clock_enable(RCC_GPIOA);

	/* Setup USART1 parameters. for 8-N-1 at 38400 baud*/
	usart_set_baudrate(USART1, 38400);
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
    rcc_periph_clock_enable(RCC_TIM3);
    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_mode_setup(PORT_PWM, GPIO_MODE_AF, GPIO_PUPD_NONE, 
        PIN_CH1 | PIN_CH2);
    gpio_set_output_options(PORT_PWM, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, 
        PIN_CH1 | PIN_CH2);
	// from datasheet, AF1 is the PWM / TIM function
    gpio_set_af(PORT_PWM, GPIO_AF1, 
        PIN_CH1 | PIN_CH2);

	// Reset the timer configuration and then set it up to use the CPU clock,
	// center-aligned PWM, and an increasing rate.
	timer_reset(TIM3);
	timer_set_prescaler(TIM3, 0);
	timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_CENTER_1, TIM_CR1_DIR_UP);
	timer_set_period(TIM3, PERIOD_VALUE);
	//needed for advanced timers?
	// timer_enable_break_main_output(TIM3);

	// SW pin is timer 3, channel 2 (pin A7).
    // Enable the channel PWM output.
    timer_enable_oc_output(TIM3, TIM_OC1);
	timer_enable_oc_output(TIM3, TIM_OC2);

	// if the SW signal must be inverted, use PWM mode 2. [mode 1 == standard]
	#ifdef DRV_INVERTED
	#define PWM_TIMER_MODE TIM_OCM_PWM2
	#else
	#define PWM_TIMER_MODE TIM_OCM_PWM1
    #endif
    
    timer_set_oc_mode(TIM3, TIM_OC1, PWM_TIMER_MODE);
    timer_set_oc_mode(TIM3, TIM_OC2, PWM_TIMER_MODE);
	timer_enable_counter(TIM3);
    timer_set_oc_value(TIM3, TIM_OC1, 0);
	timer_set_oc_value(TIM3, TIM_OC2, 0);
}

static void pwm_update_ch(uint32_t timer_threshold, int channel) {
	// if(pulseVal = 0) {
	// 	timer_disable_oc_output(TIM3, TIM_OC2);
	// 	#ifdef DRV_INVERTED
	// 	gpio_set(PORT_SW, PIN_SW);
	// 	#else
	// 	gpio_clear(PORT_SW, PIN_SW);
	// 	#endif
	// 	is_pwm_stopped_SW = true;
	// 	return;
	// } 
	
	// if(is_pwm_stopped_SW) {
	// 	is_pwm_stopped_SW = false;
	// 	pwm_setup();
	// }

	//discard input outside range
	if(timer_threshold >= PERIOD_VALUE)
		return;

    if(channel == 1) {
        timer_set_oc_value(TIM3, TIM_OC1, timer_threshold);
    } else if(channel == 2) {
        timer_set_oc_value(TIM3, TIM_OC2, timer_threshold);
    }
	
}

static void pwm_update_ch_pct(float duty_cycle, int channel) {
	uint32_t timer_threshold = PERIOD_VALUE * duty_cycle;
	pwm_update_ch(timer_threshold, channel);
}

void sssp_process_packet_pwm(sssp_packet_pwm_t* pkt) {
	int cmp_hdr = strncmp(pkt->hdr, BOOKEND_HDR, sizeof(pkt->hdr));
	int cmp_ftr = strncmp(pkt->ftr, BOOKEND_FTR, sizeof(pkt->ftr));
	if(cmp_hdr != 0 || cmp_ftr != 0){
		char str_hdr[sizeof(pkt->hdr) + 1] = {};
		char str_ftr[sizeof(pkt->ftr) + 1] = {};
		strncat(str_hdr, pkt->hdr, sizeof(pkt->hdr));
		strncat(str_ftr, pkt->ftr, sizeof(pkt->ftr));

		const int bufsz = 64;
		char receipt[bufsz];
		mini_snprintf(receipt, bufsz, 
			"invalid; hdr: %s  ftr: %s  BUF: %s", 
			str_hdr, str_ftr, pkt);
		usart_print(receipt);

		return;
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
		usart_print("err: PWM value set too high, ignoring");
		command_is_sane = false;
	} 
	if (pwm_channel > NUM_PWM_CHANNELS) {
		usart_print("err: target channel not found, ignoring");
		command_is_sane = false;
	}


	//if a valid command, commit payload data to HW
	if(command_is_sane)
		pwm_update_ch(pwm_level, pwm_channel);

	
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
		if(input_char == ENTER_CHAR) {
			//terminate buffered string and reset write position
			buffer_rx[buffer_idx] = '\0';
			buffer_idx = 0;
			//visually end user input with NL+CR
			usart_send_str(STR_NEWLINE);
			
			//forward packet for further processing
			sssp_process_packet_pwm((sssp_packet_pwm_t*) buffer_rx);
		}  
		if(char_is_alphanumeric(input_char)) {
			//echo char, append to buffer if space
			usart_send_blocking(USART1, input_char);
			if(buffer_idx < sizeof(sssp_packet_pwm_t)){
				buffer_rx[buffer_idx] = input_char;
				buffer_idx++;
			}
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
    
    pwm_update_ch_pct(0.05, 1);
	pwm_update_ch_pct(0.25, 2);
	
	sssp_receive_loop();

	while (1) {
		gpio_set(PORT_LED, PIN_LED);
		busywait_ms(5);
		gpio_clear(PORT_LED, PIN_LED);
		busywait_ms(90);
		gpio_set(PORT_LED, PIN_LED);
		busywait_ms(5);
		gpio_clear(PORT_LED, PIN_LED);

		busywait_ms(1400);
	}

	return 0;
}