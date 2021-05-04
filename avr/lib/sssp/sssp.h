#pragma once

//SSSP = Sail Simple Serial Protocol
struct sssp_packet_pwm {
	char hdr[4];
	char powerlevel[3];
	char channel[1];
	char ftr[4];
};
typedef sssp_packet_pwm sssp_packet_pwm_t; 

const char* BOOKEND_HDR 	=	"sail";
const char* BOOKEND_FTR  	=	"rsch";


bool char_is_alphanumeric(const char c) {
	bool is_num = (c >= 48U) && (c <= 57U);
	bool is_uppercase = (c >= 65U) && (c <= 90U);
	bool is_lowercase = (c >= 97U) && (c <= 122U);
	return (is_num || is_uppercase || is_lowercase);
}

uint8_t parse_powerlevel(const char* str_pwmlevel) {
    uint8_t d_hun = str_pwmlevel[0] - '0'; 
	uint8_t d_ten = str_pwmlevel[1] - '0'; 
	uint8_t d_one = str_pwmlevel[2] - '0';

	uint8_t result = d_hun*100 + d_ten*10 + d_one*1;
	return result;
}

uint8_t parse_channel(const char* str_pwmch) {
	uint8_t d_one = str_pwmch[0] - '0';

	uint8_t result = d_one*1;
	return result;
}

