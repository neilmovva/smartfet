#include "console.h"

void usart_send_str(const char* str) {
	for(uint32_t idx_c = 0; idx_c < strlen(str); idx_c++ ){
		usart_send_blocking(USART1, str[idx_c]);
	}
}

void usart_print(const char* str) {
	usart_send_str(str);
	usart_send_str(STR_NEWLINE_CHARS);
}

bool char_is_alphanumeric(const char c) {
	bool is_num = (c >= 48U) && (c <= 57U);
	bool is_uppercase = (c >= 65U) && (c <= 90U);
	bool is_lowercase = (c >= 97U) && (c <= 122U);
	return (is_num || is_uppercase || is_lowercase);
}

int myAtoi(const char *str) {
    int res = 0; // Initialize result
    for(int i = 0; str[i] != '\0'; ++i)
        res = res*10 + str[i] - '0';
    return res;
}