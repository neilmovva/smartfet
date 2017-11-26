#ifndef CONSOLE_H
#define CONSOLE_H

#include <libopencm3/stm32/usart.h>
#include <string.h>

const char* STR_NEWLINE_CHARS 	= 	"\n\r";

void usart_send_str(const char* str);
void usart_print(const char* str);

bool char_is_alphanumeric(const char c);
int myAtoi(const char *str);

#endif //CONSOLE_H