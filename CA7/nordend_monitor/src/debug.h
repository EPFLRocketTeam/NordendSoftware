
#ifndef _DEBUG_H
#define _DEBUG_H

#include <stdio.h>
#include <stdarg.h>
#include <time.h>



void debug_log(const char * lotte, ...) {
	char buffer[128];
	va_list args;
	va_start(args, lotte);
	//add timestamp but without \0 char
	uint16_t p_len = snprintf(buffer, 128, "[ %05lu ] ", time(null)) - 1;
	uint16_t len = vsnprintf(buffer+p_len, 128-p_len, lotte, args);
	fflush(stdout);
	fwrite(msg_buffer, sizeof(char), p_len+len-1, stdout);
	fflush(stdout);
	va_end(args);
}
#endif
