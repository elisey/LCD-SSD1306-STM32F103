#include "debug.h"

void assert_failed(uint8_t* file, uint32_t line)
{
	debug_printf("Assert failed: %s:%u\n", (char*)file, (unsigned int)line);
	while(1);
}
