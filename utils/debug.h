#pragma  once
#include <stdio.h>
#include <stdint.h>
#define DEBUG_ASSERT
#define DEBUG_PRINTF
#define LOG_PRINTF
#define DEBUG_EXIT

#ifdef DEBUG_ASSERT
	#define assert(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
#else
	#define assert(expr) ((void)0)
#endif

#ifdef DEBUG_PRINTF
	#define debug_printf(X, p...)  printf("\033[31m"X "\033[0m\n", ##p)
#else
	#define debug_printf		((void)0)
#endif

#ifdef LOG_PRINTF
	#define log_printf(X, p...)	printf("\033[33m" X "\033[0m", ##p)
	#define log_puts(X)			puts("\033[33m" X "\033[0m")
#else
	#define log_printf		((void)0)
	#define log_puts(X)		((void)0)
#endif

#ifdef DEBUG_EXIT
	#define debug_exit()	while(1){}
#else
	#define debug_exit()	((void)0)
#endif

void assert_failed(uint8_t* file, uint32_t line);
