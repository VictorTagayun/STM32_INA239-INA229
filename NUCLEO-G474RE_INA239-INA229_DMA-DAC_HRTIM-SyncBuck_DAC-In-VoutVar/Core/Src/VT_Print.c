/* Includes ------------------------------------------------------------------*/
#include "VT_Print.h"
//#include <stdio.h> // add in main.c
#include "main.h"

extern UART_HandleTypeDef hlpuart1;

#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

PUTCHAR_PROTOTYPE {
	/* Place your implementation of fputc here */
	/* e.g. write a character to the LPUART1 and Loop until the end of transmission */
	HAL_UART_Transmit(&hlpuart1, (uint8_t*) &ch, 1, 0xFFFF);
//	HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t*) &ch, 1);

	return ch;
}
