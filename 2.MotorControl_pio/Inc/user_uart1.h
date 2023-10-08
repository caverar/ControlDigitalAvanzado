#ifndef __USER_UART1_H__
#define __USER_UART1_H__

#ifdef __cplusplus
extern "C" {
#endif

// Standard includes
#include <stdbool.h>
#include <stdint.h>
// BSP includes
#include "stm32f4xx_hal.h"
// User includes

// Private defines
#define UART1_RX_BUFFER_LEN 50
#define UART_ITER_TIME_OUT 100000
// Private variables
uint32_t uart1_rx_dma_ready_time_out_iter;
volatile char uart1_rx_buffer[UART1_RX_BUFFER_LEN];

// Function prototypes
int _write(int file, char* ptr, int len);
void UART1_RX_DMA_Clean(void);
bool UART1_RX_DMA_Ready(void);
void UART1_RX_DMA_StartReceive(void);
void UART1_RX_DMA_StopReceive(void);
void UART1_RX_DMA_Read(char* buffer);

#ifdef __cplusplus
}
#endif

#endif // __USER_UART1_H__