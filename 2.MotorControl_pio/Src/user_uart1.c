#include "user_uart1.h"

// Standard libraries
#include <stdio.h>
#include <string.h>
// BSP libraries
#include "usart.h"
// User libraries

// Function implementations
/**
 * @brief Callback for printf
 */
int _write(int file, char* ptr, int len) {
    // Clean busy flag from past transfer (Assuming that the past transfer is
    // completed, to avoid interruptions usage).
    huart1.gState = HAL_UART_STATE_READY;
    // Transfer the printf string using DMA.
    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)ptr, len);
    return len;
}

void UART1_RX_DMA_Clean(void) {

    // memset not allowed due to volatile
    // memset(uart1_rx_buffer, 0x00, uart1_rx_buffer_len);
    // Fill the buffer with 0x00
    for (int i = 0; i < UART1_RX_BUFFER_LEN; i++) {
        uart1_rx_buffer[i] = 0x00;
    }
    // Reset the time_out iterator
    uart1_rx_dma_ready_time_out_iter = 0;
}

bool UART1_RX_DMA_Ready(void) {
    // Check if the DMA is done with the transfer
    bool not_null_characters = false;

    for (int i = 0; i < UART1_RX_BUFFER_LEN; i++) {

        if (uart1_rx_buffer[i] == '\n') {
            return true;

        } // Check if there are null characters
        else if (uart1_rx_buffer[i] != 0x00) {
            not_null_characters = true;
        }
    }
    // If there are not null characters increase the time_out iterator
    if (not_null_characters) {
        uart1_rx_dma_ready_time_out_iter++;
        // if the time out has been reached clean the buffer and restart the DMA
        if (uart1_rx_dma_ready_time_out_iter >= UART_ITER_TIME_OUT) {
            UART1_RX_DMA_Clean();
            UART1_RX_DMA_StopReceive();
            UART1_RX_DMA_StartReceive();
        }
    }
    return false;
}

void UART1_RX_DMA_StartReceive(void) {
    // Start the DMA transfer
    uart1_rx_dma_ready_time_out_iter = 0;
    HAL_UART_Receive_DMA(&huart1, (unsigned char*)((uintptr_t)uart1_rx_buffer),
        UART1_RX_BUFFER_LEN);
}

void UART1_RX_DMA_StopReceive(void) {
    HAL_UART_DMAStop(&huart1);
}

void UART1_RX_DMA_Read(char* buffer) {
    UART1_RX_DMA_StopReceive();
    memcpy((void*)buffer, (void*)strtok((char*)uart1_rx_buffer, "\n"),
        UART1_RX_BUFFER_LEN);
    UART1_RX_DMA_Clean();
}