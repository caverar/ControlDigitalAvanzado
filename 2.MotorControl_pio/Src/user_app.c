#include "user_app.h"

// Standard libraries
#include <stdio.h>
#include <string.h>

// BSP libraries
#include "stm32f4xx_hal.h"
#include "tim.h"
// User libraries
#include "user_uart1.h"
// Private variables

// Function implementations
// Peripheral configuration
void user_app_init(void) {
    // Enable Timers
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    // Config Systick interruption Frequency
}

void user_app_main(void) {

    int x = 0;
    UART1_RX_DMA_Clean();
    UART1_RX_DMA_StartReceive();

    while (true) {
        if (UART1_RX_DMA_Ready()) {
            UART1_RX_DMA_StopReceive();
            char buffer[40];
            UART1_RX_DMA_Read(buffer);
            printf(strcat(buffer, "\n"));
            UART1_RX_DMA_StartReceive();

        } else {
            printf("Hola Mundo: %d \n", x++);
        } /* code */
        HAL_Delay(500);
    }
}