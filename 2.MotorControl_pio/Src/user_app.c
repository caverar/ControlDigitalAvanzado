#include "user_app.h"

// Standard libraries
#include <stdio.h>
#include <string.h>

// BSP libraries
#include "tim.h"
#include <dataAcq.h>
// User libraries
#include "user_uart1.h"

// Function implementations
// Peripheral configuration
void user_app_init(void) {
    // Enable Timers
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    // Config Systick interruption Frequency
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 200); // 5ms

    // Start UART DMA reception
    UART1_RX_DMA_Clean();
    UART1_RX_DMA_StartReceive();

    // Variables initialization
    k = 0;
    omega = 0;
    omega_old = 0;
    theta = 0;
    encoder_value = 0;
    encoder_value_old = 0;
    trigger = 0;
    u = 0;
    printf("Start\n");
}

void user_app_interrupt(void) {
    // trigger = 1;
#ifdef IDENT_MODE
    get_motor_speed();
    if (k < 100) {
        u = 0.5f;
        k++;
    } else if (k < 200) {
        u = 0.9f;
        k++;
    } else if (k < 300) {
        u = 0.5f;
        k++;
    } else if (k < 400) {
        u = 0.9f;
        k++;
    } else if (k < 500) {
        u = 0.5f;
        k++;
    } else if (k < 600) {
        u = 0.9f;
        k++;
    } else if (k < 700) {
        u = 0.5f;
        k++;
    } else if (k < 800) {
        u = 0.9f;
        k++;
    } else if (k < 900) {
        u = 0.5f;
        k++;
    } else {
        k = 0;
    }
    set_motor_pwm(u);
    if (k > 50)
        printf("%d, %0.2f, %0.6f\n", k, u, omega);

#elif VALIDATION_DATA_MODE

    get_motor_speed();
    if (k < 100) {
        u = 0.4f;
        k++;
    } else if (k < 200) {
        u = 0.7f;
        k++;
    } else {
        k = 0;
    }

    set_motor_pwm(u);
    printf("%d, %0.2f, %0.6f\n", k, u, omega);

#elif CONTROL_MODE
    printf("test");
#endif

#ifdef LOG_TO_STM32MONITOR
    trigger = 2;
    DumpTrace();
#endif
}
char data_save[50];
void user_app_main(void) {
    if (UART1_RX_DMA_Ready()) {
        // char buffer[UART1_RX_BUFFER_LEN];
        UART1_RX_DMA_Read(data_save);
        printf(strcat(data_save, "\n"));
        UART1_RX_DMA_StartReceive();
    }
}

void set_motor_pwm(float value) {
    if (value >= 1) {
        TIM2->CCR1 = 1000;
        TIM2->CCR2 = 0;
    } else if (value < 1 && value >= 0) {
        TIM2->CCR1 = value * 1000;
        TIM2->CCR2 = 0;
    } else if (value <= -1) {
        TIM2->CCR1 = 0;
        TIM2->CCR2 = (-value) * 1000;
    } else if (value < 0 && value > -1) {
        TIM2->CCR1 = 0;
        TIM2->CCR2 = -1000;
    }
}
void get_motor_speed(void) {

    encoder_value = TIM3->CNT;
    // Motor position in degrees
    theta = 360 * (float)((u_int32_t)encoder_value / COUNTS_PER_REVOLUTION);
    // Motor angular velocity in rev/s
    omega = (int)(encoder_value - encoder_value_old) * 200.0f / 28800.0f;

    // Filter the ecoder_value jumpsdue to the counter overflow
    if ((encoder_value - encoder_value_old) > 1000) {
        omega = omega_old;
    }

    if ((omega >= 50.0f) || (omega <= -50.0f)) {
        omega = omega_old;
    }

    encoder_value_old = encoder_value;
    omega_old = omega;
}