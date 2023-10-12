#include "user_app.h"

// Standard libraries
#include <stdio.h>
#include <stdlib.h>
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

#ifdef PARAMETER_TUNING_MODE
    // Start UART DMA reception
    UART1_RX_DMA_Clean();
    UART1_RX_DMA_StartReceive();
#endif
    // Variable initialization
    k = 0;
    omega = 0;
    omega_old = 0;
    theta = 0;
    encoder_value = 0;
    encoder_value_old = 0;
    trigger = 0;
    u = 0;
    u_old1 = 0;
    u_old2 = 0;
    u_old3 = 0;
    u_old4 = 0;
    e = 0;
    e_old1 = 0;
    e_old2 = 0;
    e_old3 = 0;
    e_old4 = 0;
    r = 0;
    min_ref = 0.7f;
    max_ref = 1.2f;

    // PI
    Kp = 0.74773;
    Ki = 31.7677;
    // 2x lead compensators in w
    a0 = 0;
    a1 = 0;
    a2 = 0.2718;
    a3 = -0.5012;
    a4 = 0.2308;
    b0 = 1;
    b1 = -2.416;
    b2 = 1.8;
    b3 = -0.3513;
    b4 = -0.03255;
    sel_ctrl = 0;
}

void user_app_interrupt(void) {

#ifdef IDENT_MODE
    get_motor_speed();
    if (k < 100) {
        u = 0.6f;
        k++;
    } else if (k < 200) {
        u = 0.8f;
        k++;
    } else {
        k = 0;
    }
    set_motor_pwm(u);
    // printf("Kp=%0.2f,Ki=%0.2f\n", Kp, Ki);
    printf("%d, %0.2f, %0.8f\n", k, u, omega);

#elif defined(CONTROL_MODE)

    get_motor_speed();

    if (sel_ctrl == 0) { // PI

        // Reference
        if (k < 100) {
            r = min_ref;
            k++;
        } else if (k < 200) {
            r = max_ref;
            k++;
        } else {
            k = 0;
        }
        // Comparator (Error)
        e = r - omega;
        // Control Law (Proportional + Integral)
        u = ((Kp + (Ki * Ts)) * e) - (Kp * e_old1) + u_old1;

        // Anti-windup
        if (u > 1) {
            u = 1;
        } else if (u < -1) {
            u = -1;
        }
        // Save past values
        u_old1 = u;
        e_old1 = e;
        omega_old = omega;
    } else if (sel_ctrl == 1) { // 2x lead compensators in w
        // Reference
        if (k < 200) {
            r = min_ref;
            k++;
        } else if (k < 400) {
            r = max_ref;
            k++;
        } else {
            k = 0;
        }
        // Comparator (Error)
        e = r - omega;
        // Control Law (2 integrators plus 2 lead compensators)
        u = (((a0 * e + a1 * e_old1 + a2 * e_old2 + a3 * e_old3 + a4 * e_old4)
                - (b1 * u_old1 + b2 * u_old2 + b3 * u_old3 + b4 * u_old4)))
            / b0;
        // Anti-windup
        if (u > 1) {
            u = 1;
        } else if (u < -1) {
            u = -1;
        }
        // Save past values
        u_old4 = u_old3;
        u_old3 = u_old2;
        u_old2 = u_old1;
        u_old1 = u;
        e_old4 = e_old3;
        e_old3 = e_old2;
        e_old2 = e_old1;
        e_old1 = e;
    }
    set_motor_pwm(u);
    printf("%0.2f, %0.2f, %0.8f\n", r, u, omega);

#endif

#ifdef LOG_TO_STM32MONITOR
    trigger = 2;
    DumpTrace();
#endif
#ifdef PARAMETER_TUNING_MODE
    // UART Rx
    if (UART1_RX_DMA_Ready()) {

        UART1_RX_DMA_Read(rx_buffer);
        string_parser(rx_buffer);
        UART1_RX_DMA_StartReceive();
    }
#endif
}

void user_app_main(void) {
    if (UART1_RX_DMA_Ready()) {
        UART1_RX_DMA_Read(rx_buffer);
        printf(strcat(rx_buffer, "\n"));
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

void string_parser(char* input) {

    char* token;
    char* saveptr;
    char* saveptr2;
    // Tokenize the string with the delimiter ","
    token = strtok_r(input, ",", &saveptr);
    // strcpy(inner_string, token);

    // While for each token pair
    while (token != NULL) {
        // Tokenize the string with the delimiter "="
        char* name = strtok_r(token, "=", &saveptr2);
        char* value = strtok_r(NULL, "=", &saveptr2);
        if (name != NULL && value != NULL) {
            if (!strcmp(name, "Kp")) {
                Kp = atof(value);
            } else if (!strcmp(name, "Ki")) {
                Ki = atof(value);
            } else if (!strcmp(name, "min_ref")) {
                min_ref = atof(value);
            } else if (!strcmp(name, "max_ref")) {
                max_ref = atof(value);
            } else if (!strcmp(name, "sel_ctrl")) {
                sel_ctrl = atoi(value);
            }
            token = strtok_r(NULL, ",", &saveptr);
        }
    }
}