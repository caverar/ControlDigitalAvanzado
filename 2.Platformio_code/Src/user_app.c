#include "user_app.h"

// Includes *******************************************************************

// Standard libraries ---------------------------------------------------------
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
// BSP libraries --------------------------------------------------------------
#include "tim.h"
#include <dataAcq.h>
// User libraries -------------------------------------------------------------
#include "user_uart1.h"

// Function implementations ***************************************************

/**
 * @brief  This function is executed once at the beginning of the program, it
 * contains the peripheral configuration and the variable initialization.
 * @param  None
 * @retval None
 */
void user_app_init(void) {
    // Peripheral configuration
    // Enable Timers
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    // Config SysTick interruption Frequency
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 200); // 5ms

#ifdef PARAMETER_TUNING_MODE
    // Start UART DMA reception
    UART1_RX_DMA_Clean();
    UART1_RX_DMA_StartReceive();
#endif
    // Variable initialization
    k = 0;
    trigger = 0;

    // Process variables
    omega = 0;
    omega_old = 0;
    theta = 0;
    encoder_value = 0;
    encoder_value_old = 0;
    u = 0.0, u_old1 = 0.0, u_old2 = 0.0, u_old3 = 0.0, u_old4 = 0.0;
    e = 0.0, e_old1 = 0.0, e_old2 = 0.0, e_old3 = 0.0, e_old4 = 0.0;

    // Reference parameters
    selected_reference = SQUARE;
    r = 0.0;
    min_ref = 0.8f;
    max_ref = 1.3f;
    ref_period_seconds = 4; // 0.25HZ same as simulation.
    ref_period_samples = (uint16_t)(ref_period_seconds / Ts);

    // Controllers
    selected_controller = PI_ZOH;
    // Kp = 0.74773;
    // Ki = 31.7677;
    // PI_ZOH
    pi_zoh_a0 = 0.74773, pi_zoh_a1 = -0.58889;
    pi_zoh_b0 = 1, pi_zoh_b1 = -1;
    // PI_TUS
    pi_tus_a0 = 0.82715, pi_tus_a1 = -0.66831;
    pi_tus_b0 = 1, pi_tus_b1 = -1;
    // PI_IIN
    pi_iin_a0 = 0.15884, pi_iin_a1 = -4.9878e-17;
    pi_iin_b0 = 1, pi_iin_b1 = -1;
    // LC_W
    l1 = 0, l1_old1 = 0, l1_old2 = 0;
    l2 = 0, l2_old1 = 0, l2_old2 = 0;
    lcw_c2_a0 = 6.2733, lcw_c2_a1 = -5.6021;
    lcw_c2_b0 = 1, lcw_c2_b1 = -0.48367;
    lcw_c1_a0 = 21.664, lcw_c1_a1 = -20.597;
    lcw_c1_b0 = 1, lcw_c1_b1 = 0.067306;
    lcw_i_a0 = 0, lcw_i_a1 = 0, lcw_i_a2 = 0.002;
    lcw_i_b0 = 1, lcw_i_b1 = -2, lcw_i_b2 = 1;
    // LC_S_ZOH
    l = 0, l_old1 = 0, l_old2 = 0;
    lcs_c_zoh_a0 = 914.42, lcs_c_zoh_a1 = -907.88;
    lcs_c_zoh_b0 = 1, lcs_c_zoh_b1 = -0.34563;
    lcs_i_zoh_a0 = 0, lcs_i_zoh_a1 = 2.5e-05, lcs_i_zoh_a2 = 2.5e-05;
    lcs_i_zoh_b0 = 1, lcs_i_zoh_b1 = -2, lcs_i_zoh_b2 = 1;
    // LC_S_TUS
    l = 0, l_old1 = 0, l_old2 = 0;
    lcs_c_tus_a0 = 600.66, lcs_c_tus_a1 = -593.72;
    lcs_c_tus_b0 = 1, lcs_c_tus_b1 = -0.30617;
    lcs_i_tus_a0 = 1.25e-05, lcs_i_tus_a1 = 2.5e-05, lcs_i_tus_a2 = 1.25e-05;
    lcs_i_tus_b0 = 1, lcs_i_tus_b1 = -2, lcs_i_tus_b2 = 1;
    // LC_S_BAP
    l = 0, l_old1 = 0, l_old2 = 0;
    lcs_c_bap_a0 = 448.53, lcs_c_bap_a1 = -443.38;
    lcs_c_bap_b0 = 1, lcs_c_bap_b1 = -0.48487;
    lcs_i_bap_a0 = 5e-05, lcs_i_bap_a1 = 0, lcs_i_bap_a2 = 0;
    lcs_i_bap_b0 = 1, lcs_i_bap_b1 = -2, lcs_i_bap_b2 = 1;
}

/**
 * @brief  This function is executed at each SysTick interruption, it contains
 * the encoder reading, the reference generator, the control algorithm, the pwm
 * set, the stm32monitor buffering, an UART parser for parameter tuning, and an
 * UART print for data visualization.
 * @param  None
 * @retval None
 */
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
        u = 0.6f;
        k = 0;
    }
    set_motor_pwm(u);
    printf("%d, %0.2f, %0.8f\n", k, u, omega);

#elif defined(CONTROL_MODE)

    get_motor_speed();
    // Reference
    switch (selected_reference) {
    case SQUARE:
        if (k < ref_period_samples / 2) {
            r = min_ref;
            k++;
        } else if (k < ref_period_samples - 1) {
            r = max_ref;
            k++;
        } else {
            k = 0;
            r = max_ref;
        }
        break;
    case TRIANGULAR:
        if (k < ref_period_samples / 2) {
            r = min_ref
                + (max_ref - min_ref) * (float)k / (ref_period_samples / 2);
            k++;
        } else if (k < ref_period_samples - 1) {
            r = max_ref
                - (max_ref - min_ref) * (float)(k - ref_period_samples / 2)
                    / (ref_period_samples / 2);
            k++;
        } else {
            k = 0;
            r = max_ref
                - (max_ref - min_ref) * (float)(k - ref_period_samples / 2)
                    / (ref_period_samples / 2);
        }
        break;
    default:
        break;
    }

    // Control
    switch (selected_controller) {
    case PI_ZOH:

        // Error
        e = r - omega;
        // Control Law (Proportional + Integral)
        // u = ((Kp + (Ki * Ts)) * e) - (Kp * e_old1) + u_old1;
        u = ((pi_zoh_a0 * e + pi_zoh_a1 * e_old1) - (pi_zoh_b1 * u_old1))
            / pi_zoh_b0;
        // Anti-windup
        if (u > 1) {
            u = 1;
        } else if (u < -1) {
            u = -1;
        }
        // Save past values
        u_old1 = u;
        e_old1 = e;
        break;

    case PI_TUS:

        // Error
        e = r - omega;
        // Control Law (Proportional + Integral)
        u = ((pi_tus_a0 * e + pi_tus_a1 * e_old1) - (pi_tus_b1 * u_old1))
            / pi_tus_b0;
        // Anti-windup
        if (u > 1) {
            u = 1;
        } else if (u < -1) {
            u = -1;
        }
        // Save past values
        u_old1 = u;
        e_old1 = e;
        break;

    case PI_IIN:

        // Error
        e = r - omega;
        // Control Law (Proportional + Integral)
        u = ((pi_iin_a0 * e + pi_iin_a1 * e_old1) - (pi_iin_b1 * u_old1))
            / pi_iin_b0;
        // Anti-windup
        if (u > 1) {
            u = 1;
        } else if (u < -1) {
            u = -1;
        }
        // Save past values
        u_old1 = u;
        e_old1 = e;
        break;

    case LC_W: // 2x lead compensators designed with the w transform and then
               // transformed to z, plus 2x integrators in z.

        // Error
        e = r - omega;

        // Control Law
        // lead compensator 2
        l2 = ((lcw_c2_a0 * e + lcw_c2_a1 * e_old1) - (lcw_c2_b1 * l2_old1))
            / lcw_c2_b0;

        // lead compensator 1
        l1 = ((lcw_c1_a0 * l2 + lcw_c1_a1 * l2_old1) - (lcw_c1_b1 * l1_old1))
            / lcw_c1_b0;

        // Double integrator
        u = ((lcw_i_a0 * l1 + lcw_i_a1 * l1_old1 + lcw_i_a2 * l1_old2)
                - (lcw_i_b1 * u_old1 + lcw_i_b2 * u_old2))
            / lcw_i_b0;

        // Anti - windup
        if (u > 1) {
            u = 1;
        } else if (u < -1) {
            u = -1;
        }
        // Save past values
        u_old2 = u_old1;
        u_old1 = u;
        l1_old2 = l1_old1;
        l1_old1 = l1;
        l2_old2 = l2_old1;
        l2_old1 = l2;
        e_old1 = e;
        break;

    case LC_S_ZOH: // lead compensator plus 2 integrators designed in s and
                   // discretized with ZOH method.

        // Error
        e = r - omega;

        // Control Law

        // lead compensator
        l = ((lcs_c_zoh_a0 * e + lcs_c_zoh_a1 * e_old1)
                - (lcs_c_zoh_b1 * l_old1))
            / lcs_c_zoh_b0;

        // Double integrator
        u = ((lcs_i_zoh_a0 * l + lcs_i_zoh_a1 * l_old1 + lcs_i_zoh_a2 * l_old2)
                - (lcs_i_zoh_b1 * u_old1 + lcs_i_zoh_b2 * u_old2))
            / lcs_i_zoh_b0;

        // Anti-windup
        if (u > 1) {
            u = 1;
        } else if (u < -1) {
            u = -1;
        }
        // Save past values
        u_old2 = u_old1;
        u_old1 = u;
        l_old2 = l_old1;
        l_old1 = l;
        e_old1 = e;
        break;

    case LC_S_TUS: // lead compensator plus 2 integrators designed in s and
                   // discretized with Tustin method.

        // Error
        e = r - omega;

        // Control Law

        // lead compensator
        l = ((lcs_c_tus_a0 * e + lcs_c_tus_a1 * e_old1)
                - (lcs_c_tus_b1 * l_old1))
            / lcs_c_tus_b0;

        // Double integrator
        u = ((lcs_i_tus_a0 * l + lcs_i_tus_a1 * l_old1 + lcs_i_tus_a2 * l_old2)
                - (lcs_i_tus_b1 * u_old1 + lcs_i_tus_b2 * u_old2))
            / lcs_i_tus_b0;

        // Anti-windup
        if (u > 1) {
            u = 1;
        } else if (u < -1) {
            u = -1;
        }
        // Save past values
        u_old2 = u_old1;
        u_old1 = u;
        l_old2 = l_old1;
        l_old1 = l;
        e_old1 = e;
        break;

    case LC_S_BAP: // lead compensator plus 2 integrators designed in s and
                   // discretized with Backward Approximation method.

        // Error
        e = r - omega;

        // Control Law

        // lead compensator
        l = ((lcs_c_bap_a0 * e + lcs_c_bap_a1 * e_old1)
                - (lcs_c_bap_b1 * l_old1))
            / lcs_c_bap_b0;

        // Double integrator
        u = ((lcs_i_bap_a0 * l + lcs_i_bap_a1 * l_old1 + lcs_i_bap_a2 * l_old2)
                - (lcs_i_bap_b1 * u_old1 + lcs_i_bap_b2 * u_old2))
            / lcs_i_bap_b0;

        // Anti-windup
        if (u > 1) {
            u = 1;
        } else if (u < -1) {
            u = -1;
        }
        // Save past values
        u_old2 = u_old1;
        u_old1 = u;
        l_old2 = l_old1;
        l_old1 = l;
        e_old1 = e;
        break;

    default:
        break;
    }

    omega_old = omega;
    // Set PWM
    set_motor_pwm(u);
    printf("%0.2f, %0.2f, %0.8f\n", r, u, omega);

#endif
// Enable this to log data to stm32monitor
#ifdef LOG_TO_STM32MONITOR
    trigger = 2;
    DumpTrace();
#endif
// Enable this to tune parameters with UART RX
#ifdef PARAMETER_TUNING_MODE
    // UART Rx
    if (UART1_RX_DMA_Ready()) {
        UART1_RX_DMA_Read(rx_buffer);
        string_parser(rx_buffer);
        UART1_RX_DMA_StartReceive();
    }
#endif
}

/**
 * @brief This function not used in this application, is purpose is to test the
 * code in the main.c outside the SysTick interrupt.
 * @param  None
 * @retval None
 */
void user_app_main(void) {
    if (UART1_RX_DMA_Ready()) {
        UART1_RX_DMA_Read(rx_buffer);
        printf(strcat(rx_buffer, "\n"));
        UART1_RX_DMA_StartReceive();
    }
}

/**
 * @brief  This function sets the PWM duty cycle of the motor, the input value
 * is limited to the range [-1, 1].
 * @param  value: PWM duty cycle value.
 * @retval None
 */
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

/**
 * @brief  This function reads the encoder value and calculates the motor
 * angular velocity and position. The angular velocity is calculated in rev/s
 * and stored in the global variable omega, the angular position is calculated
 * in degrees and stored in the global variable theta.
 * @param  None
 * @retval None
 */
void get_motor_speed(void) {

    encoder_value = TIM3->CNT;
    // Motor position in degrees: (counts/counts per rev) * 360(deg/rev)
    theta = 360 * (float)((u_int32_t)encoder_value / COUNTS_PER_REVOLUTION);
    // Motor angular velocity in rev/s: (delta counts/counts per rev) * (1/Ts)
    omega = (int)(encoder_value - encoder_value_old) * SAMPLE_FREQUENCY
        / COUNTS_PER_REVOLUTION;

    // Filter the encoder_value jumps due to the counter overflow
    if ((encoder_value - encoder_value_old) > 1000) {
        omega = omega_old;
    }

    if ((omega >= 50.0f) || (omega <= -50.0f)) {
        omega = omega_old;
    }

    encoder_value_old = encoder_value;
    omega_old = omega;
}

/**
 * @brief  This function parses the input string from UART1 RX DMA, the string
 * is tokenized with the delimiter "," and "=". so the user can send a string
 * like "Kp=0.5,Ki=0.1,min_ref=0.7,max_ref=1.2,ctrl=0,ref=4\n" to change the
 * parameters of the controller, the reference, and the controller type.
 * @param  input: pointer to the input string.
 * @retval None
 */
void string_parser(char* input) {

    char* token;
    char* save_ptr;
    char* save_ptr2;
    // Tokenize the string with the delimiter ","
    token = strtok_r(input, ",", &save_ptr);
    // strcpy(inner_string, token);

    // While for each token pair
    while (token != NULL) {
        // Tokenize the string with the delimiter "="
        char* name = strtok_r(token, "=", &save_ptr2);
        char* value = strtok_r(NULL, "=", &save_ptr2);
        if (name != NULL && value != NULL) {
            if (!strcmp(name, "ctrl_type")) {
                selected_controller = (enum controller_type)(atoi(value));
            } else if (!strcmp(name, "ref_type")) {
                selected_reference = (enum reference_type)(atoi(value));
            } else if (!strcmp(name, "ref_period")) {
                ref_period_seconds = (atoi(value));
                ref_period_samples = (uint16_t)(ref_period_seconds / Ts);
            } else if (!strcmp(name, "min_ref")) {
                min_ref = atof(value);
            } else if (!strcmp(name, "max_ref")) {
                max_ref = atof(value);
            }
            token = strtok_r(NULL, ",", &save_ptr);
        }
    }
}