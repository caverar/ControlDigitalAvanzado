#ifndef __USER_APP_H__
#define __USER_APP_H__

#ifdef __cplusplus
extern "C" {
#endif

// Includes *******************************************************************

// Standard libraries ---------------------------------------------------------
#include <stdbool.h>
#include <stdint.h>
// BSP libraries --------------------------------------------------------------
// User libraries -------------------------------------------------------------
#include "user_uart1.h"

// Private defines ************************************************************

// Execution Mode selection (exclusive) ---------------------------------------
// #define IDENT_MODE // Uncomment to run the identification mode.
#define CONTROL_MODE // Uncomment to run the control mode.
// Extra features -------------------------------------------------------------
#define LOG_TO_STM32MONITOR // Uncomment to log data to stm32monitor.
#define PARAMETER_TUNING_MODE // Uncomment to allow parameter tuning by UART.
// Encoder parameters ---------------------------------------------------------
#define GEAR_RATIO 20 // Gear ratio output shaft and encoder.
#define ENCODER_RESOLUTION 1440 // 1440 counts per revolution.
#define COUNTS_PER_REVOLUTION (float)(GEAR_RATIO * ENCODER_RESOLUTION)
#define SAMPLE_TIME 0.005f // 5ms
#define Ts SAMPLE_TIME

// Private variables **********************************************************

char rx_buffer[UART1_RX_BUFFER_LEN]; // Buffer for RX UART1 DMA.

// Process variables ----------------------------------------------------------
float omega; // Current Motor angular velocity.
float omega_old; // Past Motor angular velocity.
float theta; // Current Motor angular position.
uint16_t encoder_value; // Current encoder count value.
uint16_t encoder_value_old; // Past encoder count value.
uint8_t trigger; // Trigger for stm32monitor.

// Reference parameters -------------------------------------------------------
enum reference_type { // Waveform selector.
    SQUARE = 0, // Square reference.
    TRIANGULAR = 1, // Triangular reference.
} selected_reference; // Reference selector.
uint16_t k; // Iterator for the Reference.
float r; // Reference
float min_ref; // Minimum reference.
float max_ref; // Maximum reference.
float ref_period_seconds; // Reference period in seconds.
uint16_t ref_period_samples; // Reference period in samples of Ts

// Controllers parameters -----------------------------------------------------
enum controller_type { // Controller selector.
    PI_ZOH = 0, // PI designed in s and discretized with ZOH.
    PI_FOH = 1, // PI designed in s and discretized with FOH.
    PI_TUS = 2, // PI designed in s and discretized with Tustin.
    LC_W = 3, // 2 lead compensators plus 2 integrators designed in w domain.
    LC_S_TUS = 4 // Lead compensator plus 2 integrators designed in s (Tustin).
} selected_controller; // Control selector
float u, u_old1, u_old2, u_old3, u_old4; // Control action and past samples.
float e, e_old1, e_old2, e_old3, e_old4; // Error and past samples.

// Control parameters for PI_ZOH ----------------------------------------------
float Kp; // Proportional gain
float Ki; // Integral gain

// Control parameters for LC_W ------------------------------------------------
float lcw_c1_a0, lcw_c1_a1; // lead compensator 1 numerator coefficients.
float lcw_c1_b0, lcw_c1_b1; // lead compensator 1 denominator coefficients.
float l1, l1_old1, l1_old2; // lead compensator 1 output.
float lcw_c2_a0, lcw_c2_a1; // lead compensator 2 numerator coefficients.
float lcw_c2_b0, lcw_c2_b1; // lead compensator 2 denominator coefficients.
float l2, l2_old1, l2_old2; // lead compensator 2 output.
float lcw_i_a0, lcw_i_a1, lcw_i_a2; // integrator numerator coefficients.
float lcw_i_b0, lcw_i_b1, lcw_i_b2; // integrator denominator coefficients.

// Control parameters for LC_S_FOH --------------------------------------------
float lcs_c_a0, lcs_c_a1; // lead compensator numerator coefficients.
float lcs_c_b0, lcs_c_b1; // lead compensator denominator coefficients.
float l, l_old1, l_old2; // lead compensator output.
float lcs_i_a0, lcs_i_a1, lcs_i_a2; // integrator numerator coefficients.
float lcs_i_b0, lcs_i_b1, lcs_i_b2; // integrator denominator coefficients.

// Function prototypes ********************************************************

void user_app_init(void);
void user_app_interrupt(void);
void user_app_main(void);
void string_parser(char* input);
void set_motor_pwm(float value);
void get_motor_speed(void);

#ifdef __cplusplus
}
#endif

#endif /*__ USER_APP_H__ */