#ifndef __USER_APP_H__
#define __USER_APP_H__

#ifdef __cplusplus
extern "C" {
#endif

// Standard libraries
#include <stdbool.h>
#include <stdint.h>
// BSP libraries
// User libraries
#include "user_uart1.h"

// Private defines

// Execution Mode selection (exclusive)
// #define IDENT_MODE // Uncomment to run the identification mode
#define CONTROL_MODE // Uncomment to run the control mode

// extra features
#define LOG_TO_STM32MONITOR // Uncomment to log data to stm32monitor
#define PARAMETER_TUNING_MODE // Uncomment to allow parameter tuning by UART

// Encoder parameters
#define GEAR_RATIO 20
#define ENCODER_RESOLUTION 1440
#define COUNTS_PER_REVOLUTION (float)(GEAR_RATIO * ENCODER_RESOLUTION)
#define SAMPLE_TIME 0.005f // 5ms
#define Ts SAMPLE_TIME

// Private variables
char rx_buffer[UART1_RX_BUFFER_LEN]; // Buffer for RX UART1 DMA
// Process variables
uint16_t k; // Iterator for the Control
float omega; // Current Motor angular velocity
float omega_old; // Past Motor angular velocity
float theta; // Current Motor angular position
uint16_t encoder_value; // Current encoder count value
uint16_t encoder_value_old; // Past encoder count value
uint8_t trigger; // Trigger for stm32monitor
uint8_t sel_ctrl;

// Control parameters (General)
float r; // Reference
float min_ref; // Minimum reference
float max_ref; // Maximum reference
float u; // Control action over motor
float u_old1; // Past control action
float u_old2; // Past past control action
float u_old3; // Past past past control action
float u_old4; // Past past past past control action
float e; // Error
float e_old1; // Past error
float e_old2; // Past past error
float e_old3; // Past past past error
float e_old4; // Past past past past error

// Control parameters (PI)
float Kp; // Proportional gain
float Ki; // Integral gain

// Control parameters (2xLead compensator in w)
float a0, a1, a2, a3, a4; // numerator coefficients
float b0, b1, b2, b3, b4; // denominator coefficients
// Function prototypes
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