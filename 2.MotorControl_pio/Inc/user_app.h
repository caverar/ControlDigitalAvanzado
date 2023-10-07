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

// Private defines
#define IDENT_MODE
// #define VALIDATION_DATA_MODE
#define GEAR_RATIO 20
#define ENCODER_RESOLUTION 1440
#define COUNTS_PER_REVOLUTION (float)(GEAR_RATIO * ENCODER_RESOLUTION)
// Private variables

uint16_t k; // Iterator for the Control
float omega; // Current Motor angular velocity
float omega_old; // Past Motor angular velocity
float theta; // Current Motor angular position
uint16_t encoder_value; // Current encoder count value
uint16_t encoder_value_old; // Past encoder count value
uint8_t trigger; // Trigger for stm32monitor
float u; // Control action over motor

// Function prototypes
void user_app_init(void);
void user_app_interrupt(void);
void user_app_main(void);

void set_motor_pwm(float value);
void get_motor_speed(void);

#ifdef __cplusplus
}
#endif

#endif /*__ USER_APP_H__ */