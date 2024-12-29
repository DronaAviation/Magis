/*PlutoX API V.1.0
 */


#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum reverse_motor {
  M1 = 0x0,
  M2,
  M3,
  M4
} reverse_motor_e;

typedef enum motor_direction {
  CLOCK_WISE = 0,
  ANTICLOCK_WISE
} motor_direction_e;

typedef enum {
  FORWARD = 0,
  BACKWARD
} motor_terrestrial_direction_e;

typedef enum std_motor {
  M5 = 0x0,
  M6,
  M7,
  M8
} std_motor_e;

class Motor_P {

 public:
  void initReverseMotor ( reverse_motor_e motor );
  void set ( std_motor_e motor, int16_t pwmValue );
  void set ( reverse_motor_e motor, int16_t pwmValue );
  void setDirection ( reverse_motor_e motor, motor_direction_e direction );
};

extern Motor_P Motor;

#ifdef __cplusplus
}
#endif
