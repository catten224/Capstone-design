/*
 * B_Encoder.h
 *
 *  Created on: Mar 19, 2025
 *      Author: taegy
 */

#ifndef INC_B_ENCODER_H_
#define INC_B_ENCODER_H_
#include "main.h"
#include "math.h"
#include "stdlib.h"

typedef enum {
  MOTOR_L,
  MOTOR_R
} MotorSide_t;

typedef enum {
  MOTOR_STATE_START,
  MOTOR_STATE_STOP,
  MOTOR_STATE_ERROR
} MotorPowerState_t;

typedef enum {
  MOTOR_DIR_CW,
  MOTOR_DIR_CCW,
  MOTOR_DIR_ERROR
} MotorDirection_t;

typedef enum {
  MOTOR_MOVE_IDLE,
  MOTOR_MOVE_DIR,
  MOTOR_MOVE_ACCEL,
  MOTOR_MOVE_PID,
  MOTOR_MOVE_ERROR,
  MOTOR_MOVE_DONE
} MotorMoveState_t;

/* **************************************************** */
/*                   MOTOR PARAMETERS                   */
/* **************************************************** */
#define MAX_PWM_DUTYCYCLE         60
#define MAX_PWM_DUTYCYCLE_ACCEL   60
#define MIN_PWM_DUTYCYCLE         10

#define WHEEL_DIAMETER_CM         10.0
#define WHEEL_CIRCUMFERENCE_CM    (WHEEL_DIAMETER_CM * M_PI)

#define MOTOR_CPR                 380
#define MOTOR_GEAR_RATIO          27
#define MOTOR_COUNTS_PER_REV      ((MOTOR_CPR) * 4 * (MOTOR_GEAR_RATIO))
#define COUNTS_PER_CM 			  ((uint32_t)(MOTOR_COUNTS_PER_REV / WHEEL_CIRCUMFERENCE_CM + 0.5))

/* **************************************************** */
/*                       L MOTOR                        */
/* **************************************************** */
#define L_DIR_PORT        GPIOD
#define L_DIR_PIN         GPIO_PIN_14
#define L_START_PORT      GPIOD
#define L_START_PIN       GPIO_PIN_15
#define L_PWM_TIMER       &htim8
#define L_PWM_CHANNEL     TIM_CHANNEL_1
#define L_ENCODER_TIMER   &htim1
#define L_ENCODER_CHANNEL TIM_CHANNEL_ALL

/* **************************************************** */
/*                       R MOTOR                        */
/* **************************************************** */
#define R_DIR_PORT        GPIOF
#define R_DIR_PIN         GPIO_PIN_12
#define R_START_PORT      GPIOF
#define R_START_PIN       GPIO_PIN_13
#define R_PWM_TIMER       &htim8
#define R_PWM_CHANNEL     TIM_CHANNEL_2
#define R_ENCODER_TIMER   &htim3
#define R_ENCODER_CHANNEL TIM_CHANNEL_ALL

/* **************************************************** */
/*                       ENCODER                        */
/* **************************************************** */
typedef struct {
    int32_t current_count;
    int16_t over_under_count;
    int32_t total_count;
} MotorEncoder_t;

/* **************************************************** */
/*                          PID                         */
/* **************************************************** */
typedef struct {
    int32_t error;
    int32_t error_prev;
    int32_t error_integral;
    int32_t error_derivative;

    double Kp;
    double Ki;
    double Kd;
    double output;
} PIDController_t;

/* **************************************************** */
/*                     MOTOR CONTROL                    */
/* **************************************************** */
typedef struct {
    MotorEncoder_t L;
    MotorEncoder_t R;

    PIDController_t pid_L;
    PIDController_t pid_R;

    int32_t target_count;

    MotorPowerState_t power_state;
    MotorDirection_t dir_state;
    MotorMoveState_t move_state;
} B_MOTOR_t;


void B_MOTOR_SET_STATE(B_MOTOR_t *motor, MotorPowerState_t state);
MotorPowerState_t B_MOTOR_GET_STATE(B_MOTOR_t *motor);


void B_MOTOR_SET_DIR(B_MOTOR_t *motor, MotorDirection_t dir);
MotorDirection_t B_MOTOR_GET_DIR(B_MOTOR_t *motor);


void B_MOTOR_SET_PWM_DUTYCYCLE(B_MOTOR_t *motor, uint16_t duty_cycle, MotorSide_t side);
uint8_t B_MOTOR_GET_PWM_DUTYCYCLE(B_MOTOR_t *motor, MotorSide_t side);


void B_MOTOR_INIT(B_MOTOR_t *motor);


void B_MOTOR_EncoderOver(B_MOTOR_t *motor);
void B_MOTOR_UpdateEncoder(B_MOTOR_t *motor);


void B_MOTOR_ACCEL(B_MOTOR_t *motor);
void B_MOTOR_BRAKE(B_MOTOR_t *motor);


uint32_t B_MOTOR_cell_to_cm(uint8_t target_cell);

void B_MOTOR_RESET_PID(B_MOTOR_t *motor);
void B_MOTOR_PID_CONTROLLER(B_MOTOR_t *motor);
#endif /* INC_B_ENCODER_H_ */
