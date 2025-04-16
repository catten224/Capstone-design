/*
 * B_Encoder.c
 *
 *  Created on: Mar 19, 2025
 *      Author: taegy
 */

#include "B_Encoder.h"

void B_MOTOR_SET_STATE(B_MOTOR_t *motor, MotorPowerState_t state)
{
  if (motor == NULL) return;

  if (state == MOTOR_STATE_START)
  {
    HAL_GPIO_WritePin(L_START_PORT, L_START_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(R_START_PORT, R_START_PIN, GPIO_PIN_SET);
  }
  else if (state == MOTOR_STATE_STOP)
  {
    HAL_GPIO_WritePin(L_START_PORT, L_START_PIN, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(L_PWM_TIMER, L_PWM_CHANNEL, 0);

    HAL_GPIO_WritePin(R_START_PORT, R_START_PIN, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(R_PWM_TIMER, R_PWM_CHANNEL, 0);
  }
}

MotorPowerState_t B_MOTOR_GET_STATE(B_MOTOR_t *motor)
{
  if (motor == NULL) return MOTOR_STATE_ERROR;

  GPIO_PinState L = HAL_GPIO_ReadPin(L_START_PORT, L_START_PIN);
  GPIO_PinState R = HAL_GPIO_ReadPin(R_START_PORT, R_START_PIN);

  if (L == GPIO_PIN_SET && R == GPIO_PIN_SET)
    return MOTOR_STATE_START;
  else if (L == GPIO_PIN_RESET && R == GPIO_PIN_RESET)
    return MOTOR_STATE_STOP;
  else
    return MOTOR_STATE_ERROR;
}

void B_MOTOR_SET_DIR(B_MOTOR_t *motor, MotorDirection_t dir)
{
  if (motor == NULL) return;

  if (dir == MOTOR_DIR_CW) {
    HAL_GPIO_WritePin(L_DIR_PORT, L_DIR_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(R_DIR_PORT, R_DIR_PIN, GPIO_PIN_SET);
  }
  else if (dir == MOTOR_DIR_CCW) {
    HAL_GPIO_WritePin(L_DIR_PORT, L_DIR_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(R_DIR_PORT, R_DIR_PIN, GPIO_PIN_RESET);
  }
}

MotorDirection_t B_MOTOR_GET_DIR(B_MOTOR_t *motor)
{
  if (motor == NULL) return MOTOR_DIR_ERROR;

  GPIO_PinState L = HAL_GPIO_ReadPin(L_DIR_PORT, L_DIR_PIN);
  GPIO_PinState R = HAL_GPIO_ReadPin(R_DIR_PORT, R_DIR_PIN);

  if (L == GPIO_PIN_RESET && R == GPIO_PIN_SET)
    return MOTOR_DIR_CW;
  else if (L == GPIO_PIN_SET && R == GPIO_PIN_RESET)
    return MOTOR_DIR_CCW;
  else
    return MOTOR_DIR_ERROR;
}

void B_MOTOR_SET_PWM_DUTYCYCLE(B_MOTOR_t *motor, uint16_t duty_cycle, MotorSide_t side)
{
  if (motor == NULL) return;

  duty_cycle = abs(duty_cycle);
  if (duty_cycle > MAX_PWM_DUTYCYCLE) duty_cycle = MAX_PWM_DUTYCYCLE;
  if (duty_cycle < MIN_PWM_DUTYCYCLE) duty_cycle = MIN_PWM_DUTYCYCLE;

  uint16_t ARR;
  if (side == MOTOR_L)
    ARR = __HAL_TIM_GET_AUTORELOAD(L_PWM_TIMER);
  else
    ARR = __HAL_TIM_GET_AUTORELOAD(R_PWM_TIMER);

  uint16_t CCR = ((ARR + 1) * duty_cycle) / 100;

  if (side == MOTOR_L)
    __HAL_TIM_SET_COMPARE(L_PWM_TIMER, L_PWM_CHANNEL, CCR);
  else
    __HAL_TIM_SET_COMPARE(R_PWM_TIMER, R_PWM_CHANNEL, CCR);
}

uint8_t B_MOTOR_GET_PWM_DUTYCYCLE(B_MOTOR_t *motor, MotorSide_t side)
{
  if (motor == NULL) return 0;

  uint16_t ARR, CCR;

  if (side == MOTOR_L) {
    ARR = __HAL_TIM_GET_AUTORELOAD(L_PWM_TIMER);
    CCR = __HAL_TIM_GET_COMPARE(L_PWM_TIMER, L_PWM_CHANNEL);
  }
  else {
    ARR = __HAL_TIM_GET_AUTORELOAD(R_PWM_TIMER);
    CCR = __HAL_TIM_GET_COMPARE(R_PWM_TIMER, R_PWM_CHANNEL);
  }

  return (uint8_t)(((uint32_t)CCR * 100) / (ARR + 1));
}

void B_MOTOR_INIT(B_MOTOR_t *motor)
{
  if (motor == NULL) return;

  B_MOTOR_SET_DIR(motor, MOTOR_DIR_CW);
  B_MOTOR_SET_STATE(motor, MOTOR_STATE_STOP);
  motor->move_state = MOTOR_MOVE_IDLE;

  __HAL_TIM_SET_COMPARE(L_PWM_TIMER, L_PWM_CHANNEL, 0);
  __HAL_TIM_SET_COMPARE(R_PWM_TIMER, R_PWM_CHANNEL, 0);

  __HAL_TIM_SET_COUNTER(L_ENCODER_TIMER, 0);
  __HAL_TIM_SET_COUNTER(R_ENCODER_TIMER, 0);
  __HAL_TIM_CLEAR_FLAG(L_ENCODER_TIMER, TIM_FLAG_UPDATE);
  __HAL_TIM_CLEAR_FLAG(R_ENCODER_TIMER, TIM_FLAG_UPDATE);

  HAL_TIM_Encoder_Start_IT(L_ENCODER_TIMER, L_ENCODER_CHANNEL);
  HAL_TIM_Encoder_Start_IT(R_ENCODER_TIMER, R_ENCODER_CHANNEL);
  HAL_TIM_PWM_Start(L_PWM_TIMER, L_PWM_CHANNEL);
  HAL_TIM_PWM_Start(R_PWM_TIMER, R_PWM_CHANNEL);
}

void B_MOTOR_EncoderOver(B_MOTOR_t *motor)
{
  if (motor == NULL) return;

  if (__HAL_TIM_GET_FLAG(L_ENCODER_TIMER, TIM_FLAG_UPDATE)) {
    if (__HAL_TIM_IS_TIM_COUNTING_DOWN(L_ENCODER_TIMER))
      motor->L.over_under_count--;
    else
      motor->L.over_under_count++;
    __HAL_TIM_CLEAR_FLAG(L_ENCODER_TIMER, TIM_FLAG_UPDATE);
  }

  if (__HAL_TIM_GET_FLAG(R_ENCODER_TIMER, TIM_FLAG_UPDATE)) {
    if (__HAL_TIM_IS_TIM_COUNTING_DOWN(R_ENCODER_TIMER))
      motor->R.over_under_count--;
    else
      motor->R.over_under_count++;
    __HAL_TIM_CLEAR_FLAG(R_ENCODER_TIMER, TIM_FLAG_UPDATE);
  }
}

void B_MOTOR_UpdateEncoder(B_MOTOR_t *motor)
{
  if (motor == NULL) return;

  motor->L.current_count = __HAL_TIM_GET_COUNTER(L_ENCODER_TIMER);
  motor->L.total_count = motor->L.current_count + (motor->L.over_under_count * 65536);

  motor->R.current_count = __HAL_TIM_GET_COUNTER(R_ENCODER_TIMER);
  motor->R.total_count = motor->R.current_count + (motor->R.over_under_count * 65536);
}

void B_MOTOR_ACCEL(B_MOTOR_t *motor)
{
  if (motor == NULL) return;

  uint8_t pwm_L = B_MOTOR_GET_PWM_DUTYCYCLE(motor, MOTOR_L);
  uint8_t pwm_R = B_MOTOR_GET_PWM_DUTYCYCLE(motor, MOTOR_R);

  if (pwm_L < MAX_PWM_DUTYCYCLE_ACCEL)
    B_MOTOR_SET_PWM_DUTYCYCLE(motor, pwm_L + 1, MOTOR_L);

  if (pwm_R < MAX_PWM_DUTYCYCLE_ACCEL)
    B_MOTOR_SET_PWM_DUTYCYCLE(motor, pwm_R + 1, MOTOR_R);

  if (pwm_L >= MAX_PWM_DUTYCYCLE_ACCEL && pwm_R >= MAX_PWM_DUTYCYCLE_ACCEL)
    motor->move_state = MOTOR_MOVE_PID;
}

void B_MOTOR_BRAKE(B_MOTOR_t *motor)
{
  if (motor == NULL) return;

  uint8_t pwm_L = B_MOTOR_GET_PWM_DUTYCYCLE(motor, MOTOR_L);
  uint8_t pwm_R = B_MOTOR_GET_PWM_DUTYCYCLE(motor, MOTOR_R);

  if (pwm_L > MIN_PWM_DUTYCYCLE)
    B_MOTOR_SET_PWM_DUTYCYCLE(motor, pwm_L - 1, MOTOR_L);

  if (pwm_R > MIN_PWM_DUTYCYCLE)
    B_MOTOR_SET_PWM_DUTYCYCLE(motor, pwm_R - 1, MOTOR_R);

  if (pwm_L <= MIN_PWM_DUTYCYCLE && pwm_R <= MIN_PWM_DUTYCYCLE)
  {
    B_MOTOR_SET_STATE(motor, MOTOR_STATE_STOP);
    motor->move_state = MOTOR_MOVE_DONE;  // 감속 후 완료 상태 전환
  }
}

uint32_t B_MOTOR_cell_to_cm(uint8_t target_cell)
{
  switch (target_cell) {
    case 0: return 0;
    case 1: return 40;
    case 2: return 80;
    case 3: return 120;
    default: return 0;
  }
}

void B_MOTOR_PID_CONTROLLER(B_MOTOR_t *motor)
{
  if (motor == NULL) return;

#if 1
  PIDController_t *pid = &motor->pid_R;

  // 1. 에러 계산
  pid->error = motor->target_count - motor->R.total_count;

  // 2. 정지 조건
  if (abs(pid->error) <= 50) {
      B_MOTOR_SET_STATE(motor, MOTOR_STATE_STOP);
      motor->move_state = MOTOR_MOVE_DONE;
      return;
  }

  // 3. PID 계산
  pid->error_integral += pid->error;

  if (pid->error_integral > 5000000) pid->error_integral = 5000000;
  if (pid->error_integral < -5000000) pid->error_integral = -5000000;

  pid->error_derivative = pid->error - pid->error_prev;
  pid->error_prev = pid->error;

  pid->output = (pid->Kp * pid->error)
              + (pid->Ki * pid->error_integral)
              + (pid->Kd * pid->error_derivative);

  if (pid->output < 0)
      pid->output = -pid->output;

  B_MOTOR_SET_PWM_DUTYCYCLE(motor, (uint16_t)pid->output, MOTOR_L);
  B_MOTOR_SET_PWM_DUTYCYCLE(motor, (uint16_t)pid->output, MOTOR_R);
#endif

#if 0
  PIDController_t *pid_L = &motor->pid_L;
  PIDController_t *pid_R = &motor->pid_R;

  // 1. 에러 계산
  pid_L->error = motor->target_count - motor->L.total_count;
  pid_R->error = motor->target_count - motor->R.total_count;

  // 2. 정지 조건 (좌우 모두 범위 이내일 때)
  if (abs(pid_L->error) <= 50 && abs(pid_R->error) <= 50) {
      B_MOTOR_SET_STATE(motor, MOTOR_STATE_STOP);
      motor->move_state = MOTOR_MOVE_DONE;
      return;
  }

  // 3. L PID 계산
  pid_L->error_integral += pid_L->error;
  if (pid_L->error_integral > 5000000) pid_L->error_integral = 5000000;
  if (pid_L->error_integral < -5000000) pid_L->error_integral = -5000000;
  pid_L->error_derivative = pid_L->error - pid_L->error_prev;
  pid_L->error_prev = pid_L->error;
  pid_L->output = (pid_L->Kp * pid_L->error) +
                  (pid_L->Ki * pid_L->error_integral) +
                  (pid_L->Kd * pid_L->error_derivative);

  // 4. R PID 계산
  pid_R->error_integral += pid_R->error;
  if (pid_R->error_integral > 5000000) pid_R->error_integral = 5000000;
  if (pid_R->error_integral < -5000000) pid_R->error_integral = -5000000;
  pid_R->error_derivative = pid_R->error - pid_R->error_prev;
  pid_R->error_prev = pid_R->error;
  pid_R->output = (pid_R->Kp * pid_R->error) +
                  (pid_R->Ki * pid_R->error_integral) +
                  (pid_R->Kd * pid_R->error_derivative);

  // 5. 부호 보정 (모터 PWM은 항상 양수로)
  if (pid_L->output < 0) pid_L->output = -pid_L->output;
  if (pid_R->output < 0) pid_R->output = -pid_R->output;

  // 6. PWM 출력
  B_MOTOR_SET_PWM_DUTYCYCLE(motor, (uint16_t)pid_L->output, MOTOR_L);
  B_MOTOR_SET_PWM_DUTYCYCLE(motor, (uint16_t)pid_R->output, MOTOR_R);
#endif
}

void B_MOTOR_RESET_PID(B_MOTOR_t *motor)
{
  if (motor == NULL) return;

  motor->pid_L.error = 0;
  motor->pid_L.error_prev = 0;
  motor->pid_L.error_integral = 0;
  motor->pid_L.error_derivative = 0;
  motor->pid_L.output = 0;

  motor->pid_R.error = 0;
  motor->pid_R.error_prev = 0;
  motor->pid_R.error_integral = 0;
  motor->pid_R.error_derivative = 0;
  motor->pid_R.output = 0;
}
