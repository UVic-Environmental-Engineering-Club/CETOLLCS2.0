/*
 * roll_motor.h
 *
 *  Created on: Sept 10, 2026
 *      Author: igorc
 */

#ifndef INC_ROLL_MOTOR_H_
#define INC_ROLL_MOTOR_H_

#include "main.h"
#include <stdlib.h>
#include "utils.h"

// Roll Motor Types	  ----------------------------------------------------------------------------
typedef uint16_t roll_dma_type;
typedef int roll_degree_unit; 

// PWM inlines 			  ----------------------------------------------------------------------------
#define ROLL_PWM_TIMER htim9
#define ROLL_PWM_TIMER_CHANNEL TIM_CHANNEL_2
#define ROLL_PWM_STEPS (__HAL_TIM_GET_AUTORELOAD(&ROLL_PWM_TIMER) + 1) // ARR + 1 (resolution)

// Direction inlines  ----------------------------------------------------------------------------
#define ROLL_START_DIRECTION MOTOR_DIRECTION_CW

// Pot inlines			  ----------------------------------------------------------------------------
#define ROLL_POT_SAMPLE_SIZE 7U //how many samples for median

// Duty cycle inlines	----------------------------------------------------------------------------
#define ROLL_MAX_DUTY_CYCLE 0.5f
#define ROLL_SLOWED_DUTY_CYCLE 0.25f
#define ROLL_SLOW_DOWN_DIFF 10U
#define ROLL_POSITION_ERROR 2U

// Global variables   ----------------------------------------------------------------------------
extern TIM_HandleTypeDef htim9;
extern ADC_HandleTypeDef hadc3;

// Functions			    ----------------------------------------------------------------------------
/*
 * @brief initialize motor (pwm, direction, pot)
 * @param none
 * @retval rm_return enum
 */
motor_result_t roll_init_motor(void);

/*
 * @brief wrapper running rm_set_duty_cycle(0)
 * @param none
 * @retval motor_result_t enum of running rm_set_duty_cycle(0)
 */
motor_result_t roll_stop(void);

/*
 * @brief set stm32 duty cycle
 * @param duty_cycle: desired duty cycle for pwm motor *RANGE: (0-1)*
 * @retval motor_result_t enum
 */
motor_result_t roll_set_duty_cycle(float duty_cycle);

/*
 * @brief read the pot DMA as a median of values
 * @param result int to store result
 * @retval motor_result_t enum
 */
motor_result_t roll_read_pot_safe(roll_dma_type* result);

/*
 * @brief runs fn rm_read_pot_safe and returns result as degrees
 * @param result int to store result
 * @retval motor_result_t enum
 */
motor_result_t roll_get_current_degree(roll_degree_unit* result);

/*
 * @brief getter for the current state of the roll motor
 * @param none 
 * @retval motor_states_t
 */
motor_states_t roll_get_state(void);

/*
 * @brief transitions into recovery
 * @param fault type
 * @retval none
 */
void roll_enter_recovery(motor_result_t fault_type); //not implemented

/*
 * @brief transitions into emergency state
 * @param none
 * @retval none
 */
void roll_enter_emergency(); //not implememented

/*
 * @brief moves motor to a provided target degree
 * @param target degree
 * @retval motor_result_t
 */
motor_result_t roll_move_to(roll_degree_unit target_degree);

#endif /* INC_ROLL_MOTOR_H_ */
