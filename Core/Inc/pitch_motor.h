/*
 * pitch_motor.h
 *
 * Blocking, RTOS-friendly controller for the pitch linear stepper motor.
 */

#ifndef INC_PITCH_MOTOR_H_
#define INC_PITCH_MOTOR_H_

#include "main.h"
#include "utils.h"
#include <stdint.h>

// Pitch motor types --------------------------------------------------------*/

typedef int32_t pitch_distance_unit; //mm
typedef int32_t pitch_step_unit;

// Timer inlines ------------------------------------------------------------
#define PITCH_STEP_TIMER htim2
#define PITCH_STEP_TIMER_CHANNEL TIM_CHANNEL_1

// Mechanical inlines -------------------------------------------------------
#define PITCH_FULL_STEPS_PER_REVOLUTION 200U /* 1.8 degree motor. */
#define PITCH_MICROSTEP_RATIO 8U             /* Must match the driver switches. */
#define PITCH_LEAD_MM_PER_REVOLUTION 8U      /* Tr8x8 lead screw. */
#define PITCH_STEPS_PER_MM ((PITCH_FULL_STEPS_PER_REVOLUTION * PITCH_MICROSTEP_RATIO) / PITCH_LEAD_MM_PER_REVOLUTION)

// Position inlines ---------------------------------------------------------*/
//code is assuming home is 0 and motor moves away from that
#define PITCH_MIN_POSITION_MM 6
#define PITCH_MAX_POSITION_MM 91 //112

// Motion profile inlines ---------------------------------------------------*/
#define PITCH_HOME_STEP_FREQUENCY_HZ 5000U // 5 mm/s at 200 microsteps/mm
#define PITCH_MIN_STEP_FREQUENCY_HZ 1500U   // 2.5 mm/s
#define PITCH_MAX_STEP_FREQUENCY_HZ 5000U  // 15 mm/s
#define PITCH_RAMP_DISTANCE_MM 20U // 20mm is the maximum ramp up/down distance
#define PITCH_CONTROL_DELAY_MS 1U 
#define PITCH_HOME_DEBOUNCE_MS 5U

// GPIO polarity inlines ----------------------------------------------------*/
//NEED TO BE VERIFIED
#define PITCH_FORWARD_PIN_STATE GPIO_PIN_SET
#define PITCH_BACKWARD_PIN_STATE GPIO_PIN_RESET
#define PITCH_HOME_DIRECTION MOTOR_DIRECTION_BACKWARD

// Global variables ---------------------------------------------------------
extern TIM_HandleTypeDef htim2;

// Functions ----------------------------------------------------------------
/*
 * @brief initialize motor (turn on, run homing)
 * @param none
 * @retval motor_result_t enum
 */
motor_result_t pitch_init_motor(void);

/*
 * @brief stops motor with interupt
 * @param none
 * @retval motor_result_t enum
 */
motor_result_t pitch_stop(void);

/*
 * @brief sets the step pulse frequency as value provided
 * @param uint32_t desired frequency (between set max/min)
 * @retval motor_result_t enum
 */
motor_result_t pitch_set_step_frequency(uint32_t frequency_hz);

/*
 * @brief run homing procedure
 * @param none
 * @retval motor_result_t enum
 */
motor_result_t pitch_home(void);

/*
 * @brief getter for the current position of motor (mm)
 * @param result pointer for storage
 * @retval motor_result_t enum
 */
motor_result_t pitch_get_current_position(pitch_distance_unit *result);

/*
 * @brief getter for the current state of motor 
 * @param none
 * @retval motor_states_t enum
 */
motor_states_t pitch_get_state(void);

/*
 * @brief motor enters recovery state
 * @param motor_result_t fault_type
 * @retval none
 */
void pitch_enter_recovery(motor_result_t fault_type);

/*
 * @brief motor enters emergency state
 * @param none
 * @retval none
 */
void pitch_enter_emergency();

/*
 * @brief moves motor to a provided target distance away from home (mm)
 * @param none
 * @retval motor_result_t enum
 */
motor_result_t pitch_move_to(pitch_distance_unit target_position_mm);

/*
 * @brief initialize motor (pwm, direction, pot)
 * @param none
 * @retval rm_return enum
 */
void pitch_handle_pwm_pulse_finished(TIM_HandleTypeDef *htim);

#endif /* INC_PITCH_MOTOR_H_ */
