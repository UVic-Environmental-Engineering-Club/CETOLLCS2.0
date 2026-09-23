/*
 * pitch_motor.c
 *
 * Blocking, RTOS-friendly controller for the pitch linear stepper motor.
 */

#include "pitch_motor.h"
#include "cmsis_os.h"
#include <stdbool.h>
#include <stdlib.h>

// Private variables --------------------------------------------------------
static volatile pitch_step_unit pitch_current_steps = 0;
static volatile pitch_step_unit pitch_target_steps = 0;
static volatile motor_direction_t pitch_direction = MOTOR_DIRECTION_FORWARD;
static volatile bool pitch_pwm_running = false;
static volatile bool pitch_target_reached = false;
static volatile bool pitch_homing = false;
static volatile motor_states_t pitch_state = MOTOR_STATE_UNINITIALIZED;

// Private function prototypes ---------------------------------------------
static motor_result_t pitch_set_direction(motor_direction_t direction);
static bool pitch_check_home_switch(void);
static pitch_step_unit pitch_mm_to_steps(pitch_distance_unit distance_mm);
static uint32_t pitch_get_timer_clock_hz(void);
static uint32_t pitch_calculate_step_frequency(
    pitch_step_unit moved_steps,
    pitch_step_unit remaining_steps
);
static motor_result_t pitch_start_pulses(void);

// Private functions --------------------------------------------------------
static motor_result_t pitch_set_direction(motor_direction_t direction){
    switch (direction)
    {
        case MOTOR_DIRECTION_FORWARD:
            HAL_GPIO_WritePin(
                Pitch_DIR_GPIO_Port,
                Pitch_DIR_Pin,
                PITCH_FORWARD_PIN_STATE
            );
            break;

        case MOTOR_DIRECTION_BACKWARD:
            HAL_GPIO_WritePin(
                Pitch_DIR_GPIO_Port,
                Pitch_DIR_Pin,
                PITCH_BACKWARD_PIN_STATE
            );
            break;

        default:
            return MOTOR_STATUS_INVALID_INPUT;
    }

    pitch_direction = direction;
    return MOTOR_STATUS_OK;
}

static void pitch_turn_on(void){ //set enabled pin
	HAL_GPIO_WritePin(Pitch_EN_GPIO_Port, Pitch_EN_Pin, GPIO_PIN_RESET);
}
static void pitch_turn_off(void){ //reset enabled pin
	 HAL_GPIO_WritePin(Pitch_EN_GPIO_Port, Pitch_EN_Pin, GPIO_PIN_SET);
}

static bool pitch_check_home_switch(void){ //check if the home switch is on or off VERIFY POLARITY
    return HAL_GPIO_ReadPin(Pitch_SW_GPIO_Port, Pitch_SW_Pin) == GPIO_PIN_SET;
}

static pitch_step_unit pitch_mm_to_steps(pitch_distance_unit distance_mm){
    return distance_mm * (pitch_step_unit)PITCH_STEPS_PER_MM;
}

static uint32_t pitch_get_timer_clock_hz(void){
    uint32_t timer_clock_hz = HAL_RCC_GetPCLK1Freq();

    //STM32F4 timers receive 2 x PCLK when the APB prescaler is not 1
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0U){
        timer_clock_hz *= 2U;
    }

    return timer_clock_hz;
}

//decides the frequency for acceleration/deceleration 
static uint32_t pitch_calculate_step_frequency( pitch_step_unit steps_from_start, pitch_step_unit steps_to_target){
    const pitch_step_unit ramp_steps = 
        (pitch_step_unit)PITCH_RAMP_DISTANCE_MM * (pitch_step_unit)PITCH_STEPS_PER_MM;   
    
    //if we are not past halfway the speed is limited by the distance traveled and vice versa
    pitch_step_unit steps_to_nearest_end = steps_from_start;
    if (steps_to_target < steps_to_nearest_end){
        steps_to_nearest_end = steps_to_target;
    }
    if (steps_to_nearest_end >= ramp_steps){
        return PITCH_MAX_STEP_FREQUENCY_HZ;
    }

    return PITCH_MIN_STEP_FREQUENCY_HZ +
        (uint32_t)(
            ((uint64_t)(PITCH_MAX_STEP_FREQUENCY_HZ - PITCH_MIN_STEP_FREQUENCY_HZ) *
             (uint32_t)steps_to_nearest_end) /
            (uint32_t)ramp_steps
        );
}

static motor_result_t pitch_start_pulses(void){
    __HAL_TIM_SET_COUNTER(&PITCH_STEP_TIMER, 0U);
    HAL_TIM_GenerateEvent( &PITCH_STEP_TIMER, TIM_EVENTSOURCE_UPDATE);
    __HAL_TIM_CLEAR_FLAG(
        &PITCH_STEP_TIMER,
        TIM_FLAG_CC1 | TIM_FLAG_UPDATE
    );

    if (HAL_TIM_PWM_Start_IT(&PITCH_STEP_TIMER, PITCH_STEP_TIMER_CHANNEL) != HAL_OK){
        return MOTOR_STATUS_RUNTIME_ERROR;
    }

    pitch_pwm_running = true;
    return MOTOR_STATUS_OK;
}

/* Public functions ---------------------------------------------------------*/
motor_result_t pitch_init_motor(void){
    motor_result_t result;

    pitch_state = MOTOR_STATE_INITIALIZING;

    result = pitch_stop();
    if (result != MOTOR_STATUS_OK){
        pitch_state = MOTOR_STATE_UNINITIALIZED;
        return result;
    }

    result = pitch_set_direction(PITCH_HOME_DIRECTION);
    if (result != MOTOR_STATUS_OK){
        pitch_state = MOTOR_STATE_UNINITIALIZED;
        return result;
    }

    pitch_turn_on();
    HAL_Delay(10);
    //osDelay(1U);// allow the driver enable input to settle

    result = pitch_home();
    if (result != MOTOR_STATUS_OK){
        return result;
    }

    pitch_state = MOTOR_STATE_READY;
    return MOTOR_STATUS_OK;
}

motor_result_t pitch_stop(void){
    HAL_StatusTypeDef hal_status = HAL_OK;

    if (pitch_pwm_running){
        hal_status = HAL_TIM_PWM_Stop_IT(
            &PITCH_STEP_TIMER,
            PITCH_STEP_TIMER_CHANNEL
        );
    }

    pitch_pwm_running = false;
    __HAL_TIM_SET_COMPARE(&PITCH_STEP_TIMER, PITCH_STEP_TIMER_CHANNEL, 0U);

    return (hal_status == HAL_OK) ? MOTOR_STATUS_OK : MOTOR_STATUS_RUNTIME_ERROR;
}

motor_result_t pitch_set_step_frequency(uint32_t frequency_hz){
    uint32_t timer_clock_hz;
    uint32_t period_ticks;

    if ((frequency_hz < PITCH_MIN_STEP_FREQUENCY_HZ) ||
        (frequency_hz > PITCH_MAX_STEP_FREQUENCY_HZ)){
        return MOTOR_STATUS_OUT_OF_BOUNDS;
    }

    timer_clock_hz = pitch_get_timer_clock_hz();
    period_ticks = timer_clock_hz / frequency_hz;

    if (period_ticks < 2U){
        return MOTOR_STATUS_OUT_OF_BOUNDS;
    }

    __HAL_TIM_SET_AUTORELOAD(&PITCH_STEP_TIMER, period_ticks - 1U);
    __HAL_TIM_SET_COMPARE(
        &PITCH_STEP_TIMER,
        PITCH_STEP_TIMER_CHANNEL,
        period_ticks / 2U
    );

    return MOTOR_STATUS_OK;
}

motor_result_t pitch_home(void){
    motor_result_t result;
    bool switch_confirmed = false;

    if ((pitch_state == MOTOR_STATE_MOVING) ||
        (pitch_state == MOTOR_STATE_EMERGENCY)){
        return MOTOR_STATUS_RUNTIME_ERROR;
    }

    pitch_state = MOTOR_STATE_INITIALIZING;
    pitch_turn_on();

    result = pitch_set_direction(PITCH_HOME_DIRECTION);
    if (result != MOTOR_STATUS_OK){
        pitch_enter_recovery(result);
        return result;
    }

    result = pitch_set_step_frequency(PITCH_HOME_STEP_FREQUENCY_HZ);
    if (result != MOTOR_STATUS_OK){
        pitch_enter_recovery(result);
        return result;
    }

    pitch_homing = true;
    while (!switch_confirmed){
        if (!pitch_check_home_switch()){
            result = pitch_start_pulses();
            if (result != MOTOR_STATUS_OK){
                pitch_homing = false;
                pitch_enter_recovery(result);
                return result;
            }

            while (!pitch_check_home_switch()){
                // TODO: Add a homing timeout, stall detection, task ownership, and a complete asynchronous emergency-exit policy.
                if ((pitch_state == MOTOR_STATE_RECOVERY) ||
                    (pitch_state == MOTOR_STATE_EMERGENCY)){
                    pitch_homing = false;
                    (void)pitch_stop();
                    return MOTOR_STATUS_RUNTIME_ERROR;
                }
                HAL_Delay(10);
                //osDelay(PITCH_CONTROL_DELAY_MS);
            }

            result = pitch_stop();
            if (result != MOTOR_STATUS_OK){
                pitch_homing = false;
                pitch_enter_recovery(result);
                return result;
            }
        }
        HAL_Delay(10);
        //osDelay(PITCH_HOME_DEBOUNCE_MS);
        switch_confirmed = pitch_check_home_switch();
    }

    pitch_homing = false;
    pitch_current_steps = 0;
    pitch_target_steps = 0;
    pitch_target_reached = true;
    pitch_state = MOTOR_STATE_READY;
    return MOTOR_STATUS_OK;
}

motor_result_t pitch_get_current_position(pitch_distance_unit *result){
    if (result == NULL){
        return MOTOR_STATUS_INVALID_INPUT;
    }

    *result = (pitch_distance_unit)(pitch_current_steps / (pitch_step_unit)PITCH_STEPS_PER_MM);
    return MOTOR_STATUS_OK;
}

motor_states_t pitch_get_state(void){
    return pitch_state;
}

void pitch_enter_recovery(motor_result_t fault_type){
    (void)fault_type;
    (void)pitch_stop();
    pitch_turn_off();
    pitch_state = MOTOR_STATE_RECOVERY;

    // TODO: Define recovery logging, retry, re-homing, and escalation policy
}

void pitch_enter_emergency(){
    (void)pitch_stop();
    pitch_turn_off();
    pitch_state = MOTOR_STATE_EMERGENCY;

    // TODO: Define emergency notification, latching, and reset policy.
}

motor_result_t pitch_move_to(pitch_distance_unit target_position_mm){
    pitch_step_unit start_steps;
    pitch_step_unit remaining_steps;
    pitch_step_unit moved_steps;
    uint32_t requested_frequency = PITCH_MIN_STEP_FREQUENCY_HZ;
    uint32_t active_frequency = 0U;
    motor_result_t result;

    if ((target_position_mm < PITCH_MIN_POSITION_MM) ||
        (target_position_mm > PITCH_MAX_POSITION_MM)){
        return MOTOR_STATUS_OUT_OF_BOUNDS;
    }

    if (pitch_state != MOTOR_STATE_READY){
        return MOTOR_STATUS_RUNTIME_ERROR;
    }

    pitch_target_steps = pitch_mm_to_steps(target_position_mm);
    start_steps = pitch_current_steps;

    if (pitch_target_steps == pitch_current_steps){
        return MOTOR_STATUS_OK;
    }

    if (pitch_target_steps > pitch_current_steps){
        result = pitch_set_direction(MOTOR_DIRECTION_FORWARD);
    }
    else{
        result = pitch_set_direction(MOTOR_DIRECTION_BACKWARD);
    }

    if (result != MOTOR_STATUS_OK){
        return result;
    }

    pitch_turn_on();
    pitch_target_reached = false;
    pitch_state = MOTOR_STATE_MOVING;

    result = pitch_set_step_frequency(requested_frequency);
    if (result != MOTOR_STATUS_OK){
        pitch_enter_recovery(result);
        return result;
    }
    active_frequency = requested_frequency;

    result = pitch_start_pulses();
    if (result != MOTOR_STATUS_OK){
        pitch_enter_recovery(result);
        return result;
    }

    while (!pitch_target_reached){
        /*
         * TODO: Add a movement timeout, stall detection, task ownership,
         * and a complete asynchronous emergency-exit policy.
         */
        if ((pitch_state == MOTOR_STATE_RECOVERY) ||
            (pitch_state == MOTOR_STATE_EMERGENCY)){
            (void)pitch_stop();
            return MOTOR_STATUS_RUNTIME_ERROR;
        }

        remaining_steps = (pitch_step_unit)abs(pitch_target_steps - pitch_current_steps);
        moved_steps = (pitch_step_unit)abs(pitch_current_steps - start_steps);
        requested_frequency = pitch_calculate_step_frequency(
            moved_steps,
            remaining_steps
        );

        if (requested_frequency != active_frequency){
            result = pitch_set_step_frequency(requested_frequency);
            if (result != MOTOR_STATUS_OK){
                pitch_enter_recovery(result);
                return result;
            }
            active_frequency = requested_frequency;
        }
        HAL_Delay(10);
        //osDelay(PITCH_CONTROL_DELAY_MS);
    }

    result = pitch_stop();
    if (result != MOTOR_STATUS_OK){
        pitch_enter_recovery(result);
        return result;
    }

    pitch_current_steps = pitch_target_steps;
    pitch_state = MOTOR_STATE_READY;
    return MOTOR_STATUS_OK;
}

void pitch_handle_pwm_pulse_finished(TIM_HandleTypeDef *htim){
    if ((htim != &PITCH_STEP_TIMER) ||
        (htim->Channel != HAL_TIM_ACTIVE_CHANNEL_1) ||
        !pitch_pwm_running){
        return;
    }

    if (pitch_homing){
        return;
    }

    if (pitch_direction == MOTOR_DIRECTION_FORWARD){
        pitch_current_steps++;
    }
    else if ((pitch_direction == MOTOR_DIRECTION_BACKWARD) &&
             (pitch_current_steps > 0)){
        pitch_current_steps--;
    }

    if (pitch_current_steps == pitch_target_steps){
        // Stop in the ISR so no extra STEP pulses are generated
        (void)HAL_TIM_PWM_Stop_IT(
            &PITCH_STEP_TIMER,
            PITCH_STEP_TIMER_CHANNEL
        );
        pitch_pwm_running = false;
        pitch_target_reached = true;
    }

    /* Declare the following in main.c later to run interupt handeling
    void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
        pitch_handle_pwm_pulse_finished(htim);
    }
    
    
    */
}
