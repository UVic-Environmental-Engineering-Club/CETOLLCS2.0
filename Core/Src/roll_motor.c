/*
 * roll_motor.c
 *
 * Created on: Sept 10, 2026
 * Author: igorc
 */

#include "roll_motor.h"
#include "cmsis_os.h"
#include <stdbool.h>

// Private constants --------------------------------------------------------*/
#define ROLL_DEGREE_RANGE 180
#define ROLL_ADC_POSITION_MAX 3447U
#define ROLL_ADC_POSITION_MIN 550U
#define ROLL_ADC_VALID_MIN 50U
#define ROLL_ADC_VALID_MAX 4095U

// Private variables --------------------------------------------------------*/
static volatile roll_dma_type roll_adc_buffer[1];
static roll_degree_unit roll_current_position;
static volatile motor_states_t roll_state = MOTOR_STATE_UNINITIALIZED;

// Private function prototypes ---------------------------------------------*/
static motor_result_t roll_set_direction(motor_direction_t direction);

// Private functions --------------------------------------------------------*/
static motor_result_t roll_set_direction(motor_direction_t direction){
    switch (direction)
    {
        case MOTOR_DIRECTION_CW:
            HAL_GPIO_WritePin(
                Roll_DIR_GPIO_Port,
                Roll_DIR_Pin,
                GPIO_PIN_RESET
            );
            break;

        case MOTOR_DIRECTION_CCW:
            HAL_GPIO_WritePin(
                Roll_DIR_GPIO_Port,
                Roll_DIR_Pin,
                GPIO_PIN_SET
            );
            break;

        default:
            return MOTOR_STATUS_INVALID_INPUT;
    }

    return MOTOR_STATUS_OK;
}

// Public functions ---------------------------------------------------------*/
motor_result_t roll_init_motor(void){
    motor_result_t result;
    roll_degree_unit initial_position;

    roll_state = MOTOR_STATE_INITIALIZING;

    if (HAL_TIM_PWM_Start(&ROLL_PWM_TIMER, ROLL_PWM_TIMER_CHANNEL) != HAL_OK){
        roll_state = MOTOR_STATE_UNINITIALIZED;
        return MOTOR_STATUS_RUNTIME_ERROR;
    }

    result = roll_stop();
    if (result != MOTOR_STATUS_OK){
        roll_state = MOTOR_STATE_UNINITIALIZED;
        return result;
    }

    if (HAL_ADC_Start_DMA(&hadc3, (uint32_t *)roll_adc_buffer, 1U) != HAL_OK){
        (void)roll_stop();
        roll_state = MOTOR_STATE_UNINITIALIZED;
        return MOTOR_STATUS_RUNTIME_ERROR;
    }

    result = roll_set_direction(ROLL_START_DIRECTION);
    if (result != MOTOR_STATUS_OK){
        (void)roll_stop();
        roll_state = MOTOR_STATE_UNINITIALIZED;
        return result;
    }

    //allow ADC/DMA to collect an initial value, then verify the pot.
    HAL_Delay(10);
    //osDelay(50U);
    result = roll_get_current_degree(&initial_position);
    if (result != MOTOR_STATUS_OK){
        (void)roll_stop();
        roll_state = MOTOR_STATE_UNINITIALIZED;
        return result;
    }

    roll_current_position = initial_position;
    roll_state = MOTOR_STATE_READY;
    return MOTOR_STATUS_OK;
}

motor_result_t roll_stop(void){
    return roll_set_duty_cycle(0.0f);
}

motor_result_t roll_set_duty_cycle(float duty_cycle){
    if ((duty_cycle < 0.0f) || (duty_cycle > 1.0f)){
        return MOTOR_STATUS_OUT_OF_BOUNDS;
    }

    uint32_t timer_arr = __HAL_TIM_GET_AUTORELOAD(&ROLL_PWM_TIMER);
    uint32_t compare_value = (uint32_t)((float)timer_arr * duty_cycle);
    __HAL_TIM_SET_COMPARE(
        &ROLL_PWM_TIMER,
        ROLL_PWM_TIMER_CHANNEL,
        compare_value
    );

    return MOTOR_STATUS_OK;
}

motor_result_t roll_read_pot_safe(roll_dma_type *result){
    roll_dma_type samples[ROLL_POT_SAMPLE_SIZE];

    if (result == NULL){
        return MOTOR_STATUS_INVALID_INPUT;
    }

    for (uint32_t i = 0U; i < ROLL_POT_SAMPLE_SIZE; i++){
        samples[i] = roll_adc_buffer[0];
        HAL_Delay(2);
        //osDelay(2U);
    }

    //bubble sort
    for (uint32_t i = 0U; i < (ROLL_POT_SAMPLE_SIZE - 1U); i++){
        for (uint32_t j = 0U; j < (ROLL_POT_SAMPLE_SIZE - 1U - i); j++){
            if (samples[j] > samples[j + 1U]){
                roll_dma_type temp = samples[j];

                samples[j] = samples[j + 1U];
                samples[j + 1U] = temp;
            }
        }
    }
    roll_dma_type median = samples[ROLL_POT_SAMPLE_SIZE / 2U];

    if ((median < ROLL_ADC_VALID_MIN) || (median > ROLL_ADC_VALID_MAX)){
        return MOTOR_STATUS_RUNTIME_ERROR;
    }

    *result = median;
    return MOTOR_STATUS_OK;
}

motor_result_t roll_get_current_degree(roll_degree_unit *result){
    roll_dma_type current_adc;
    motor_result_t status;

    if (result == NULL){
        return MOTOR_STATUS_INVALID_INPUT;
    }

    status = roll_read_pot_safe(&current_adc);
    if (status != MOTOR_STATUS_OK){
        return status;
    }

    if ((current_adc < ROLL_ADC_POSITION_MIN) ||
        (current_adc > ROLL_ADC_POSITION_MAX)){
        return MOTOR_STATUS_OUT_OF_BOUNDS;
    }

    //convert ADC position to degrees.
    *result = (roll_degree_unit)(
        ((uint32_t)(current_adc - ROLL_ADC_POSITION_MIN) * ROLL_DEGREE_RANGE) /
        (ROLL_ADC_POSITION_MAX - ROLL_ADC_POSITION_MIN)
    );

    return MOTOR_STATUS_OK;
}

motor_states_t roll_get_state(void){
    return roll_state;
}

void roll_enter_recovery(motor_result_t fault_type){
    (void)fault_type;
    (void)roll_stop();
    roll_state = MOTOR_STATE_RECOVERY;

    //TODO: Define recovery logging, retry, and escalation policy
}

void roll_enter_emergency(){
    (void)roll_stop();
    roll_state = MOTOR_STATE_EMERGENCY;

    //TODO: Define emergency notification, latching, and reset policy
}

motor_result_t roll_move_to(roll_degree_unit target_degree){
    roll_degree_unit check_pos;
    motor_direction_t movement_direction;
    motor_result_t result;
    int difference;
    float active_duty_cycle;

    if ((target_degree > ROLL_DEGREE_RANGE) || (target_degree < 0)){
        return MOTOR_STATUS_INVALID_INPUT;
    }

    if (roll_state != MOTOR_STATE_READY){
        return MOTOR_STATUS_RUNTIME_ERROR;
    }

    result = roll_get_current_degree(&check_pos);
    if (result != MOTOR_STATUS_OK){
        return result;
    }
    roll_current_position = check_pos;

    if (target_degree > roll_current_position){
        movement_direction = MOTOR_DIRECTION_CW;
    }
    else if (target_degree < roll_current_position){
        movement_direction = MOTOR_DIRECTION_CCW;
    }
    else{
        return MOTOR_STATUS_OK;
    }

    result = roll_set_direction(movement_direction);
    if (result != MOTOR_STATUS_OK){
        return result;
    }

    difference = abs(roll_current_position - target_degree);
    active_duty_cycle = (difference < ROLL_SLOW_DOWN_DIFF) ?
        ROLL_SLOWED_DUTY_CYCLE : ROLL_MAX_DUTY_CYCLE;

    roll_state = MOTOR_STATE_MOVING;
    result = roll_set_duty_cycle(active_duty_cycle);
    if (result != MOTOR_STATUS_OK){
        roll_enter_recovery(result);
        return result;
    }

    while (true){
        // TODO: Add a movement timeout, stall detection, task ownership, and a complete asynchronous emergency-exit policy

        if ((roll_state == MOTOR_STATE_EMERGENCY) ||
            (roll_state == MOTOR_STATE_RECOVERY)){
            (void)roll_stop();
            return MOTOR_STATUS_RUNTIME_ERROR;
        }

        result = roll_get_current_degree(&check_pos);
        if (result != MOTOR_STATUS_OK){
            roll_enter_recovery(result);
            return result;
        }
        roll_current_position = check_pos;

        difference = abs(check_pos - target_degree);
        if (difference <= ROLL_POSITION_ERROR){ //arrived
            break;
        }

        // Stop if the sampled position crossed the target between iterations. */
        if (((movement_direction == MOTOR_DIRECTION_CW) &&
             (check_pos > target_degree)) ||
            ((movement_direction == MOTOR_DIRECTION_CCW) &&
             (check_pos < target_degree))){
            break;
        }

        if ((difference < ROLL_SLOW_DOWN_DIFF) &&
            (active_duty_cycle != ROLL_SLOWED_DUTY_CYCLE)){
            result = roll_set_duty_cycle(ROLL_SLOWED_DUTY_CYCLE);
            if (result != MOTOR_STATUS_OK){
                roll_enter_recovery(result);
                return result;
            }
            active_duty_cycle = ROLL_SLOWED_DUTY_CYCLE;
        }
        HAL_Delay(10);
        //osDelay(10U);
    }

    result = roll_stop();
    if (result != MOTOR_STATUS_OK){
        roll_enter_recovery(result);
        return result;
    }

    roll_state = MOTOR_STATE_READY;
    return MOTOR_STATUS_OK;
}
