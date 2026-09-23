/*
 * utils.h
 *
 *  Created on: Sept 10, 2026
 *      Author: igorc
 */


typedef enum {
    MOTOR_STATUS_OK = 0,
    MOTOR_STATUS_INVALID_INPUT,
    MOTOR_STATUS_OUT_OF_BOUNDS,
    MOTOR_STATUS_RUNTIME_ERROR,
} motor_result_t;

typedef enum {
    MOTOR_DIRECTION_CW,
    MOTOR_DIRECTION_CCW,
    MOTOR_DIRECTION_FORWARD,
    MOTOR_DIRECTION_BACKWARD,
} motor_direction_t;

typedef enum {
    MOTOR_STATE_UNINITIALIZED,
    MOTOR_STATE_INITIALIZING, //homing and such
    MOTOR_STATE_READY,
    MOTOR_STATE_MOVING,
    MOTOR_STATE_RECOVERY,
    MOTOR_STATE_EMERGENCY,
} motor_states_t;