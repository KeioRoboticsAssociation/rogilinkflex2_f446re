#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define ROBOMASTER_MAX_MOTORS 8
#define ROBOMASTER_CAN_TIMEOUT 1000
#define ROBOMASTER_MAX_CURRENT 30000
#define ROBOMASTER_MIN_CURRENT -30000

// Motor types
typedef enum {
    ROBOMASTER_MOTOR_UNKNOWN = 0,
    ROBOMASTER_MOTOR_GM6020 = 1,
    ROBOMASTER_MOTOR_M3508 = 2,
    ROBOMASTER_MOTOR_M2006 = 3
} robomaster_motor_type_t;

// Motor state
typedef enum {
    ROBOMASTER_STATE_UNINITIALIZED = 0,
    ROBOMASTER_STATE_INITIALIZING,
    ROBOMASTER_STATE_READY,
    ROBOMASTER_STATE_ERROR,
    ROBOMASTER_STATE_TIMEOUT
} robomaster_motor_state_t;

// Control modes
typedef enum {
    ROBOMASTER_CONTROL_CURRENT = 0,
    ROBOMASTER_CONTROL_VELOCITY,
    ROBOMASTER_CONTROL_POSITION
} robomaster_control_mode_t;

// Motor feedback data
typedef struct {
    uint16_t angle_raw;           // Raw angle (0-8191 for GM6020)
    float angle_deg;              // Angle in degrees (0-359.99)
    float angle_continuous_deg;   // Continuous angle tracking
    int16_t velocity_raw;         // Raw velocity (RPM for GM6020)
    float velocity_rpm;           // Velocity in RPM
    int16_t current_raw;          // Raw current/torque
    float current_actual;         // Actual current in mA
    uint8_t temperature;          // Temperature in Celsius
    uint32_t last_update_time;    // Last feedback update timestamp
    bool data_valid;              // Data validity flag
} robomaster_feedback_t;

// Motor configuration
typedef struct {
    robomaster_motor_type_t type;
    uint8_t motor_id;             // Motor ID (1-8)
    CAN_HandleTypeDef* hcan;      // CAN handle
    robomaster_control_mode_t control_mode;
    
    // Control parameters
    float max_current;            // Maximum current limit
    float max_velocity;           // Maximum velocity limit (RPM)
    float position_deadband;      // Position control deadband (degrees)
    
    // PID parameters for velocity/position control
    float kp, ki, kd;
    
    // Safety limits
    bool enable_current_limit;
    bool enable_velocity_limit;
    bool enable_position_limit;
    float min_position_deg;
    float max_position_deg;
} robomaster_motor_config_t;

// Motor handle
typedef struct {
    robomaster_motor_config_t config;
    robomaster_motor_state_t state;
    robomaster_feedback_t feedback;
    
    // Control variables
    float target_current;
    float target_velocity;
    float target_position;
    
    // PID control state
    float pid_integral;
    float pid_last_error;
    uint32_t last_control_time;
    
    // Communication
    uint32_t tx_can_id;           // TX CAN ID for this motor
    uint32_t rx_can_id;           // RX CAN ID for this motor
    uint8_t can_mailbox;          // CAN mailbox for transmission
    
    // Status flags
    bool initialized;
    uint32_t error_count;
    uint32_t timeout_count;
    uint32_t last_heartbeat;
} robomaster_motor_t;

// Motor manager for handling multiple motors
typedef struct {
    robomaster_motor_t motors[ROBOMASTER_MAX_MOTORS];
    uint8_t motor_count;
    CAN_HandleTypeDef* hcan;
    uint32_t last_send_time;
    bool auto_send_enabled;
    uint16_t send_interval_ms;
} robomaster_motor_manager_t;

// Base motor functions
bool robomaster_motor_init(robomaster_motor_t* motor, const robomaster_motor_config_t* config);
void robomaster_motor_deinit(robomaster_motor_t* motor);
bool robomaster_motor_update_feedback(robomaster_motor_t* motor, uint8_t* data);
bool robomaster_motor_send_current(robomaster_motor_t* motor, float current);
bool robomaster_motor_set_velocity(robomaster_motor_t* motor, float velocity_rpm);
bool robomaster_motor_set_position(robomaster_motor_t* motor, float position_deg);
bool robomaster_motor_emergency_stop(robomaster_motor_t* motor);
robomaster_motor_state_t robomaster_motor_get_state(robomaster_motor_t* motor);
robomaster_feedback_t robomaster_motor_get_feedback(robomaster_motor_t* motor);

// Motor manager functions
bool robomaster_manager_init(robomaster_motor_manager_t* manager, CAN_HandleTypeDef* hcan);
bool robomaster_manager_add_motor(robomaster_motor_manager_t* manager, const robomaster_motor_config_t* config);
bool robomaster_manager_remove_motor(robomaster_motor_manager_t* manager, uint8_t motor_id);
robomaster_motor_t* robomaster_manager_get_motor(robomaster_motor_manager_t* manager, uint8_t motor_id);
bool robomaster_manager_send_all_currents(robomaster_motor_manager_t* manager);
bool robomaster_manager_send_m3508_group(CAN_HandleTypeDef* hcan, uint32_t can_id, float currents[4]);
void robomaster_manager_update_all(robomaster_motor_manager_t* manager);
void robomaster_manager_emergency_stop_all(robomaster_motor_manager_t* manager);
void robomaster_manager_enable_auto_send(robomaster_motor_manager_t* manager, bool enable, uint16_t interval_ms);
bool robomaster_manager_process_can_rx(robomaster_motor_manager_t* manager, uint32_t can_id, uint8_t* data);
uint8_t robomaster_manager_get_motor_count(robomaster_motor_manager_t* manager);
uint8_t robomaster_manager_get_healthy_motor_count(robomaster_motor_manager_t* manager);
void robomaster_manager_print_status(robomaster_motor_manager_t* manager);
void robomaster_manager_reset_all_errors(robomaster_motor_manager_t* manager);

// Utility functions
uint32_t robomaster_get_tx_can_id(robomaster_motor_type_t type, uint8_t motor_id);
uint32_t robomaster_get_rx_can_id(robomaster_motor_type_t type, uint8_t motor_id);
float robomaster_raw_angle_to_degrees(uint16_t raw_angle, robomaster_motor_type_t type);
int16_t robomaster_current_to_raw(float current_ma, robomaster_motor_type_t type);
float robomaster_raw_current_to_actual(int16_t raw_current, robomaster_motor_type_t type);

// Control loop functions
void robomaster_motor_control_loop(robomaster_motor_t* motor);
float robomaster_pid_calculate(robomaster_motor_t* motor, float setpoint, float measured_value, float dt);

// Error handling and diagnostics
const char* robomaster_get_state_string(robomaster_motor_state_t state);
const char* robomaster_get_motor_type_string(robomaster_motor_type_t type);
bool robomaster_motor_is_healthy(robomaster_motor_t* motor);
void robomaster_motor_reset_errors(robomaster_motor_t* motor);

#ifdef __cplusplus
}
#endif