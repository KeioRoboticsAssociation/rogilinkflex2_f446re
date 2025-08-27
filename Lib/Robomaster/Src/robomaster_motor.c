#include "robomaster_motor.h"
#include <string.h>
#include <math.h>

static const char* MOTOR_STATE_STRINGS[] = {
    "UNINITIALIZED",
    "INITIALIZING", 
    "READY",
    "ERROR",
    "TIMEOUT"
};

static const char* MOTOR_TYPE_STRINGS[] = {
    "UNKNOWN",
    "GM6020",
    "M3508", 
    "M2006"
};

bool robomaster_motor_init(robomaster_motor_t* motor, const robomaster_motor_config_t* config) {
    if (!motor || !config || !config->hcan) {
        return false;
    }
    
    // Clear motor structure
    memset(motor, 0, sizeof(robomaster_motor_t));
    
    // Copy configuration
    memcpy(&motor->config, config, sizeof(robomaster_motor_config_t));
    
    // Set initial state
    motor->state = ROBOMASTER_STATE_INITIALIZING;
    
    // Calculate CAN IDs based on motor type and ID
    motor->tx_can_id = robomaster_get_tx_can_id(config->type, config->motor_id);
    motor->rx_can_id = robomaster_get_rx_can_id(config->type, config->motor_id);
    
    // Initialize control parameters
    motor->target_current = 0.0f;
    motor->target_velocity = 0.0f;
    motor->target_position = 0.0f;
    
    // Initialize PID state
    motor->pid_integral = 0.0f;
    motor->pid_last_error = 0.0f;
    motor->last_control_time = HAL_GetTick();
    
    // Initialize feedback
    motor->feedback.data_valid = false;
    motor->feedback.last_update_time = 0;
    
    // Mark as initialized
    motor->initialized = true;
    motor->state = ROBOMASTER_STATE_READY;
    motor->last_heartbeat = HAL_GetTick();
    
    return true;
}

void robomaster_motor_deinit(robomaster_motor_t* motor) {
    if (!motor) return;
    
    // Send zero current to stop motor
    robomaster_motor_emergency_stop(motor);
    
    // Clear structure
    memset(motor, 0, sizeof(robomaster_motor_t));
}

bool robomaster_motor_update_feedback(robomaster_motor_t* motor, uint8_t* data) {
    if (!motor || !data) {
        return false;
    }
    
    uint32_t current_time = HAL_GetTick();
    
    // Parse feedback based on motor type
    switch (motor->config.type) {
        case ROBOMASTER_MOTOR_GM6020: {
            // Extract angle (16-bit, big-endian)
            motor->feedback.angle_raw = ((uint16_t)data[0] << 8) | data[1];
            motor->feedback.angle_deg = robomaster_raw_angle_to_degrees(motor->feedback.angle_raw, motor->config.type);
            
            // Extract velocity (16-bit, big-endian, signed)
            motor->feedback.velocity_raw = ((int16_t)data[2] << 8) | data[3];
            motor->feedback.velocity_rpm = (float)motor->feedback.velocity_raw;
            
            // Extract current (16-bit, big-endian, signed)
            motor->feedback.current_raw = ((int16_t)data[4] << 8) | data[5];
            motor->feedback.current_actual = robomaster_raw_current_to_actual(motor->feedback.current_raw, motor->config.type);
            
            // Extract temperature
            motor->feedback.temperature = data[6];
            break;
        }
        
        case ROBOMASTER_MOTOR_M3508:
        case ROBOMASTER_MOTOR_M2006:
            // Similar parsing for other motor types can be added here
            // For now, use same format as GM6020
            motor->feedback.angle_raw = ((uint16_t)data[0] << 8) | data[1];
            motor->feedback.angle_deg = robomaster_raw_angle_to_degrees(motor->feedback.angle_raw, motor->config.type);
            motor->feedback.velocity_raw = ((int16_t)data[2] << 8) | data[3];
            motor->feedback.velocity_rpm = (float)motor->feedback.velocity_raw;
            motor->feedback.current_raw = ((int16_t)data[4] << 8) | data[5];
            motor->feedback.current_actual = robomaster_raw_current_to_actual(motor->feedback.current_raw, motor->config.type);
            motor->feedback.temperature = data[6];
            break;
            
        default:
            return false;
    }
    
    // Update continuous angle tracking
    static uint16_t last_raw_angle = 0;
    static int32_t angle_turns = 0;
    
    if (motor->feedback.data_valid) {
        int32_t angle_diff = (int32_t)motor->feedback.angle_raw - (int32_t)last_raw_angle;
        
        // Handle wrap-around
        if (angle_diff > 4096) {
            angle_turns--;
        } else if (angle_diff < -4096) {
            angle_turns++;
        }
    }
    
    last_raw_angle = motor->feedback.angle_raw;
    motor->feedback.angle_continuous_deg = motor->feedback.angle_deg + (angle_turns * 360.0f);
    
    // Update timestamps and status
    motor->feedback.last_update_time = current_time;
    motor->feedback.data_valid = true;
    motor->last_heartbeat = current_time;
    
    // Update motor state
    if (motor->state == ROBOMASTER_STATE_TIMEOUT) {
        motor->state = ROBOMASTER_STATE_READY;
    }
    
    return true;
}

bool robomaster_motor_send_current(robomaster_motor_t* motor, float current) {
    if (!motor || !motor->initialized || !motor->config.hcan) {
        return false;
    }
    
    // Apply current limits
    if (motor->config.enable_current_limit) {
        if (current > motor->config.max_current) {
            current = motor->config.max_current;
        } else if (current < -motor->config.max_current) {
            current = -motor->config.max_current;
        }
    }
    
    // Clamp to absolute hardware limits
    if (current > ROBOMASTER_MAX_CURRENT) {
        current = ROBOMASTER_MAX_CURRENT;
    } else if (current < ROBOMASTER_MIN_CURRENT) {
        current = ROBOMASTER_MIN_CURRENT;
    }
    
    // Convert to raw current value
    int16_t raw_current = robomaster_current_to_raw(current, motor->config.type);
    
    // Prepare CAN message
    CAN_TxHeaderTypeDef tx_header;
    tx_header.StdId = motor->tx_can_id;
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;
    
    uint8_t tx_data[8] = {0};
    
    // Pack current data based on motor ID
    uint8_t motor_index = (motor->config.motor_id - 1) % 4;  // 0-3 for motors 1-4, 5-8
    tx_data[motor_index * 2] = (raw_current >> 8) & 0xFF;      // High byte
    tx_data[motor_index * 2 + 1] = raw_current & 0xFF;         // Low byte
    
    // Send CAN message
    uint32_t mailbox;
    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(motor->config.hcan, &tx_header, tx_data, &mailbox);
    
    if (status == HAL_OK) {
        motor->target_current = current;
        return true;
    } else {
        motor->error_count++;
        return false;
    }
}

bool robomaster_motor_set_velocity(robomaster_motor_t* motor, float velocity_rpm) {
    if (!motor || !motor->initialized) {
        return false;
    }
    
    // Apply velocity limits
    if (motor->config.enable_velocity_limit) {
        if (fabs(velocity_rpm) > motor->config.max_velocity) {
            velocity_rpm = (velocity_rpm > 0) ? motor->config.max_velocity : -motor->config.max_velocity;
        }
    }
    
    motor->target_velocity = velocity_rpm;
    motor->config.control_mode = ROBOMASTER_CONTROL_VELOCITY;
    
    return true;
}

bool robomaster_motor_set_position(robomaster_motor_t* motor, float position_deg) {
    if (!motor || !motor->initialized) {
        return false;
    }
    
    // Apply position limits
    if (motor->config.enable_position_limit) {
        if (position_deg > motor->config.max_position_deg) {
            position_deg = motor->config.max_position_deg;
        } else if (position_deg < motor->config.min_position_deg) {
            position_deg = motor->config.min_position_deg;
        }
    }
    
    motor->target_position = position_deg;
    motor->config.control_mode = ROBOMASTER_CONTROL_POSITION;
    
    return true;
}

bool robomaster_motor_emergency_stop(robomaster_motor_t* motor) {
    if (!motor) {
        return false;
    }
    
    // Send zero current
    bool result = robomaster_motor_send_current(motor, 0.0f);
    
    // Reset control variables
    motor->target_current = 0.0f;
    motor->target_velocity = 0.0f;
    motor->pid_integral = 0.0f;
    motor->pid_last_error = 0.0f;
    
    return result;
}

robomaster_motor_state_t robomaster_motor_get_state(robomaster_motor_t* motor) {
    if (!motor) {
        return ROBOMASTER_STATE_ERROR;
    }
    
    // Check for timeout
    uint32_t current_time = HAL_GetTick();
    if (current_time - motor->last_heartbeat > ROBOMASTER_CAN_TIMEOUT) {
        motor->state = ROBOMASTER_STATE_TIMEOUT;
        motor->timeout_count++;
    }
    
    return motor->state;
}

robomaster_feedback_t robomaster_motor_get_feedback(robomaster_motor_t* motor) {
    robomaster_feedback_t empty_feedback = {0};
    
    if (!motor) {
        return empty_feedback;
    }
    
    return motor->feedback;
}

void robomaster_motor_control_loop(robomaster_motor_t* motor) {
    if (!motor || !motor->initialized || motor->state != ROBOMASTER_STATE_READY) {
        return;
    }
    
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - motor->last_control_time) / 1000.0f;  // Convert to seconds
    motor->last_control_time = current_time;
    
    if (dt <= 0.0f) return;  // Avoid division by zero
    
    float control_current = 0.0f;
    
    switch (motor->config.control_mode) {
        case ROBOMASTER_CONTROL_CURRENT:
            control_current = motor->target_current;
            break;
            
        case ROBOMASTER_CONTROL_VELOCITY:
            if (motor->feedback.data_valid) {
                control_current = robomaster_pid_calculate(motor, motor->target_velocity, 
                                                          motor->feedback.velocity_rpm, dt);
            }
            break;
            
        case ROBOMASTER_CONTROL_POSITION:
            if (motor->feedback.data_valid) {
                float position_error = motor->target_position - motor->feedback.angle_continuous_deg;
                
                // Apply deadband
                if (fabs(position_error) < motor->config.position_deadband) {
                    control_current = 0.0f;
                } else {
                    control_current = robomaster_pid_calculate(motor, motor->target_position, 
                                                              motor->feedback.angle_continuous_deg, dt);
                }
            }
            break;
    }
    
    // Send calculated current
    robomaster_motor_send_current(motor, control_current);
}

float robomaster_pid_calculate(robomaster_motor_t* motor, float setpoint, float measured_value, float dt) {
    if (!motor || dt <= 0.0f) {
        return 0.0f;
    }
    
    float error = setpoint - measured_value;
    
    // Proportional term
    float p_term = motor->config.kp * error;
    
    // Integral term with windup protection
    motor->pid_integral += error * dt;
    float i_limit = motor->config.max_current / (motor->config.ki + 1e-6f);  // Prevent division by zero
    if (motor->pid_integral > i_limit) {
        motor->pid_integral = i_limit;
    } else if (motor->pid_integral < -i_limit) {
        motor->pid_integral = -i_limit;
    }
    float i_term = motor->config.ki * motor->pid_integral;
    
    // Derivative term
    float d_term = motor->config.kd * (error - motor->pid_last_error) / dt;
    motor->pid_last_error = error;
    
    // Calculate output
    float output = p_term + i_term + d_term;
    
    return output;
}

// Utility functions
uint32_t robomaster_get_tx_can_id(robomaster_motor_type_t type, uint8_t motor_id) {
    switch (type) {
        case ROBOMASTER_MOTOR_GM6020:
            return 0x1FF;  // All GM6020 motors use 0x1FF for control
            
        case ROBOMASTER_MOTOR_M3508:
        case ROBOMASTER_MOTOR_M2006:
            return (motor_id <= 4) ? 0x200 : 0x1FF;  // Motors 1-4 use 0x200, 5-8 use 0x1FF
            
        default:
            return 0x1FF;
    }
}

uint32_t robomaster_get_rx_can_id(robomaster_motor_type_t type, uint8_t motor_id) {
    switch (type) {
        case ROBOMASTER_MOTOR_GM6020:
            return 0x205 + (motor_id - 1);  // 0x205-0x208 for GM6020 motors 1-4
            
        case ROBOMASTER_MOTOR_M3508:
        case ROBOMASTER_MOTOR_M2006:
            return 0x201 + (motor_id - 1);  // 0x201-0x208 for M3508/M2006 motors 1-8
            
        default:
            return 0x205;
    }
}

float robomaster_raw_angle_to_degrees(uint16_t raw_angle, robomaster_motor_type_t type) {
    switch (type) {
        case ROBOMASTER_MOTOR_GM6020:
            return ((float)raw_angle / 8192.0f) * 360.0f;  // 13-bit resolution
            
        case ROBOMASTER_MOTOR_M3508:
        case ROBOMASTER_MOTOR_M2006:
            return ((float)raw_angle / 8192.0f) * 360.0f;  // Same resolution
            
        default:
            return 0.0f;
    }
}

int16_t robomaster_current_to_raw(float current_ma, robomaster_motor_type_t type) {
    // For most RoboMaster motors, raw current is directly proportional
    // GM6020: Current range is typically -30000 to +30000 for full torque
    int16_t raw = (int16_t)current_ma;
    
    // Clamp to valid range
    if (raw > ROBOMASTER_MAX_CURRENT) raw = ROBOMASTER_MAX_CURRENT;
    if (raw < ROBOMASTER_MIN_CURRENT) raw = ROBOMASTER_MIN_CURRENT;
    
    return raw;
}

float robomaster_raw_current_to_actual(int16_t raw_current, robomaster_motor_type_t type) {
    // Simple conversion - could be calibrated for specific motor types
    return (float)raw_current;
}

// String utility functions
const char* robomaster_get_state_string(robomaster_motor_state_t state) {
    if (state >= 0 && state < sizeof(MOTOR_STATE_STRINGS)/sizeof(MOTOR_STATE_STRINGS[0])) {
        return MOTOR_STATE_STRINGS[state];
    }
    return "UNKNOWN";
}

const char* robomaster_get_motor_type_string(robomaster_motor_type_t type) {
    if (type >= 0 && type < sizeof(MOTOR_TYPE_STRINGS)/sizeof(MOTOR_TYPE_STRINGS[0])) {
        return MOTOR_TYPE_STRINGS[type];
    }
    return "UNKNOWN";
}

bool robomaster_motor_is_healthy(robomaster_motor_t* motor) {
    if (!motor) return false;
    
    uint32_t current_time = HAL_GetTick();
    
    // Check basic health conditions
    bool initialized_ok = motor->initialized;
    bool state_ok = (motor->state == ROBOMASTER_STATE_READY);
    bool heartbeat_ok = (current_time - motor->last_heartbeat) < ROBOMASTER_CAN_TIMEOUT;
    bool feedback_ok = motor->feedback.data_valid;
    bool error_rate_ok = motor->error_count < 100;  // Arbitrary threshold
    
    return initialized_ok && state_ok && heartbeat_ok && feedback_ok && error_rate_ok;
}

void robomaster_motor_reset_errors(robomaster_motor_t* motor) {
    if (!motor) return;
    
    motor->error_count = 0;
    motor->timeout_count = 0;
    motor->pid_integral = 0.0f;
    motor->pid_last_error = 0.0f;
    
    if (motor->state == ROBOMASTER_STATE_ERROR) {
        motor->state = ROBOMASTER_STATE_READY;
    }
}