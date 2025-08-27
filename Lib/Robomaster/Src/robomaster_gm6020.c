#include "robomaster_gm6020.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

// GM6020 default configuration
const robomaster_motor_config_t GM6020_DEFAULT_CONFIG = {
    .type = ROBOMASTER_MOTOR_GM6020,
    .motor_id = 1,
    .hcan = NULL,  // Must be set by user
    .control_mode = ROBOMASTER_CONTROL_CURRENT,
    .max_current = GM6020_MAX_CURRENT * 0.8f,  // 80% of max for safety
    .max_velocity = 600.0f,  // RPM - reasonable limit for GM6020
    .position_deadband = 0.5f,  // 0.5 degrees deadband
    .kp = 50.0f,    // Default PID gains - should be tuned
    .ki = 0.1f,
    .kd = 2.0f,
    .enable_current_limit = true,
    .enable_velocity_limit = true,
    .enable_position_limit = false,
    .min_position_deg = -180.0f,
    .max_position_deg = 180.0f
};

bool gm6020_init(robomaster_motor_t* motor, uint8_t motor_id, CAN_HandleTypeDef* hcan) {
    if (!motor || !hcan || motor_id < 1 || motor_id > 8) {
        return false;
    }
    
    // Create configuration for GM6020
    robomaster_motor_config_t config = GM6020_DEFAULT_CONFIG;
    config.motor_id = motor_id;
    config.hcan = hcan;
    
    // Initialize using base motor initialization
    return robomaster_motor_init(motor, &config);
}

bool gm6020_parse_feedback(robomaster_motor_t* motor, uint8_t* can_data) {
    if (!motor || !can_data) {
        return false;
    }
    
    // Parse GM6020 specific feedback format
    uint16_t raw_angle = gm6020_get_raw_angle(can_data);
    int16_t raw_velocity = gm6020_get_raw_velocity(can_data);
    int16_t raw_current = gm6020_get_raw_current(can_data);
    uint8_t temperature = gm6020_get_temperature(can_data);
    
    // Update feedback structure
    motor->feedback.angle_raw = raw_angle;
    motor->feedback.angle_deg = gm6020_raw_angle_to_degrees(raw_angle);
    motor->feedback.velocity_raw = raw_velocity;
    motor->feedback.velocity_rpm = (float)raw_velocity;  // GM6020 reports in RPM
    motor->feedback.current_raw = raw_current;
    motor->feedback.current_actual = gm6020_raw_current_to_actual(raw_current);
    motor->feedback.temperature = temperature;
    
    // Update continuous angle tracking
    gm6020_update_continuous_angle(motor);
    
    // Update timestamps and status
    uint32_t current_time = HAL_GetTick();
    motor->feedback.last_update_time = current_time;
    motor->feedback.data_valid = true;
    motor->last_heartbeat = current_time;
    
    // Update motor state if it was in timeout
    if (motor->state == ROBOMASTER_STATE_TIMEOUT) {
        motor->state = ROBOMASTER_STATE_READY;
    }
    
    return true;
}

bool gm6020_send_current_group(CAN_HandleTypeDef* hcan, float current1, float current2, float current3, float current4) {
    if (!hcan) {
        return false;
    }
    
    // Convert currents to raw values
    int16_t raw_currents[4] = {
        gm6020_current_to_raw(current1),
        gm6020_current_to_raw(current2),
        gm6020_current_to_raw(current3),
        gm6020_current_to_raw(current4)
    };
    
    // Prepare CAN message
    CAN_TxHeaderTypeDef tx_header;
    tx_header.StdId = GM6020_TX_CAN_ID_BASE;
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;
    
    uint8_t tx_data[8];
    
    // Pack current data for motors 1-4
    tx_data[GM6020_CURRENT_1_HIGH_BYTE] = (raw_currents[0] >> 8) & 0xFF;
    tx_data[GM6020_CURRENT_1_LOW_BYTE] = raw_currents[0] & 0xFF;
    tx_data[GM6020_CURRENT_2_HIGH_BYTE] = (raw_currents[1] >> 8) & 0xFF;
    tx_data[GM6020_CURRENT_2_LOW_BYTE] = raw_currents[1] & 0xFF;
    tx_data[GM6020_CURRENT_3_HIGH_BYTE] = (raw_currents[2] >> 8) & 0xFF;
    tx_data[GM6020_CURRENT_3_LOW_BYTE] = raw_currents[2] & 0xFF;
    tx_data[GM6020_CURRENT_4_HIGH_BYTE] = (raw_currents[3] >> 8) & 0xFF;
    tx_data[GM6020_CURRENT_4_LOW_BYTE] = raw_currents[3] & 0xFF;
    
    // Send CAN message
    uint32_t mailbox;
    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &mailbox);
    
    return (status == HAL_OK);
}

bool gm6020_send_current_single(robomaster_motor_t* motor, float current) {
    if (!motor || !motor->initialized) {
        return false;
    }
    
    // For single motor control, we still need to send all 4 currents in one message
    // Set only the target motor's current, others to 0
    float currents[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t motor_index = (motor->config.motor_id - 1) % 4;
    currents[motor_index] = current;
    
    return gm6020_send_current_group(motor->config.hcan, currents[0], currents[1], currents[2], currents[3]);
}

// GM6020 utility functions
uint16_t gm6020_get_raw_angle(uint8_t* can_data) {
    return ((uint16_t)can_data[GM6020_ANGLE_HIGH_BYTE] << 8) | can_data[GM6020_ANGLE_LOW_BYTE];
}

int16_t gm6020_get_raw_velocity(uint8_t* can_data) {
    return ((int16_t)can_data[GM6020_VELOCITY_HIGH_BYTE] << 8) | can_data[GM6020_VELOCITY_LOW_BYTE];
}

int16_t gm6020_get_raw_current(uint8_t* can_data) {
    return ((int16_t)can_data[GM6020_CURRENT_HIGH_BYTE] << 8) | can_data[GM6020_CURRENT_LOW_BYTE];
}

uint8_t gm6020_get_temperature(uint8_t* can_data) {
    return can_data[GM6020_TEMPERATURE_BYTE];
}

uint32_t gm6020_get_rx_can_id(uint8_t motor_id) {
    if (motor_id >= 1 && motor_id <= 4) {
        return GM6020_RX_CAN_ID_BASE + (motor_id - 1);  // 0x205-0x208
    }
    return GM6020_RX_CAN_ID_BASE;  // Default to 0x205
}

float gm6020_raw_angle_to_degrees(uint16_t raw_angle) {
    return ((float)raw_angle * GM6020_ANGLE_DEGREES_PER_COUNT);
}

float gm6020_normalize_angle(float angle_deg) {
    // Normalize angle to 0-360 range
    while (angle_deg < 0.0f) {
        angle_deg += 360.0f;
    }
    while (angle_deg >= 360.0f) {
        angle_deg -= 360.0f;
    }
    return angle_deg;
}

int16_t gm6020_current_to_raw(float current_ma) {
    // Clamp to GM6020 limits
    if (current_ma > GM6020_MAX_CURRENT) {
        current_ma = GM6020_MAX_CURRENT;
    } else if (current_ma < GM6020_MIN_CURRENT) {
        current_ma = GM6020_MIN_CURRENT;
    }
    
    return (int16_t)current_ma;
}

float gm6020_raw_current_to_actual(int16_t raw_current) {
    // For GM6020, the raw current value directly represents the current
    // Additional scaling could be applied here if needed for specific applications
    return (float)raw_current;
}

void gm6020_update_continuous_angle(robomaster_motor_t* motor) {
    if (!motor || !motor->feedback.data_valid) {
        return;
    }
    
    static uint16_t last_raw_angles[8] = {0};  // Support up to 8 motors
    static int32_t angle_turns[8] = {0};
    static bool first_update[8] = {true, true, true, true, true, true, true, true};
    
    uint8_t motor_index = motor->config.motor_id - 1;
    if (motor_index >= 8) return;  // Safety check
    
    if (first_update[motor_index]) {
        last_raw_angles[motor_index] = motor->feedback.angle_raw;
        angle_turns[motor_index] = 0;
        first_update[motor_index] = false;
    } else {
        int32_t angle_diff = (int32_t)motor->feedback.angle_raw - (int32_t)last_raw_angles[motor_index];
        
        // Handle wrap-around (GM6020 has 13-bit resolution: 0-8191)
        if (angle_diff > GM6020_ANGLE_RESOLUTION / 2) {
            angle_turns[motor_index]--;
        } else if (angle_diff < -GM6020_ANGLE_RESOLUTION / 2) {
            angle_turns[motor_index]++;
        }
        
        last_raw_angles[motor_index] = motor->feedback.angle_raw;
    }
    
    // Calculate continuous angle
    motor->feedback.angle_continuous_deg = motor->feedback.angle_deg + (angle_turns[motor_index] * 360.0f);
}

// GM6020 advanced control functions
bool gm6020_set_position_with_feedforward(robomaster_motor_t* motor, float target_pos_deg, float feedforward_current) {
    if (!motor || !motor->initialized) {
        return false;
    }
    
    // Set target position
    motor->target_position = target_pos_deg;
    motor->config.control_mode = ROBOMASTER_CONTROL_POSITION;
    
    // Calculate position control current
    if (motor->feedback.data_valid) {
        float position_error = target_pos_deg - motor->feedback.angle_continuous_deg;
        
        // Simple proportional control with feedforward
        float control_current = motor->config.kp * position_error + feedforward_current;
        
        return gm6020_send_current_single(motor, control_current);
    }
    
    return false;
}

bool gm6020_calibrate_zero_position(robomaster_motor_t* motor) {
    if (!motor || !motor->initialized || !motor->feedback.data_valid) {
        return false;
    }
    
    // Reset continuous angle tracking to use current position as zero
    motor->feedback.angle_continuous_deg = 0.0f;
    
    // You might want to store this offset in non-volatile memory
    // This is a simple implementation that just resets the tracking
    
    return true;
}

bool gm6020_self_test(robomaster_motor_t* motor) {
    if (!motor || !motor->initialized) {
        return false;
    }
    
    // Basic self-test sequence
    uint32_t test_start_time = HAL_GetTick();
    const uint32_t TEST_TIMEOUT = 5000;  // 5 second timeout
    
    // Step 1: Check if we can receive feedback
    uint32_t initial_update_time = motor->feedback.last_update_time;
    while (motor->feedback.last_update_time == initial_update_time) {
        HAL_Delay(10);
        if (HAL_GetTick() - test_start_time > TEST_TIMEOUT) {
            return false;  // No feedback received
        }
    }
    
    // Step 2: Send small current command and check for response
    float initial_angle = motor->feedback.angle_continuous_deg;
    gm6020_send_current_single(motor, 5000);  // Small positive current
    HAL_Delay(500);
    
    // Stop motor
    gm6020_send_current_single(motor, 0);
    HAL_Delay(100);
    
    // Check if motor moved (indicating it's responding to commands)
    float angle_change = fabs(motor->feedback.angle_continuous_deg - initial_angle);
    
    return (angle_change > 0.1f);  // Motor should have moved at least 0.1 degrees
}

void gm6020_print_status(robomaster_motor_t* motor) {
    if (!motor) {
        printf("GM6020: Invalid motor handle\n");
        return;
    }
    
    printf("GM6020 Motor %d Status:\n", motor->config.motor_id);
    printf("  State: %s\n", robomaster_get_state_string(motor->state));
    printf("  Initialized: %s\n", motor->initialized ? "Yes" : "No");
    printf("  Feedback Valid: %s\n", motor->feedback.data_valid ? "Yes" : "No");
    
    if (motor->feedback.data_valid) {
        printf("  Angle: %.2f° (Raw: %d)\n", motor->feedback.angle_deg, motor->feedback.angle_raw);
        printf("  Continuous Angle: %.2f°\n", motor->feedback.angle_continuous_deg);
        printf("  Velocity: %.1f RPM\n", motor->feedback.velocity_rpm);
        printf("  Current: %.1f (Raw: %d)\n", motor->feedback.current_actual, motor->feedback.current_raw);
        printf("  Temperature: %d°C\n", motor->feedback.temperature);
    }
    
    printf("  Control Mode: ");
    switch (motor->config.control_mode) {
        case ROBOMASTER_CONTROL_CURRENT: printf("Current"); break;
        case ROBOMASTER_CONTROL_VELOCITY: printf("Velocity"); break;
        case ROBOMASTER_CONTROL_POSITION: printf("Position"); break;
        default: printf("Unknown"); break;
    }
    printf("\n");
    
    printf("  Target Current: %.1f\n", motor->target_current);
    printf("  Target Velocity: %.1f RPM\n", motor->target_velocity);
    printf("  Target Position: %.2f°\n", motor->target_position);
    
    printf("  Error Count: %lu\n", motor->error_count);
    printf("  Timeout Count: %lu\n", motor->timeout_count);
    printf("  Last Heartbeat: %lu ms ago\n", HAL_GetTick() - motor->last_heartbeat);
    
    printf("  Healthy: %s\n", robomaster_motor_is_healthy(motor) ? "Yes" : "No");
}