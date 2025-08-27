#include "robomaster_motor.h"
#include "robomaster_gm6020.h"
#include <string.h>

bool robomaster_manager_init(robomaster_motor_manager_t* manager, CAN_HandleTypeDef* hcan) {
    if (!manager || !hcan) {
        return false;
    }
    
    // Clear manager structure
    memset(manager, 0, sizeof(robomaster_motor_manager_t));
    
    // Set CAN handle
    manager->hcan = hcan;
    manager->motor_count = 0;
    manager->auto_send_enabled = false;
    manager->send_interval_ms = 10;  // Default 10ms send interval (100Hz)
    manager->last_send_time = HAL_GetTick();
    
    return true;
}

bool robomaster_manager_add_motor(robomaster_motor_manager_t* manager, const robomaster_motor_config_t* config) {
    if (!manager || !config || manager->motor_count >= ROBOMASTER_MAX_MOTORS) {
        return false;
    }
    
    // Check if motor ID already exists
    for (uint8_t i = 0; i < manager->motor_count; i++) {
        if (manager->motors[i].config.motor_id == config->motor_id) {
            return false;  // Motor ID already in use
        }
    }
    
    // Initialize new motor
    robomaster_motor_t* motor = &manager->motors[manager->motor_count];
    if (!robomaster_motor_init(motor, config)) {
        return false;
    }
    
    manager->motor_count++;
    return true;
}

bool robomaster_manager_remove_motor(robomaster_motor_manager_t* manager, uint8_t motor_id) {
    if (!manager) {
        return false;
    }
    
    // Find motor with matching ID
    for (uint8_t i = 0; i < manager->motor_count; i++) {
        if (manager->motors[i].config.motor_id == motor_id) {
            // Deinitialize motor
            robomaster_motor_deinit(&manager->motors[i]);
            
            // Shift remaining motors down
            for (uint8_t j = i; j < manager->motor_count - 1; j++) {
                manager->motors[j] = manager->motors[j + 1];
            }
            
            manager->motor_count--;
            
            // Clear last slot
            memset(&manager->motors[manager->motor_count], 0, sizeof(robomaster_motor_t));
            
            return true;
        }
    }
    
    return false;  // Motor not found
}

robomaster_motor_t* robomaster_manager_get_motor(robomaster_motor_manager_t* manager, uint8_t motor_id) {
    if (!manager) {
        return NULL;
    }
    
    // Find motor with matching ID
    for (uint8_t i = 0; i < manager->motor_count; i++) {
        if (manager->motors[i].config.motor_id == motor_id) {
            return &manager->motors[i];
        }
    }
    
    return NULL;  // Motor not found
}

bool robomaster_manager_send_all_currents(robomaster_motor_manager_t* manager) {
    if (!manager || !manager->hcan) {
        return false;
    }
    
    // Group motors by their CAN control ID
    // GM6020 motors 1-4 use 0x1FF
    // M3508/M2006 motors 1-4 use 0x200, motors 5-8 use 0x1FF
    
    float gm6020_currents_1ff[4] = {0.0f, 0.0f, 0.0f, 0.0f};  // GM6020 motors 1-4
    float m3508_currents_200[4] = {0.0f, 0.0f, 0.0f, 0.0f};   // M3508/M2006 motors 1-4
    float m3508_currents_1ff[4] = {0.0f, 0.0f, 0.0f, 0.0f};   // M3508/M2006 motors 5-8
    
    bool send_gm6020_1ff = false;
    bool send_m3508_200 = false;
    bool send_m3508_1ff = false;
    
    // Collect currents from all motors
    for (uint8_t i = 0; i < manager->motor_count; i++) {
        robomaster_motor_t* motor = &manager->motors[i];
        
        if (!motor->initialized) continue;
        
        uint8_t motor_id = motor->config.motor_id;
        float current = motor->target_current;
        
        switch (motor->config.type) {
            case ROBOMASTER_MOTOR_GM6020:
                if (motor_id >= 1 && motor_id <= 4) {
                    gm6020_currents_1ff[motor_id - 1] = current;
                    send_gm6020_1ff = true;
                }
                break;
                
            case ROBOMASTER_MOTOR_M3508:
            case ROBOMASTER_MOTOR_M2006:
                if (motor_id >= 1 && motor_id <= 4) {
                    m3508_currents_200[motor_id - 1] = current;
                    send_m3508_200 = true;
                } else if (motor_id >= 5 && motor_id <= 8) {
                    m3508_currents_1ff[motor_id - 5] = current;
                    send_m3508_1ff = true;
                }
                break;
                
            default:
                break;
        }
    }
    
    bool all_success = true;
    
    // Send CAN messages
    if (send_gm6020_1ff) {
        if (!gm6020_send_current_group(manager->hcan, gm6020_currents_1ff[0], 
                                       gm6020_currents_1ff[1], gm6020_currents_1ff[2], 
                                       gm6020_currents_1ff[3])) {
            all_success = false;
        }
    }
    
    if (send_m3508_200) {
        if (!robomaster_manager_send_m3508_group(manager->hcan, 0x200, m3508_currents_200)) {
            all_success = false;
        }
    }
    
    if (send_m3508_1ff) {
        if (!robomaster_manager_send_m3508_group(manager->hcan, 0x1FF, m3508_currents_1ff)) {
            all_success = false;
        }
    }
    
    manager->last_send_time = HAL_GetTick();
    
    return all_success;
}

bool robomaster_manager_send_m3508_group(CAN_HandleTypeDef* hcan, uint32_t can_id, float currents[4]) {
    if (!hcan) {
        return false;
    }
    
    // Convert currents to raw values
    int16_t raw_currents[4] = {
        robomaster_current_to_raw(currents[0], ROBOMASTER_MOTOR_M3508),
        robomaster_current_to_raw(currents[1], ROBOMASTER_MOTOR_M3508),
        robomaster_current_to_raw(currents[2], ROBOMASTER_MOTOR_M3508),
        robomaster_current_to_raw(currents[3], ROBOMASTER_MOTOR_M3508)
    };
    
    // Prepare CAN message
    CAN_TxHeaderTypeDef tx_header;
    tx_header.StdId = can_id;
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;
    
    uint8_t tx_data[8];
    
    // Pack current data
    tx_data[0] = (raw_currents[0] >> 8) & 0xFF;  // Motor 1 high byte
    tx_data[1] = raw_currents[0] & 0xFF;         // Motor 1 low byte
    tx_data[2] = (raw_currents[1] >> 8) & 0xFF;  // Motor 2 high byte
    tx_data[3] = raw_currents[1] & 0xFF;         // Motor 2 low byte
    tx_data[4] = (raw_currents[2] >> 8) & 0xFF;  // Motor 3 high byte
    tx_data[5] = raw_currents[2] & 0xFF;         // Motor 3 low byte
    tx_data[6] = (raw_currents[3] >> 8) & 0xFF;  // Motor 4 high byte
    tx_data[7] = raw_currents[3] & 0xFF;         // Motor 4 low byte
    
    // Send CAN message
    uint32_t mailbox;
    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &mailbox);
    
    return (status == HAL_OK);
}

void robomaster_manager_update_all(robomaster_motor_manager_t* manager) {
    if (!manager) return;
    
    uint32_t current_time = HAL_GetTick();
    
    // Update all motors
    for (uint8_t i = 0; i < manager->motor_count; i++) {
        robomaster_motor_t* motor = &manager->motors[i];
        
        // Check for timeouts
        if (current_time - motor->last_heartbeat > ROBOMASTER_CAN_TIMEOUT) {
            motor->state = ROBOMASTER_STATE_TIMEOUT;
            motor->timeout_count++;
        }
        
        // Run control loop for each motor
        robomaster_motor_control_loop(motor);
    }
    
    // Auto-send currents if enabled and interval has passed
    if (manager->auto_send_enabled && 
        (current_time - manager->last_send_time) >= manager->send_interval_ms) {
        robomaster_manager_send_all_currents(manager);
    }
}

void robomaster_manager_emergency_stop_all(robomaster_motor_manager_t* manager) {
    if (!manager) return;
    
    // Stop all motors immediately
    for (uint8_t i = 0; i < manager->motor_count; i++) {
        robomaster_motor_emergency_stop(&manager->motors[i]);
    }
    
    // Send zero currents to all groups
    float zero_currents[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    
    // Send to all possible CAN IDs
    gm6020_send_current_group(manager->hcan, 0.0f, 0.0f, 0.0f, 0.0f);
    robomaster_manager_send_m3508_group(manager->hcan, 0x200, zero_currents);
    robomaster_manager_send_m3508_group(manager->hcan, 0x1FF, zero_currents);
}

void robomaster_manager_enable_auto_send(robomaster_motor_manager_t* manager, bool enable, uint16_t interval_ms) {
    if (!manager) return;
    
    manager->auto_send_enabled = enable;
    if (interval_ms > 0) {
        manager->send_interval_ms = interval_ms;
    }
}

bool robomaster_manager_process_can_rx(robomaster_motor_manager_t* manager, uint32_t can_id, uint8_t* data) {
    if (!manager || !data) {
        return false;
    }
    
    // Determine which motor this feedback is for based on CAN ID
    uint8_t motor_id = 0;
    robomaster_motor_type_t motor_type = ROBOMASTER_MOTOR_UNKNOWN;
    
    // GM6020 feedback IDs: 0x205-0x208 for motors 1-4
    if (can_id >= 0x205 && can_id <= 0x208) {
        motor_id = can_id - 0x205 + 1;
        motor_type = ROBOMASTER_MOTOR_GM6020;
    }
    // M3508/M2006 feedback IDs: 0x201-0x208 for motors 1-8
    else if (can_id >= 0x201 && can_id <= 0x208) {
        motor_id = can_id - 0x201 + 1;
        motor_type = ROBOMASTER_MOTOR_M3508;  // Could also be M2006
    }
    else {
        return false;  // Unknown CAN ID
    }
    
    // Find the motor in our manager
    robomaster_motor_t* motor = NULL;
    for (uint8_t i = 0; i < manager->motor_count; i++) {
        if (manager->motors[i].config.motor_id == motor_id) {
            // Additional check for GM6020 to ensure type match
            if (motor_type == ROBOMASTER_MOTOR_GM6020 && 
                manager->motors[i].config.type != ROBOMASTER_MOTOR_GM6020) {
                continue;
            }
            motor = &manager->motors[i];
            break;
        }
    }
    
    if (!motor) {
        return false;  // Motor not found in manager
    }
    
    // Update motor feedback based on type
    switch (motor->config.type) {
        case ROBOMASTER_MOTOR_GM6020:
            return gm6020_parse_feedback(motor, data);
            
        case ROBOMASTER_MOTOR_M3508:
        case ROBOMASTER_MOTOR_M2006:
            return robomaster_motor_update_feedback(motor, data);
            
        default:
            return false;
    }
}

uint8_t robomaster_manager_get_motor_count(robomaster_motor_manager_t* manager) {
    return manager ? manager->motor_count : 0;
}

uint8_t robomaster_manager_get_healthy_motor_count(robomaster_motor_manager_t* manager) {
    if (!manager) return 0;
    
    uint8_t healthy_count = 0;
    for (uint8_t i = 0; i < manager->motor_count; i++) {
        if (robomaster_motor_is_healthy(&manager->motors[i])) {
            healthy_count++;
        }
    }
    
    return healthy_count;
}

void robomaster_manager_print_status(robomaster_motor_manager_t* manager) {
    if (!manager) {
        printf("RoboMaster Manager: Invalid handle\n");
        return;
    }
    
    printf("RoboMaster Motor Manager Status:\n");
    printf("  Motor Count: %d/%d\n", manager->motor_count, ROBOMASTER_MAX_MOTORS);
    printf("  Healthy Motors: %d\n", robomaster_manager_get_healthy_motor_count(manager));
    printf("  Auto Send: %s (Interval: %d ms)\n", 
           manager->auto_send_enabled ? "Enabled" : "Disabled",
           manager->send_interval_ms);
    printf("  Last Send: %lu ms ago\n", HAL_GetTick() - manager->last_send_time);
    
    printf("\nMotor Details:\n");
    for (uint8_t i = 0; i < manager->motor_count; i++) {
        robomaster_motor_t* motor = &manager->motors[i];
        printf("  Motor %d (%s): %s - %s\n", 
               motor->config.motor_id,
               robomaster_get_motor_type_string(motor->config.type),
               robomaster_get_state_string(motor->state),
               robomaster_motor_is_healthy(motor) ? "Healthy" : "Unhealthy");
    }
}

void robomaster_manager_reset_all_errors(robomaster_motor_manager_t* manager) {
    if (!manager) return;
    
    for (uint8_t i = 0; i < manager->motor_count; i++) {
        robomaster_motor_reset_errors(&manager->motors[i]);
    }
}