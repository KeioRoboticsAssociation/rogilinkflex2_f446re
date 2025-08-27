/**
 * @file example.c
 * @brief Example usage of RoboMaster motor library with GM6020 motors
 * 
 * This file demonstrates how to use the RoboMaster motor library for controlling
 * GM6020 motors in the RogiLinkFlex2 system. It shows basic initialization,
 * control, and feedback processing.
 */

#include "robomaster_motor.h"
#include "robomaster_gm6020.h"

// Global variables
static robomaster_motor_manager_t motor_manager;
static CAN_HandleTypeDef hcan1;  // Assume this is initialized elsewhere

// Example 1: Basic single motor control
void example_single_motor_control(void) {
    // Initialize motor manager
    if (!robomaster_manager_init(&motor_manager, &hcan1)) {
        // Handle initialization error
        return;
    }
    
    // Create GM6020 motor configuration
    robomaster_motor_config_t motor_config = GM6020_DEFAULT_CONFIG;
    motor_config.motor_id = 1;
    motor_config.hcan = &hcan1;
    
    // Add motor to manager
    if (!robomaster_manager_add_motor(&motor_manager, &motor_config)) {
        // Handle motor addition error
        return;
    }
    
    // Get motor handle
    robomaster_motor_t* motor = robomaster_manager_get_motor(&motor_manager, 1);
    if (!motor) {
        // Handle error
        return;
    }
    
    // Control loop example
    for (int i = 0; i < 1000; i++) {
        // Set different control commands
        if (i < 250) {
            // Current control - apply 5000 units of current
            robomaster_motor_send_current(motor, 5000.0f);
        } else if (i < 500) {
            // Velocity control - rotate at 100 RPM
            robomaster_motor_set_velocity(motor, 100.0f);
        } else if (i < 750) {
            // Position control - go to 90 degrees
            robomaster_motor_set_position(motor, 90.0f);
        } else {
            // Stop motor
            robomaster_motor_emergency_stop(motor);
        }
        
        // Update all motors (includes control loops)
        robomaster_manager_update_all(&motor_manager);
        
        // Print motor status every 100 iterations
        if (i % 100 == 0) {
            gm6020_print_status(motor);
        }
        
        HAL_Delay(10);  // 10ms loop
    }
}

// Example 2: Multiple motor control
void example_multiple_motor_control(void) {
    // Initialize motor manager
    robomaster_manager_init(&motor_manager, &hcan1);
    
    // Add multiple GM6020 motors
    for (uint8_t motor_id = 1; motor_id <= 4; motor_id++) {
        robomaster_motor_config_t config = GM6020_DEFAULT_CONFIG;
        config.motor_id = motor_id;
        config.hcan = &hcan1;
        
        // Customize PID parameters for each motor if needed
        config.kp = 50.0f + (motor_id * 5.0f);  // Slightly different gains
        
        robomaster_manager_add_motor(&motor_manager, &config);
    }
    
    // Enable automatic current sending
    robomaster_manager_enable_auto_send(&motor_manager, true, 10);  // 10ms interval
    
    // Coordinate motion example - make motors follow a pattern
    float angle_targets[4] = {0.0f, 90.0f, 180.0f, 270.0f};
    
    for (int cycle = 0; cycle < 10; cycle++) {
        for (int step = 0; step < 360; step += 5) {
            // Update target positions for all motors
            for (uint8_t motor_id = 1; motor_id <= 4; motor_id++) {
                robomaster_motor_t* motor = robomaster_manager_get_motor(&motor_manager, motor_id);
                if (motor) {
                    float target_angle = angle_targets[motor_id - 1] + step;
                    if (target_angle >= 360.0f) target_angle -= 360.0f;
                    
                    robomaster_motor_set_position(motor, target_angle);
                }
            }
            
            // Update all motors
            robomaster_manager_update_all(&motor_manager);
            
            HAL_Delay(50);  // 50ms per step
        }
        
        // Print manager status after each cycle
        robomaster_manager_print_status(&motor_manager);
    }
    
    // Emergency stop all motors
    robomaster_manager_emergency_stop_all(&motor_manager);
}

// Example 3: CAN receive interrupt handler
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    
    // Get the received message
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
        // Process the message through the motor manager
        robomaster_manager_process_can_rx(&motor_manager, rx_header.StdId, rx_data);
    }
}

// Example 4: Position control with custom PID tuning
void example_position_control_with_tuning(void) {
    robomaster_manager_init(&motor_manager, &hcan1);
    
    // Create motor with custom PID parameters
    robomaster_motor_config_t config = GM6020_DEFAULT_CONFIG;
    config.motor_id = 1;
    config.hcan = &hcan1;
    config.control_mode = ROBOMASTER_CONTROL_POSITION;
    
    // Fine-tuned PID parameters for position control
    config.kp = 80.0f;   // Proportional gain
    config.ki = 0.5f;    // Integral gain  
    config.kd = 5.0f;    // Derivative gain
    config.position_deadband = 0.2f;  // 0.2 degree deadband
    
    // Safety limits
    config.enable_current_limit = true;
    config.max_current = 20000.0f;  // Limit current for safety
    config.enable_position_limit = true;
    config.min_position_deg = -90.0f;
    config.max_position_deg = 90.0f;
    
    robomaster_manager_add_motor(&motor_manager, &config);
    
    robomaster_motor_t* motor = robomaster_manager_get_motor(&motor_manager, 1);
    
    // Position control sequence
    float target_positions[] = {0.0f, 45.0f, -45.0f, 90.0f, -90.0f, 0.0f};
    uint8_t num_positions = sizeof(target_positions) / sizeof(target_positions[0]);
    
    for (uint8_t pos_idx = 0; pos_idx < num_positions; pos_idx++) {
        float target = target_positions[pos_idx];
        robomaster_motor_set_position(motor, target);
        
        // Wait for motor to reach target (with timeout)
        uint32_t start_time = HAL_GetTick();
        bool position_reached = false;
        
        while ((HAL_GetTick() - start_time) < 3000 && !position_reached) {  // 3 second timeout
            robomaster_manager_update_all(&motor_manager);
            
            // Check if position is reached
            robomaster_feedback_t feedback = robomaster_motor_get_feedback(motor);
            if (feedback.data_valid) {
                float position_error = fabsf(target - feedback.angle_continuous_deg);
                if (position_error < config.position_deadband) {
                    position_reached = true;
                }
            }
            
            HAL_Delay(10);
        }
        
        if (position_reached) {
            printf("Reached position %.1f degrees\n", target);
        } else {
            printf("Timeout reaching position %.1f degrees\n", target);
        }
        
        // Hold position for a moment
        for (int i = 0; i < 100; i++) {
            robomaster_manager_update_all(&motor_manager);
            HAL_Delay(10);
        }
    }
}

// Example 5: Velocity control with acceleration limiting
void example_velocity_control_with_ramp(void) {
    robomaster_manager_init(&motor_manager, &hcan1);
    
    robomaster_motor_config_t config = GM6020_DEFAULT_CONFIG;
    config.motor_id = 1;
    config.hcan = &hcan1;
    config.control_mode = ROBOMASTER_CONTROL_VELOCITY;
    config.kp = 10.0f;   // Lower P gain for smoother velocity control
    config.ki = 0.2f;
    config.kd = 0.1f;
    config.max_velocity = 300.0f;  // 300 RPM max
    
    robomaster_manager_add_motor(&motor_manager, &config);
    
    robomaster_motor_t* motor = robomaster_manager_get_motor(&motor_manager, 1);
    
    // Velocity ramping example
    float target_velocity = 200.0f;  // Target 200 RPM
    float current_velocity_target = 0.0f;
    float velocity_ramp_rate = 5.0f;  // 5 RPM per update
    
    // Ramp up
    while (current_velocity_target < target_velocity) {
        current_velocity_target += velocity_ramp_rate;
        if (current_velocity_target > target_velocity) {
            current_velocity_target = target_velocity;
        }
        
        robomaster_motor_set_velocity(motor, current_velocity_target);
        robomaster_manager_update_all(&motor_manager);
        
        // Print status every 20 updates
        static int update_counter = 0;
        if (++update_counter >= 20) {
            update_counter = 0;
            robomaster_feedback_t feedback = robomaster_motor_get_feedback(motor);
            printf("Target: %.1f RPM, Actual: %.1f RPM\n", 
                   current_velocity_target, feedback.velocity_rpm);
        }
        
        HAL_Delay(20);  // 20ms updates
    }
    
    // Hold velocity for 5 seconds
    for (int i = 0; i < 250; i++) {  // 250 * 20ms = 5 seconds
        robomaster_manager_update_all(&motor_manager);
        HAL_Delay(20);
    }
    
    // Ramp down
    while (current_velocity_target > 0.0f) {
        current_velocity_target -= velocity_ramp_rate;
        if (current_velocity_target < 0.0f) {
            current_velocity_target = 0.0f;
        }
        
        robomaster_motor_set_velocity(motor, current_velocity_target);
        robomaster_manager_update_all(&motor_manager);
        
        HAL_Delay(20);
    }
    
    // Final stop
    robomaster_motor_emergency_stop(motor);
}

// Example 6: Motor health monitoring
void example_motor_health_monitoring(void) {
    robomaster_manager_init(&motor_manager, &hcan1);
    
    // Add several motors
    for (uint8_t motor_id = 1; motor_id <= 4; motor_id++) {
        robomaster_motor_config_t config = GM6020_DEFAULT_CONFIG;
        config.motor_id = motor_id;
        config.hcan = &hcan1;
        robomaster_manager_add_motor(&motor_manager, &config);
    }
    
    // Monitoring loop
    for (int cycle = 0; cycle < 1000; cycle++) {
        robomaster_manager_update_all(&motor_manager);
        
        // Check health every 50 cycles
        if (cycle % 50 == 0) {
            uint8_t total_motors = robomaster_manager_get_motor_count(&motor_manager);
            uint8_t healthy_motors = robomaster_manager_get_healthy_motor_count(&motor_manager);
            
            printf("Motor Health: %d/%d motors healthy\n", healthy_motors, total_motors);
            
            // Check individual motor health
            for (uint8_t motor_id = 1; motor_id <= 4; motor_id++) {
                robomaster_motor_t* motor = robomaster_manager_get_motor(&motor_manager, motor_id);
                if (motor) {
                    if (!robomaster_motor_is_healthy(motor)) {
                        printf("  Motor %d is unhealthy: %s\n", 
                               motor_id, robomaster_get_state_string(motor->state));
                        
                        // Attempt to reset errors
                        robomaster_motor_reset_errors(motor);
                    }
                }
            }
            
            // Print full status every 10 health checks
            if ((cycle / 50) % 10 == 0) {
                robomaster_manager_print_status(&motor_manager);
            }
        }
        
        HAL_Delay(10);
    }
}

// Example 7: Integration with RogiLinkFlex2 device interface
#include "../rogilinkflex2/Inc/device_interfaces.h"

// Example device interface functions for GM6020
float robomaster_gm6020_read_data(void* handle, const char* data_type) {
    robomaster_motor_t* motor = (robomaster_motor_t*)handle;
    if (!motor) return 0.0f;
    
    robomaster_feedback_t feedback = robomaster_motor_get_feedback(motor);
    
    if (strcmp(data_type, "angle") == 0) {
        return feedback.angle_deg;
    } else if (strcmp(data_type, "angle_continuous") == 0) {
        return feedback.angle_continuous_deg;
    } else if (strcmp(data_type, "speed") == 0) {
        return feedback.velocity_rpm;
    } else if (strcmp(data_type, "current") == 0) {
        return feedback.current_actual;
    } else if (strcmp(data_type, "temperature") == 0) {
        return (float)feedback.temperature;
    }
    
    return 0.0f;
}

bool robomaster_gm6020_write_data(void* handle, const char* data_type, float value) {
    robomaster_motor_t* motor = (robomaster_motor_t*)handle;
    if (!motor) return false;
    
    if (strcmp(data_type, "current") == 0) {
        return robomaster_motor_send_current(motor, value);
    } else if (strcmp(data_type, "velocity") == 0) {
        return robomaster_motor_set_velocity(motor, value);
    } else if (strcmp(data_type, "position") == 0) {
        return robomaster_motor_set_position(motor, value);
    }
    
    return false;
}

bool robomaster_gm6020_init_device(void* handle, uint8_t hw_id) {
    // In this context, hw_id would be the motor ID
    return gm6020_init((robomaster_motor_t*)handle, hw_id, &hcan1);
}

// Device interface structure for GM6020
const device_interface_t gm6020_device_interface = {
    .init = robomaster_gm6020_init_device,
    .read = robomaster_gm6020_read_data,
    .write = robomaster_gm6020_write_data
};