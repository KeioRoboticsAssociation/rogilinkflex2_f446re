#include "device_interfaces.h"
#include "rogilinkflex2.hpp"
#include <stdlib.h>

// Static device handle storage
static dynamixel_handle_t dynamixel_handles[8];
static robomaster_handle_t robomaster_handles[4];  
static temp_sensor_handle_t temp_sensor_handles[4];
static encoder_handle_t encoder_handles[4];

// Handle allocation counters
static uint8_t dynamixel_handle_count = 0;
static uint8_t robomaster_handle_count = 0;
static uint8_t temp_sensor_handle_count = 0;
static uint8_t encoder_handle_count = 0;

void* allocate_device_handle(device_type_t type, uint8_t hw_id) {
    switch (type) {
        case DEVICE_DYNAMIXEL:
            if (dynamixel_handle_count < 8) {
                dynamixel_handle_t* handle = &dynamixel_handles[dynamixel_handle_count++];
                if (device_interfaces[type].init && device_interfaces[type].init(handle, hw_id)) {
                    return handle;
                }
            }
            break;
            
        case DEVICE_ROBOMASTER:
            if (robomaster_handle_count < 4) {
                robomaster_handle_t* handle = &robomaster_handles[robomaster_handle_count++];
                if (device_interfaces[type].init && device_interfaces[type].init(handle, hw_id)) {
                    return handle;
                }
            }
            break;
            
        case DEVICE_SENSOR_TEMP:
            if (temp_sensor_handle_count < 4) {
                temp_sensor_handle_t* handle = &temp_sensor_handles[temp_sensor_handle_count++];
                if (device_interfaces[type].init && device_interfaces[type].init(handle, hw_id)) {
                    return handle;
                }
            }
            break;
            
        case DEVICE_ENCODER:
            if (encoder_handle_count < 4) {
                encoder_handle_t* handle = &encoder_handles[encoder_handle_count++];
                if (device_interfaces[type].init && device_interfaces[type].init(handle, hw_id)) {
                    return handle;
                }
            }
            break;
            
        default:
            break;
    }
    
    return NULL;
}

void initialize_all_device_handles(void) {
    // Initialize device handle storage
    dynamixel_handle_count = 0;
    robomaster_handle_count = 0;
    temp_sensor_handle_count = 0;
    encoder_handle_count = 0;
    
    // Register devices with actual handles instead of NULL
    void* handle;
    
    // Dynamixel motors
    handle = allocate_device_handle(DEVICE_DYNAMIXEL, 1);
    if (handle) {
        rogilinkflex2_register_device(0, DEVICE_DYNAMIXEL, handle, 1, 1);
    }
    
    handle = allocate_device_handle(DEVICE_DYNAMIXEL, 2);
    if (handle) {
        rogilinkflex2_register_device(1, DEVICE_DYNAMIXEL, handle, 1, 2);
    }
    
    // Robomaster motor
    handle = allocate_device_handle(DEVICE_ROBOMASTER, 0x201);
    if (handle) {
        rogilinkflex2_register_device(2, DEVICE_ROBOMASTER, handle, 2, 0x201);
    }
    
    // Temperature sensor
    handle = allocate_device_handle(DEVICE_SENSOR_TEMP, 0x48);
    if (handle) {
        rogilinkflex2_register_device(3, DEVICE_SENSOR_TEMP, handle, 3, 0x48);
    }
    
    // Encoders
    handle = allocate_device_handle(DEVICE_ENCODER, 0);
    if (handle) {
        rogilinkflex2_register_device(4, DEVICE_ENCODER, handle, 4, 0);
    }
    
    handle = allocate_device_handle(DEVICE_ENCODER, 1);
    if (handle) {
        rogilinkflex2_register_device(5, DEVICE_ENCODER, handle, 4, 1);
    }
}

// CAN receive callback for Robomaster feedback
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
        // Process feedback for Robomaster motors
        for (uint8_t i = 0; i < robomaster_handle_count; i++) {
            robomaster_handle_t* rm = &robomaster_handles[i];
            if (rx_header.StdId == rm->motor_id && rm->initialized) {
                robomaster_process_feedback(rm, rx_data);
                break;
            }
        }
    }
}