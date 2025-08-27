#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// Device interface function pointers
typedef float (*device_read_func_t)(void* handle, const char* data_type);
typedef bool (*device_write_func_t)(void* handle, const char* data_type, float value);
typedef bool (*device_init_func_t)(void* handle, uint8_t hw_id);

// Dynamixel interface
typedef struct {
    UART_HandleTypeDef* uart;
    uint8_t motor_id;
    uint8_t tx_buffer[32];
    uint8_t rx_buffer[32];
    uint32_t last_comm_time;
    bool initialized;
} dynamixel_handle_t;

// CAN (Robomaster) interface  
typedef struct {
    CAN_HandleTypeDef* hcan;
    uint16_t motor_id;
    int16_t target_current;
    float angle;
    float speed;
    int16_t current;
    uint8_t temperature;
    uint32_t last_comm_time;
    bool initialized;
} robomaster_handle_t;

// Temperature sensor interface
typedef struct {
    I2C_HandleTypeDef* hi2c;
    uint8_t sensor_addr;
    float temperature;
    uint32_t last_read_time;
    bool initialized;
} temp_sensor_handle_t;

// Encoder interface
typedef struct {
    TIM_HandleTypeDef* htim;
    uint32_t position;
    float velocity;
    uint32_t last_position;
    uint32_t last_time;
    bool initialized;
} encoder_handle_t;

// Dynamixel Protocol 2.0 functions
bool dynamixel_init(void* handle, uint8_t motor_id);
float dynamixel_read_data(void* handle, const char* data_type);
bool dynamixel_write_data(void* handle, const char* data_type, float value);
uint16_t dynamixel_calculate_crc(uint8_t* data, uint8_t length);
bool dynamixel_send_packet(dynamixel_handle_t* dxl, uint8_t instruction, 
                          uint16_t address, uint8_t* data, uint8_t data_length);
bool dynamixel_read_packet(dynamixel_handle_t* dxl, uint8_t* data, uint8_t* length);

// CAN (Robomaster) functions
bool robomaster_init(void* handle, uint16_t motor_id);
float robomaster_read_data(void* handle, const char* data_type);
bool robomaster_write_data(void* handle, const char* data_type, float value);
void robomaster_send_current(robomaster_handle_t* rm, int16_t current);
void robomaster_process_feedback(robomaster_handle_t* rm, uint8_t* data);

// Temperature sensor functions
bool temp_sensor_init(void* handle, uint8_t sensor_addr);
float temp_sensor_read_data(void* handle, const char* data_type);
bool temp_sensor_write_data(void* handle, const char* data_type, float value);

// Encoder functions
bool encoder_init(void* handle, uint8_t channel);
float encoder_read_data(void* handle, const char* data_type);
bool encoder_write_data(void* handle, const char* data_type, float value);
void encoder_update_velocity(encoder_handle_t* enc);

// Device interface registry
typedef struct {
    device_init_func_t init;
    device_read_func_t read;
    device_write_func_t write;
} device_interface_t;

extern const device_interface_t device_interfaces[];

#ifdef __cplusplus
}
#endif