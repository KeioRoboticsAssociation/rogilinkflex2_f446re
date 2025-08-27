#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "json_parser.h"
#include <stdint.h>
#include <stdbool.h>

// Maximum number of devices and configurations
#define MAX_DEVICES 32
#define MAX_PERIODIC_REQUESTS 16
#define MAX_JSON_SIZE 1024
#define UART_BUFFER_SIZE 2048

// Device types
typedef enum {
    DEVICE_DYNAMIXEL = 0,
    DEVICE_ROBOMASTER,
    DEVICE_SENSOR_TEMP,
    DEVICE_ENCODER,
    DEVICE_COUNT
} device_type_t;

// Device instance structure
typedef struct {
    device_type_t type;
    void* device_handle;
    uint8_t bus_id;
    uint16_t hw_id;
    bool active;
} device_instance_t;

// Periodic request configuration
typedef struct {
    char request_name[32];
    uint8_t instance_ids[16];
    uint8_t instance_count;
    char data_names[8][16];
    uint8_t data_count;
    uint32_t interval_ms;
    uint32_t next_send_time;
    bool active;
} periodic_request_t;

// Communication message types
typedef enum {
    MSG_TYPE_CONFIGURATION = 0,
    MSG_TYPE_READ,
    MSG_TYPE_WRITE,
    MSG_TYPE_READ_RESPONSE,
    MSG_TYPE_PERIODIC_DATA,
    MSG_TYPE_ERROR
} message_type_t;

// Communication functions
void rogilinkflex2_init(void);
void rogilinkflex2_process(void);
void rogilinkflex2_register_device(uint8_t id, device_type_t type, void* handle, uint8_t bus_id, uint16_t hw_id);
device_instance_t* rogilinkflex2_get_device(uint8_t id);

// UART communication functions
void uart_send_json(const char* json_str);
void uart_receive_process(void);
void uart_send_response(uint8_t device_id, const char* data_type, float value, const char* status);
void uart_send_periodic_data(const char* request_name, uint8_t device_count, 
                            const uint8_t* device_ids, const float* values);
void uart_send_periodic_data_multi(const char* request_name, uint8_t device_count,
                                  const uint8_t* device_ids, const char data_names[][16],
                                  const float* values, uint8_t data_count);

// JSON processing functions
bool parse_json_message(const char* json_str);
void process_configuration_message(const char* json_str);
void process_read_message(const char* json_str);
void process_write_message(const char* json_str);

// Enhanced configuration parsing
bool parse_periodic_requests_config(const char* json_str);
bool validate_periodic_request(const char* request_name, const uint8_t* instance_ids, 
                              uint8_t instance_count, const char data_names[][16],
                              uint8_t data_count, uint32_t interval_ms);
void clear_all_periodic_requests(void);
bool add_periodic_request_from_config(const char* request_name, const uint8_t* instance_ids,
                                     uint8_t instance_count, const char data_names[][16],
                                     uint8_t data_count, uint32_t interval_ms);

// JSON array parsing helpers
json_token_t* json_get_array_element(const char* json_str, json_token_t* array_token, int index);
int json_get_array_size(json_token_t* array_token);
bool json_parse_string_array(const char* json_str, json_token_t* array_token, 
                            char result[][16], uint8_t max_count, uint8_t* actual_count);
bool json_parse_int_array(const char* json_str, json_token_t* array_token,
                         uint8_t* result, uint8_t max_count, uint8_t* actual_count);

// Device interface functions
float read_device_data(uint8_t device_id, const char* data_type);
bool write_device_data(uint8_t device_id, const char* data_type, float value);

// Device handle management
void* allocate_device_handle(device_type_t type, uint8_t hw_id);
void initialize_all_device_handles(void);

// Periodic task management
void periodic_task_init(void);
void periodic_task_process(void);
void add_periodic_request(const char* request_name, const uint8_t* instance_ids, 
                         uint8_t instance_count, uint32_t interval_ms);
void add_periodic_request_multi(const char* request_name, const uint8_t* instance_ids,
                               uint8_t instance_count, const char data_names[][16],
                               uint8_t data_count, uint32_t interval_ms);

// Error handling
void handle_communication_error(const char* error_msg);
void handle_device_error(uint8_t device_id, const char* error_msg);

#ifdef __cplusplus
}
#endif