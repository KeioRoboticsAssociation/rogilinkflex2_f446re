#include "rogilinkflex2.hpp"
#include "device_interfaces.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// Global variables
static device_instance_t device_table[MAX_DEVICES];
static periodic_request_t periodic_requests[MAX_PERIODIC_REQUESTS];
static uint8_t device_count = 0;
static uint8_t periodic_request_count = 0;

// UART communication buffers
static char uart_rx_buffer[UART_BUFFER_SIZE];
static char uart_tx_buffer[UART_BUFFER_SIZE];
static volatile uint16_t uart_rx_index = 0;
static bool message_ready = false;

// External HAL objects
extern UART_HandleTypeDef huart2;

// Private function declarations
static void process_uart_message(const char* message);
static float simulate_device_read(uint8_t device_id, const char* data_type);
static bool simulate_device_write(uint8_t device_id, const char* data_type, float value);

void rogilinkflex2_init(void) {
    // Initialize device table
    memset(device_table, 0, sizeof(device_table));
    memset(periodic_requests, 0, sizeof(periodic_requests));
    device_count = 0;
    periodic_request_count = 0;
    
    // Initialize UART receive interrupt
    HAL_UART_Receive_IT(&huart2, (uint8_t*)&uart_rx_buffer[uart_rx_index], 1);
    
    // Initialize periodic task timer
    periodic_task_init();
    
    // Initialize device handles and register devices with actual handles
    initialize_all_device_handles();
}

void rogilinkflex2_process(void) {
    // Process UART messages
    uart_receive_process();
    
    // Process periodic tasks
    periodic_task_process();
}

void rogilinkflex2_register_device(uint8_t id, device_type_t type, void* handle, uint8_t bus_id, uint16_t hw_id) {
    if (id >= MAX_DEVICES) return;
    
    device_table[id].type = type;
    device_table[id].device_handle = handle;
    device_table[id].bus_id = bus_id;
    device_table[id].hw_id = hw_id;
    device_table[id].active = true;
    
    if (id >= device_count) {
        device_count = id + 1;
    }
}

device_instance_t* rogilinkflex2_get_device(uint8_t id) {
    if (id >= MAX_DEVICES || !device_table[id].active) {
        return NULL;
    }
    return &device_table[id];
}

void uart_send_json(const char* json_str) {
    // Send JSON string followed by newline
    HAL_UART_Transmit(&huart2, (uint8_t*)json_str, strlen(json_str), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t*)"\n", 1, HAL_MAX_DELAY);
}

void uart_receive_process(void) {
    if (message_ready) {
        uart_rx_buffer[uart_rx_index] = '\0';  // Null terminate
        process_uart_message(uart_rx_buffer);
        
        // Reset for next message
        uart_rx_index = 0;
        message_ready = false;
        HAL_UART_Receive_IT(&huart2, (uint8_t*)&uart_rx_buffer[uart_rx_index], 1);
    }
}

void uart_send_response(uint8_t device_id, const char* data_type, float value, const char* status) {
    char json_buffer[MAX_JSON_SIZE];
    int pos = 0;
    
    json_build_object_start(json_buffer, MAX_JSON_SIZE, &pos);
    json_build_string(json_buffer, MAX_JSON_SIZE, &pos, "type", "read_response");
    json_build_int(json_buffer, MAX_JSON_SIZE, &pos, "id", device_id);
    json_build_string(json_buffer, MAX_JSON_SIZE, &pos, "data_type", data_type);
    json_build_float(json_buffer, MAX_JSON_SIZE, &pos, "value", value);
    json_build_string(json_buffer, MAX_JSON_SIZE, &pos, "status", status);
    json_build_object_end(json_buffer, MAX_JSON_SIZE, &pos);
    
    uart_send_json(json_buffer);
}

void uart_send_periodic_data_multi(const char* request_name, uint8_t device_count,
                                  const uint8_t* device_ids, const char data_names[][16],
                                  const float* values, uint8_t data_count) {
    char json_buffer[MAX_JSON_SIZE];
    int pos = 0;
    
    json_build_object_start(json_buffer, MAX_JSON_SIZE, &pos);
    json_build_string(json_buffer, MAX_JSON_SIZE, &pos, "type", "periodic_data");
    json_build_string(json_buffer, MAX_JSON_SIZE, &pos, "request", request_name);
    json_build_array_start(json_buffer, MAX_JSON_SIZE, &pos, "data");
    
    for (uint8_t i = 0; i < device_count; i++) {
        if (pos < MAX_JSON_SIZE - 100) {  // Safety check
            if (i > 0) {
                json_buffer[pos++] = ',';
            }
            json_buffer[pos++] = '{';
            
            // Add device ID
            int len = snprintf(json_buffer + pos, MAX_JSON_SIZE - pos, "\"id\":%d", device_ids[i]);
            if (len > 0 && pos + len < MAX_JSON_SIZE) pos += len;
            
            // Add data values
            for (uint8_t j = 0; j < data_count; j++) {
                if (pos < MAX_JSON_SIZE - 50) {
                    json_buffer[pos++] = ',';
                    len = snprintf(json_buffer + pos, MAX_JSON_SIZE - pos, "\"%s\":%.2f", 
                                  data_names[j], values[i * data_count + j]);
                    if (len > 0 && pos + len < MAX_JSON_SIZE) pos += len;
                }
            }
            
            json_buffer[pos++] = '}';
        }
    }
    
    json_build_array_end(json_buffer, MAX_JSON_SIZE, &pos);
    json_build_object_end(json_buffer, MAX_JSON_SIZE, &pos);
    
    uart_send_json(json_buffer);
}

void uart_send_periodic_data(const char* request_name, uint8_t device_count, 
                            const uint8_t* device_ids, const float* values) {
    // Legacy function for backward compatibility
    char data_names[1][16];
    strcpy(data_names[0], "value");
    uart_send_periodic_data_multi(request_name, device_count, device_ids, data_names, values, 1);
}

bool parse_json_message(const char* json_str) {
    json_parser_t parser;
    json_token_t tokens[JSON_MAX_TOKENS];
    
    json_init(&parser);
    int token_count = json_parse(&parser, json_str, strlen(json_str), tokens, JSON_MAX_TOKENS);
    
    if (token_count < 0) {
        handle_communication_error("JSON parse error");
        return false;
    }
    
    // Find message type
    json_token_t* type_token = json_find_token(json_str, tokens, token_count, "type");
    if (!type_token) {
        handle_communication_error("Missing message type");
        return false;
    }
    
    char type_str[32];
    json_get_string(json_str, type_token, type_str, sizeof(type_str));
    
    if (strcmp(type_str, "configuration") == 0) {
        process_configuration_message(json_str);
    } else if (strcmp(type_str, "read") == 0) {
        process_read_message(json_str);
    } else if (strcmp(type_str, "write") == 0) {
        process_write_message(json_str);
    } else {
        handle_communication_error("Unknown message type");
        return false;
    }
    
    return true;
}

void process_configuration_message(const char* json_str) {
    json_parser_t parser;
    json_token_t tokens[JSON_MAX_TOKENS];
    
    json_init(&parser);
    int token_count = json_parse(&parser, json_str, strlen(json_str), tokens, JSON_MAX_TOKENS);
    
    if (token_count < 0) return;
    
    // Find periodic_requests array
    json_token_t* requests_token = json_find_token(json_str, tokens, token_count, "periodic_requests");
    if (!requests_token || requests_token->type != JSON_ARRAY) {
        return;
    }
    
    // Clear existing periodic requests
    periodic_request_count = 0;
    memset(periodic_requests, 0, sizeof(periodic_requests));
    
    // For now, add default periodic requests based on specification
    // Motor angle and speed monitoring
    if (periodic_request_count < MAX_PERIODIC_REQUESTS) {
        periodic_request_t* req = &periodic_requests[periodic_request_count];
        strcpy(req->request_name, "motor_angle_speed");
        req->instance_ids[0] = 0;  // dynamixel_1
        req->instance_ids[1] = 1;  // dynamixel_2  
        req->instance_ids[2] = 2;  // robomaster_1
        req->instance_count = 3;
        req->data_count = 2;
        strcpy(req->data_names[0], "angle");
        strcpy(req->data_names[1], "speed");
        req->interval_ms = 500;
        req->next_send_time = HAL_GetTick() + req->interval_ms;
        req->active = true;
        periodic_request_count++;
    }
    
    // Temperature sensor monitoring
    if (periodic_request_count < MAX_PERIODIC_REQUESTS) {
        periodic_request_t* req = &periodic_requests[periodic_request_count];
        strcpy(req->request_name, "sensor_temperature");
        req->instance_ids[0] = 3;  // temp_sensor_1
        req->instance_count = 1;
        req->data_count = 1;
        strcpy(req->data_names[0], "temperature");
        req->interval_ms = 1000;
        req->next_send_time = HAL_GetTick() + req->interval_ms;
        req->active = true;
        periodic_request_count++;
    }
    
    // Encoder position and velocity monitoring
    if (periodic_request_count < MAX_PERIODIC_REQUESTS) {
        periodic_request_t* req = &periodic_requests[periodic_request_count];
        strcpy(req->request_name, "encoder_position");
        req->instance_ids[0] = 4;  // encoder_left
        req->instance_ids[1] = 5;  // encoder_right
        req->instance_count = 2;
        req->data_count = 2;
        strcpy(req->data_names[0], "position");
        strcpy(req->data_names[1], "velocity");
        req->interval_ms = 100;
        req->next_send_time = HAL_GetTick() + req->interval_ms;
        req->active = true;
        periodic_request_count++;
    }
}

void process_read_message(const char* json_str) {
    json_parser_t parser;
    json_token_t tokens[JSON_MAX_TOKENS];
    
    json_init(&parser);
    int token_count = json_parse(&parser, json_str, strlen(json_str), tokens, JSON_MAX_TOKENS);
    
    if (token_count < 0) return;
    
    // Get device ID and data type
    json_token_t* id_token = json_find_token(json_str, tokens, token_count, "id");
    json_token_t* data_type_token = json_find_token(json_str, tokens, token_count, "data_type");
    
    if (!id_token || !data_type_token) {
        handle_communication_error("Invalid read message format");
        return;
    }
    
    uint8_t device_id = json_get_int(json_str, id_token);
    char data_type[32];
    json_get_string(json_str, data_type_token, data_type, sizeof(data_type));
    
    // Read data from device
    float value = read_device_data(device_id, data_type);
    
    // Send response
    uart_send_response(device_id, data_type, value, "success");
}

void process_write_message(const char* json_str) {
    json_parser_t parser;
    json_token_t tokens[JSON_MAX_TOKENS];
    
    json_init(&parser);
    int token_count = json_parse(&parser, json_str, strlen(json_str), tokens, JSON_MAX_TOKENS);
    
    if (token_count < 0) return;
    
    // Get device ID, data type, and value
    json_token_t* id_token = json_find_token(json_str, tokens, token_count, "id");
    json_token_t* data_type_token = json_find_token(json_str, tokens, token_count, "data_type");
    json_token_t* value_token = json_find_token(json_str, tokens, token_count, "value");
    
    if (!id_token || !data_type_token || !value_token) {
        handle_communication_error("Invalid write message format");
        return;
    }
    
    uint8_t device_id = json_get_int(json_str, id_token);
    char data_type[32];
    json_get_string(json_str, data_type_token, data_type, sizeof(data_type));
    float value = json_get_float(json_str, value_token);
    
    // Write data to device
    bool success = write_device_data(device_id, data_type, value);
    
    if (!success) {
        handle_device_error(device_id, "Write operation failed");
    }
}

float read_device_data(uint8_t device_id, const char* data_type) {
    device_instance_t* device = rogilinkflex2_get_device(device_id);
    if (!device || device->type >= DEVICE_COUNT) {
        return 0.0f;
    }
    
    // Use actual device interface if available and device handle exists
    if (device->device_handle && device_interfaces[device->type].read) {
        return device_interfaces[device->type].read(device->device_handle, data_type);
    }
    
    // Fallback to simulation
    return simulate_device_read(device_id, data_type);
}

bool write_device_data(uint8_t device_id, const char* data_type, float value) {
    device_instance_t* device = rogilinkflex2_get_device(device_id);
    if (!device || device->type >= DEVICE_COUNT) {
        return false;
    }
    
    // Use actual device interface if available and device handle exists
    if (device->device_handle && device_interfaces[device->type].write) {
        return device_interfaces[device->type].write(device->device_handle, data_type, value);
    }
    
    // Fallback to simulation
    return simulate_device_write(device_id, data_type, value);
}

void periodic_task_init(void) {
    // Initialize periodic task system
    // In a real implementation, this would set up FreeRTOS tasks or hardware timers
}

void periodic_task_process(void) {
    uint32_t current_time = HAL_GetTick();
    
    for (uint8_t i = 0; i < periodic_request_count; i++) {
        periodic_request_t* req = &periodic_requests[i];
        
        if (req->active && current_time >= req->next_send_time) {
            // Collect data from all instances
            uint8_t device_ids[16];
            float values[16 * 8];  // Max 8 data types per device
            uint8_t valid_count = 0;
            
            for (uint8_t j = 0; j < req->instance_count && j < 16; j++) {
                uint8_t device_id = req->instance_ids[j];
                if (rogilinkflex2_get_device(device_id)) {
                    device_ids[valid_count] = device_id;
                    
                    // Read all data types for this device
                    for (uint8_t k = 0; k < req->data_count && k < 8; k++) {
                        values[valid_count * req->data_count + k] = 
                            read_device_data(device_id, req->data_names[k]);
                    }
                    valid_count++;
                }
            }
            
            if (valid_count > 0) {
                uart_send_periodic_data_multi(req->request_name, valid_count, device_ids,
                                            req->data_names, values, req->data_count);
            }
            
            // Schedule next transmission
            req->next_send_time = current_time + req->interval_ms;
        }
    }
}

void add_periodic_request(const char* request_name, const uint8_t* instance_ids, 
                         uint8_t instance_count, uint32_t interval_ms) {
    if (periodic_request_count >= MAX_PERIODIC_REQUESTS) {
        return;
    }
    
    periodic_request_t* req = &periodic_requests[periodic_request_count];
    strcpy(req->request_name, request_name);
    
    for (uint8_t i = 0; i < instance_count && i < 16; i++) {
        req->instance_ids[i] = instance_ids[i];
    }
    req->instance_count = instance_count;
    req->interval_ms = interval_ms;
    req->next_send_time = HAL_GetTick() + interval_ms;
    req->active = true;
    
    periodic_request_count++;
}

void handle_communication_error(const char* error_msg) {
    // Send error response
    char json_buffer[MAX_JSON_SIZE];
    int pos = 0;
    
    json_build_object_start(json_buffer, MAX_JSON_SIZE, &pos);
    json_build_string(json_buffer, MAX_JSON_SIZE, &pos, "type", "error");
    json_build_string(json_buffer, MAX_JSON_SIZE, &pos, "message", error_msg);
    json_build_object_end(json_buffer, MAX_JSON_SIZE, &pos);
    
    uart_send_json(json_buffer);
}

void handle_device_error(uint8_t device_id, const char* error_msg) {
    // Send device-specific error
    char json_buffer[MAX_JSON_SIZE];
    int pos = 0;
    
    json_build_object_start(json_buffer, MAX_JSON_SIZE, &pos);
    json_build_string(json_buffer, MAX_JSON_SIZE, &pos, "type", "device_error");
    json_build_int(json_buffer, MAX_JSON_SIZE, &pos, "device_id", device_id);
    json_build_string(json_buffer, MAX_JSON_SIZE, &pos, "message", error_msg);
    json_build_object_end(json_buffer, MAX_JSON_SIZE, &pos);
    
    uart_send_json(json_buffer);
}

// UART interrupt callback
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2) {
        char received_char = uart_rx_buffer[uart_rx_index];
        
        if (received_char == '\n' || received_char == '\r') {
            // End of message
            message_ready = true;
        } else {
            // Continue receiving
            uart_rx_index++;
            if (uart_rx_index >= UART_BUFFER_SIZE - 1) {
                // Buffer overflow, reset
                uart_rx_index = 0;
            }
            HAL_UART_Receive_IT(&huart2, (uint8_t*)&uart_rx_buffer[uart_rx_index], 1);
        }
    }
}

// Private helper functions
static void process_uart_message(const char* message) {
    parse_json_message(message);
}

static float simulate_device_read(uint8_t device_id, const char* data_type) {
    // Simulate device readings based on device type and data type
    device_instance_t* device = rogilinkflex2_get_device(device_id);
    if (!device) return 0.0f;
    
    switch (device->type) {
        case DEVICE_DYNAMIXEL:
            if (strcmp(data_type, "angle") == 0) {
                return 45.5f + (device_id * 10.0f);  // Simulated angle
            } else if (strcmp(data_type, "speed") == 0) {
                return 30.0f + (device_id * 5.0f);   // Simulated speed
            } else if (strcmp(data_type, "torque_limit") == 0) {
                return 1023.0f;                      // Simulated torque limit
            }
            break;
            
        case DEVICE_ROBOMASTER:
            if (strcmp(data_type, "angle") == 0) {
                return 90.0f + (device_id * 15.0f);  // Simulated angle
            } else if (strcmp(data_type, "speed") == 0) {
                return 50.0f + (device_id * 8.0f);   // Simulated speed
            }
            break;
            
        case DEVICE_SENSOR_TEMP:
            if (strcmp(data_type, "temperature") == 0) {
                return 25.5f + (device_id * 2.0f);   // Simulated temperature
            }
            break;
            
        case DEVICE_ENCODER:
            if (strcmp(data_type, "position") == 0) {
                return 1000.0f + (device_id * 100.0f); // Simulated position
            } else if (strcmp(data_type, "velocity") == 0) {
                return 10.5f + (device_id * 1.5f);     // Simulated velocity
            }
            break;
            
        default:
            break;
    }
    
    return 0.0f;
}

static bool simulate_device_write(uint8_t device_id, const char* data_type, float value) {
    // Simulate device writes
    device_instance_t* device = rogilinkflex2_get_device(device_id);
    if (!device) return false;
    
    // For simulation, all writes succeed
    // In a real implementation, this would communicate with actual devices
    return true;
}