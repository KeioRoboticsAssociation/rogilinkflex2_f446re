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
    // initialize_all_device_handles();  // Commented out for testing with mock devices
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
    // Ensure the JSON string is properly terminated
    size_t json_len = strlen(json_str);
    if (json_len == 0) {
        return; // Don't send empty messages
    }
    
    // Send JSON string followed by newline in one transmission
    // Create a temporary buffer to hold JSON + newline
    static char uart_tx_complete[MAX_JSON_SIZE + 2];
    if (json_len < MAX_JSON_SIZE) {
        memcpy(uart_tx_complete, json_str, json_len);
        uart_tx_complete[json_len] = '\n';
        uart_tx_complete[json_len + 1] = '\0';
        
        // Send the complete message in one UART transmission
        HAL_UART_Transmit(&huart2, (uint8_t*)uart_tx_complete, json_len + 1, HAL_MAX_DELAY);
    } else {
        // Fallback for oversized messages
        HAL_UART_Transmit(&huart2, (uint8_t*)json_str, json_len, HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart2, (uint8_t*)"\n", 1, HAL_MAX_DELAY);
    }
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
    
    // Initialize buffer
    memset(json_buffer, 0, MAX_JSON_SIZE);
    
    json_build_object_start(json_buffer, MAX_JSON_SIZE, &pos);
    json_build_string(json_buffer, MAX_JSON_SIZE, &pos, "type", "read_response");
    json_build_int(json_buffer, MAX_JSON_SIZE, &pos, "id", device_id);
    json_build_string(json_buffer, MAX_JSON_SIZE, &pos, "data_type", data_type);
    json_build_float(json_buffer, MAX_JSON_SIZE, &pos, "value", value);
    json_build_string(json_buffer, MAX_JSON_SIZE, &pos, "status", status);
    json_build_object_end(json_buffer, MAX_JSON_SIZE, &pos);
    
    // Ensure buffer is null-terminated
    if (pos < MAX_JSON_SIZE) {
        json_buffer[pos] = '\0';
    } else {
        json_buffer[MAX_JSON_SIZE - 1] = '\0';
        pos = MAX_JSON_SIZE - 1;
    }
    
    // Validate JSON structure before sending
    bool json_valid = false;
    if (pos >= 10) {
        // Check if JSON structure is complete
        int last_char_pos = pos - 1;
        while (last_char_pos >= 0 && (json_buffer[last_char_pos] == '\0' || json_buffer[last_char_pos] == ' ')) {
            last_char_pos--;
        }
        
        if (last_char_pos >= 0 && json_buffer[last_char_pos] == '}') {
            // Also check for required fields
            if (json_buffer[0] == '{' && 
                strstr(json_buffer, "\"type\":") != NULL &&
                strstr(json_buffer, "\"id\":") != NULL &&
                strstr(json_buffer, "\"value\":") != NULL) {
                json_valid = true;
            }
        }
    }
    
    if (json_valid) {
        uart_send_json(json_buffer);
    } else {
        // Send a simple error response instead of incomplete JSON
        char error_buffer[256];
        snprintf(error_buffer, sizeof(error_buffer), 
                "{\"type\":\"error\",\"message\":\"Response too large\",\"device_id\":%d}", 
                device_id);
        uart_send_json(error_buffer);
    }
}

void uart_send_periodic_data_multi(const char* request_name, uint8_t device_count,
                                  const uint8_t* device_ids, const char data_names[][16],
                                  const float* values, uint8_t data_count) {
    char json_buffer[MAX_JSON_SIZE];
    int pos = 0;
    
    // Initialize buffer
    memset(json_buffer, 0, MAX_JSON_SIZE);
    
    // Build JSON using the reliable JSON builder functions
    json_build_object_start(json_buffer, MAX_JSON_SIZE, &pos);
    json_build_string(json_buffer, MAX_JSON_SIZE, &pos, "type", "periodic_data");
    json_build_string(json_buffer, MAX_JSON_SIZE, &pos, "request", request_name);
    json_build_array_start(json_buffer, MAX_JSON_SIZE, &pos, "data");
    
    uint8_t devices_processed = 0;
    for (uint8_t i = 0; i < device_count; i++) {
        // Calculate space needed for this device entry (more accurate estimate)
        int space_needed = 50 + (data_count * 25); // Base + data fields
        if (pos + space_needed + 20 >= MAX_JSON_SIZE) { // Extra safety margin
            break; // Stop if we don't have enough space
        }
        
        // Add comma separator between array elements
        if (devices_processed > 0) {
            json_buffer[pos++] = ',';
        }
        
        // Start device object
        json_buffer[pos++] = '{';
        
        // Add device ID
        int len = snprintf(json_buffer + pos, MAX_JSON_SIZE - pos - 10, "\"id\":%d", device_ids[i]);
        if (len > 0 && pos + len < MAX_JSON_SIZE - 10) {
            pos += len;
        } else {
            // Rollback - remove the opening brace
            pos--;
            if (devices_processed > 0) pos--; // Also remove comma
            break;
        }
        
        // Add data values for this device
        uint8_t fields_added = 0;
        for (uint8_t j = 0; j < data_count; j++) {
            // Check if we have space for this field (more conservative)
            if (pos + 30 >= MAX_JSON_SIZE) {
                break;
            }
            
            // Add comma before field
            json_buffer[pos++] = ',';
            
            // Add the data field with bounds checking
            len = snprintf(json_buffer + pos, MAX_JSON_SIZE - pos - 10, "\"%s\":%.2f", 
                          data_names[j], values[i * data_count + j]);
            if (len > 0 && pos + len < MAX_JSON_SIZE - 10) {
                pos += len;
                fields_added++;
            } else {
                // Rollback - remove the comma
                pos--;
                break;
            }
        }
        
        // Only close device object if we added at least some fields
        if (fields_added > 0) {
            json_buffer[pos++] = '}';
            devices_processed++;
        } else {
            // Rollback the entire device entry
            // Find the start of this device entry and remove it
            while (pos > 0 && json_buffer[pos-1] != '{') pos--;
            if (pos > 0) pos--; // Remove the opening brace
            if (devices_processed > 0 && pos > 0) pos--; // Remove comma if not first
            break;
        }
    }
    
    // Close the JSON structure
    json_build_array_end(json_buffer, MAX_JSON_SIZE, &pos);
    json_build_object_end(json_buffer, MAX_JSON_SIZE, &pos);
    
    // Ensure buffer is null-terminated and validate JSON structure
    if (pos < MAX_JSON_SIZE) {
        json_buffer[pos] = '\0';
    } else {
        json_buffer[MAX_JSON_SIZE - 1] = '\0';
        pos = MAX_JSON_SIZE - 1;
    }
    
    // Validate that JSON is properly closed before sending
    bool json_valid = false;
    if (devices_processed > 0 && pos >= 10) {
        // Check if JSON structure is complete: should end with }
        int last_char_pos = pos - 1;
        while (last_char_pos >= 0 && (json_buffer[last_char_pos] == '\0' || json_buffer[last_char_pos] == ' ')) {
            last_char_pos--;
        }
        
        if (last_char_pos >= 0 && json_buffer[last_char_pos] == '}') {
            // Also check that we have proper opening structure
            if (json_buffer[0] == '{' && strstr(json_buffer, "\"type\":") != NULL) {
                json_valid = true;
            }
        }
    }
    
    // Only send if we have a complete and valid JSON structure
    if (json_valid) {
        uart_send_json(json_buffer);
    } else {
        // Send error message if JSON is incomplete or no devices could fit
        if (devices_processed == 0) {
            handle_communication_error("Periodic data too large for buffer");
        } else {
            handle_communication_error("JSON structure incomplete, data truncated");
        }
    }
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
    
    if (token_count < 0) {
        handle_communication_error("Failed to parse configuration JSON");
        return;
    }
    
    // Parse and apply periodic requests configuration
    if (!parse_periodic_requests_config(json_str)) {
        handle_communication_error("Failed to parse periodic requests configuration");
        return;
    }
    
    // Send confirmation response
    char response[MAX_JSON_SIZE];
    int pos = 0;
    json_build_object_start(response, MAX_JSON_SIZE, &pos);
    json_build_string(response, MAX_JSON_SIZE, &pos, "type", "configuration_ack");
    json_build_int(response, MAX_JSON_SIZE, &pos, "periodic_requests_count", periodic_request_count);
    json_build_string(response, MAX_JSON_SIZE, &pos, "status", "success");
    json_build_object_end(response, MAX_JSON_SIZE, &pos);
    uart_send_json(response);
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
    req->data_count = 1;  // Default single data type
    strcpy(req->data_names[0], "value");
    req->interval_ms = interval_ms;
    req->next_send_time = HAL_GetTick() + interval_ms;
    req->active = true;
    
    periodic_request_count++;
}

void add_periodic_request_multi(const char* request_name, const uint8_t* instance_ids,
                               uint8_t instance_count, const char data_names[][16],
                               uint8_t data_count, uint32_t interval_ms) {
    if (periodic_request_count >= MAX_PERIODIC_REQUESTS) {
        return;
    }
    
    periodic_request_t* req = &periodic_requests[periodic_request_count];
    strcpy(req->request_name, request_name);
    
    for (uint8_t i = 0; i < instance_count && i < 16; i++) {
        req->instance_ids[i] = instance_ids[i];
    }
    req->instance_count = instance_count;
    
    for (uint8_t i = 0; i < data_count && i < 8; i++) {
        strcpy(req->data_names[i], data_names[i]);
    }
    req->data_count = data_count;
    
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

// ========== ENHANCED CONFIGURATION PARSING ==========

bool parse_periodic_requests_config(const char* json_str) {
    json_parser_t parser;
    json_token_t tokens[JSON_MAX_TOKENS];
    
    json_init(&parser);
    int token_count = json_parse(&parser, json_str, strlen(json_str), tokens, JSON_MAX_TOKENS);
    
    if (token_count < 0) return false;
    
    // Find periodic_requests array
    json_token_t* requests_token = json_find_token(json_str, tokens, token_count, "periodic_requests");
    if (!requests_token || requests_token->type != JSON_ARRAY) {
        return false;
    }
    
    // Clear existing periodic requests
    clear_all_periodic_requests();
    
    // Parse each periodic request in the array
    int array_size = json_get_array_size(requests_token);
    for (int i = 0; i < array_size && periodic_request_count < MAX_PERIODIC_REQUESTS; i++) {
        json_token_t* request_obj = json_get_array_element(json_str, requests_token, i);
        if (!request_obj || request_obj->type != JSON_OBJECT) {
            continue;
        }
        
        // Parse individual request object
        char request_name[32];
        uint8_t instance_ids[16];
        uint8_t instance_count = 0;
        char data_names[8][16];
        uint8_t data_count = 0;
        uint32_t interval_ms = 1000; // Default interval
        
        // Extract request name
        json_token_t* name_token = json_find_token(json_str, tokens, token_count, "request");
        if (name_token && name_token->type == JSON_STRING) {
            json_get_string(json_str, name_token, request_name, sizeof(request_name));
        } else {
            continue; // Skip if no request name
        }
        
        // Extract instance IDs array
        json_token_t* ids_token = json_find_token(json_str, tokens, token_count, "instance_ids");
        if (ids_token && ids_token->type == JSON_ARRAY) {
            if (!json_parse_int_array(json_str, ids_token, instance_ids, 16, &instance_count)) {
                continue;
            }
        } else {
            continue; // Skip if no instance IDs
        }
        
        // Extract data names array
        json_token_t* data_names_token = json_find_token(json_str, tokens, token_count, "data_names");
        if (data_names_token && data_names_token->type == JSON_ARRAY) {
            if (!json_parse_string_array(json_str, data_names_token, data_names, 8, &data_count)) {
                continue;
            }
        } else {
            data_count = 1;
            strcpy(data_names[0], "value"); // Default data name
        }
        
        // Extract interval
        json_token_t* interval_token = json_find_token(json_str, tokens, token_count, "interval_ms");
        if (interval_token && interval_token->type == JSON_PRIMITIVE) {
            interval_ms = (uint32_t)json_get_int(json_str, interval_token);
        }
        
        // Validate and add the periodic request
        if (validate_periodic_request(request_name, instance_ids, instance_count, data_names, data_count, interval_ms)) {
            add_periodic_request_from_config(request_name, instance_ids, instance_count, data_names, data_count, interval_ms);
        }
    }
    
    return true;
}

bool validate_periodic_request(const char* request_name, const uint8_t* instance_ids, 
                              uint8_t instance_count, const char data_names[][16],
                              uint8_t data_count, uint32_t interval_ms) {
    // Validate request name
    if (!request_name || strlen(request_name) == 0 || strlen(request_name) >= 32) {
        return false;
    }
    
    // Validate instance count
    if (instance_count == 0 || instance_count > 16) {
        return false;
    }
    
    // Validate data count
    if (data_count == 0 || data_count > 8) {
        return false;
    }
    
    // Validate interval
    if (interval_ms < 10 || interval_ms > 60000) { // 10ms to 60s range
        return false;
    }
    
    // Validate that all instance IDs refer to active devices
    for (uint8_t i = 0; i < instance_count; i++) {
        device_instance_t* device = rogilinkflex2_get_device(instance_ids[i]);
        if (!device || !device->active) {
            return false;
        }
    }
    
    // Validate data names are not empty
    for (uint8_t i = 0; i < data_count; i++) {
        if (strlen(data_names[i]) == 0) {
            return false;
        }
    }
    
    return true;
}

void clear_all_periodic_requests(void) {
    periodic_request_count = 0;
    memset(periodic_requests, 0, sizeof(periodic_requests));
}

bool add_periodic_request_from_config(const char* request_name, const uint8_t* instance_ids,
                                     uint8_t instance_count, const char data_names[][16],
                                     uint8_t data_count, uint32_t interval_ms) {
    if (periodic_request_count >= MAX_PERIODIC_REQUESTS) {
        return false;
    }
    
    periodic_request_t* req = &periodic_requests[periodic_request_count];
    
    // Copy request name
    strncpy(req->request_name, request_name, sizeof(req->request_name) - 1);
    req->request_name[sizeof(req->request_name) - 1] = '\0';
    
    // Copy instance IDs
    for (uint8_t i = 0; i < instance_count && i < 16; i++) {
        req->instance_ids[i] = instance_ids[i];
    }
    req->instance_count = instance_count;
    
    // Copy data names
    for (uint8_t i = 0; i < data_count && i < 8; i++) {
        strncpy(req->data_names[i], data_names[i], sizeof(req->data_names[i]) - 1);
        req->data_names[i][sizeof(req->data_names[i]) - 1] = '\0';
    }
    req->data_count = data_count;
    
    // Set timing and activation
    req->interval_ms = interval_ms;
    req->next_send_time = HAL_GetTick() + interval_ms;
    req->active = true;
    
    periodic_request_count++;
    return true;
}

// ========== JSON ARRAY PARSING HELPERS ==========

json_token_t* json_get_array_element(const char* json_str, json_token_t* array_token, int index) {
    if (!array_token || array_token->type != JSON_ARRAY || index < 0) {
        return NULL;
    }
    
    if (index >= array_token->size) {
        return NULL;
    }
    
    // Find the element at the specified index by traversing children
    // This assumes tokens are ordered and children follow parents
    int array_token_index = array_token - (json_token_t*)0; // Get token index (simplified)
    int current_element = 0;
    
    // Start from the token right after the array token
    json_token_t* current_token = array_token + 1;
    
    for (int i = 0; i < 100 && current_token; i++) { // Safety limit
        if (current_token->parent == array_token_index) {
            if (current_element == index) {
                return current_token;
            }
            current_element++;
        }
        current_token++;
        
        // Safety check - don't go beyond array bounds
        if (current_token->start >= array_token->end) {
            break;
        }
    }
    
    return NULL;
}

int json_get_array_size(json_token_t* array_token) {
    if (!array_token || array_token->type != JSON_ARRAY) {
        return 0;
    }
    return array_token->size;
}

bool json_parse_string_array(const char* json_str, json_token_t* array_token, 
                            char result[][16], uint8_t max_count, uint8_t* actual_count) {
    if (!array_token || array_token->type != JSON_ARRAY || !result || !actual_count) {
        return false;
    }
    
    int array_size = json_get_array_size(array_token);
    *actual_count = 0;
    
    for (int i = 0; i < array_size && *actual_count < max_count; i++) {
        json_token_t* element = json_get_array_element(json_str, array_token, i);
        if (element && element->type == JSON_STRING) {
            if (json_get_string(json_str, element, result[*actual_count], 16) > 0) {
                (*actual_count)++;
            }
        }
    }
    
    return *actual_count > 0;
}

bool json_parse_int_array(const char* json_str, json_token_t* array_token,
                         uint8_t* result, uint8_t max_count, uint8_t* actual_count) {
    if (!array_token || array_token->type != JSON_ARRAY || !result || !actual_count) {
        return false;
    }
    
    int array_size = json_get_array_size(array_token);
    *actual_count = 0;
    
    for (int i = 0; i < array_size && *actual_count < max_count; i++) {
        json_token_t* element = json_get_array_element(json_str, array_token, i);
        if (element && element->type == JSON_PRIMITIVE) {
            int value = json_get_int(json_str, element);
            if (value >= 0 && value <= 255) {
                result[*actual_count] = (uint8_t)value;
                (*actual_count)++;
            }
        }
    }
    
    return *actual_count > 0;
}