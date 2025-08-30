#include "device_interfaces.h"
#include "rogilinkflex2.hpp"
#include <math.h>
#include <stdlib.h>
#include <string.h>

// Mock device state structures
typedef struct {
    float angle;
    float speed;
    float target_angle;
    float torque_limit;
    bool enabled;
} mock_dynamixel_t;

typedef struct {
    float angle;
    float speed;
    float target_speed;
    float current;
    bool enabled;
} mock_robomaster_t;

typedef struct {
    float temperature;
    float humidity;
    bool active;
} mock_temp_sensor_t;

typedef struct {
    float position;
    float velocity;
    uint32_t count;
} mock_encoder_t;

// Global mock device instances
static mock_dynamixel_t mock_dynamixels[8];
static mock_robomaster_t mock_robomasters[8];
static mock_temp_sensor_t mock_temp_sensors[8];
static mock_encoder_t mock_encoders[8];

// Mock device interface functions
static float mock_dynamixel_read(void* handle, const char* data_type);
static bool mock_dynamixel_write(void* handle, const char* data_type, float value);
static float mock_robomaster_read(void* handle, const char* data_type);
static bool mock_robomaster_write(void* handle, const char* data_type, float value);
static float mock_temp_sensor_read(void* handle, const char* data_type);
static bool mock_temp_sensor_write(void* handle, const char* data_type, float value);
static float mock_encoder_read(void* handle, const char* data_type);
static bool mock_encoder_write(void* handle, const char* data_type, float value);

void initialize_all_device_handles(void) {
    // Initialize mock devices with realistic starting values
    for (int i = 0; i < 8; i++) {
        // Initialize Dynamixel mock data
        mock_dynamixels[i].angle = 0.0f + (i * 15.0f);
        mock_dynamixels[i].speed = 0.0f;
        mock_dynamixels[i].target_angle = 0.0f;
        mock_dynamixels[i].torque_limit = 1023.0f;
        mock_dynamixels[i].enabled = true;
        
        // Initialize Robomaster mock data
        mock_robomasters[i].angle = 90.0f + (i * 20.0f);
        mock_robomasters[i].speed = 0.0f;
        mock_robomasters[i].target_speed = 0.0f;
        mock_robomasters[i].current = 0.0f;
        mock_robomasters[i].enabled = true;
        
        // Initialize temperature sensor mock data
        mock_temp_sensors[i].temperature = 25.0f + (i * 2.0f);
        mock_temp_sensors[i].humidity = 60.0f + (i * 5.0f);
        mock_temp_sensors[i].active = true;
        
        // Initialize encoder mock data
        mock_encoders[i].position = 1000.0f * i;
        mock_encoders[i].velocity = 0.0f;
        mock_encoders[i].count = 0;
    }
}

void* allocate_device_handle(device_type_t type, uint8_t hw_id) {
    // Return pointers to mock device structures
    switch (type) {
        case DEVICE_DYNAMIXEL:
            if (hw_id < 8) return &mock_dynamixels[hw_id];
            break;
        case DEVICE_ROBOMASTER:
            if (hw_id < 8) return &mock_robomasters[hw_id];
            break;
        case DEVICE_SENSOR_TEMP:
            if (hw_id < 8) return &mock_temp_sensors[hw_id];
            break;
        case DEVICE_ENCODER:
            if (hw_id < 8) return &mock_encoders[hw_id];
            break;
        default:
            break;
    }
    return NULL;
}

// Dynamixel mock functions
static float mock_dynamixel_read(void* handle, const char* data_type) {
    if (!handle) return 0.0f;
    mock_dynamixel_t* device = (mock_dynamixel_t*)handle;
    
    // Simulate realistic dynamic behavior
    static uint32_t last_update = 0;
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - last_update) / 1000.0f;
    
    // Simulate servo movement towards target
    if (dt > 0 && dt < 1.0f) {  // Reasonable time delta
        float angle_diff = device->target_angle - device->angle;
        if (fabs(angle_diff) > 0.1f) {
            float max_speed = 50.0f; // degrees per second
            float move_amount = max_speed * dt;
            if (angle_diff > 0) {
                device->angle += fmin(move_amount, angle_diff);
                device->speed = move_amount / dt;
            } else {
                device->angle += fmax(-move_amount, angle_diff);
                device->speed = -move_amount / dt;
            }
        } else {
            device->speed = 0.0f;
        }
    }
    last_update = current_time;
    
    if (strcmp(data_type, "angle") == 0) {
        return device->angle;
    } else if (strcmp(data_type, "speed") == 0) {
        return device->speed;
    } else if (strcmp(data_type, "target_angle") == 0) {
        return device->target_angle;
    } else if (strcmp(data_type, "torque_limit") == 0) {
        return device->torque_limit;
    } else if (strcmp(data_type, "enabled") == 0) {
        return device->enabled ? 1.0f : 0.0f;
    }
    
    return 0.0f;
}

static bool mock_dynamixel_write(void* handle, const char* data_type, float value) {
    if (!handle) return false;
    mock_dynamixel_t* device = (mock_dynamixel_t*)handle;
    
    if (strcmp(data_type, "target_angle") == 0) {
        device->target_angle = value;
        return true;
    } else if (strcmp(data_type, "torque_limit") == 0) {
        device->torque_limit = value;
        return true;
    } else if (strcmp(data_type, "enabled") == 0) {
        device->enabled = (value != 0.0f);
        return true;
    }
    
    return false;
}

// Robomaster mock functions
static float mock_robomaster_read(void* handle, const char* data_type) {
    if (!handle) return 0.0f;
    mock_robomaster_t* device = (mock_robomaster_t*)handle;
    
    // Simulate motor behavior
    static uint32_t last_update = 0;
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - last_update) / 1000.0f;
    
    if (dt > 0 && dt < 1.0f) {
        // Simulate speed control
        float speed_diff = device->target_speed - device->speed;
        if (fabs(speed_diff) > 1.0f) {
            float acceleration = 100.0f; // rpm per second
            float speed_change = acceleration * dt;
            if (speed_diff > 0) {
                device->speed += fmin(speed_change, speed_diff);
            } else {
                device->speed += fmax(-speed_change, speed_diff);
            }
        }
        
        // Update angle based on speed
        device->angle += device->speed * dt * 6.0f; // 6 degrees per rpm per second
        if (device->angle > 360.0f) device->angle -= 360.0f;
        if (device->angle < 0.0f) device->angle += 360.0f;
        
        // Simulate current based on load
        device->current = fabs(device->speed) * 0.1f + 0.5f;
    }
    last_update = current_time;
    
    if (strcmp(data_type, "angle") == 0) {
        return device->angle;
    } else if (strcmp(data_type, "speed") == 0) {
        return device->speed;
    } else if (strcmp(data_type, "target_speed") == 0) {
        return device->target_speed;
    } else if (strcmp(data_type, "current") == 0) {
        return device->current;
    } else if (strcmp(data_type, "enabled") == 0) {
        return device->enabled ? 1.0f : 0.0f;
    }
    
    return 0.0f;
}

static bool mock_robomaster_write(void* handle, const char* data_type, float value) {
    if (!handle) return false;
    mock_robomaster_t* device = (mock_robomaster_t*)handle;
    
    if (strcmp(data_type, "target_speed") == 0) {
        device->target_speed = value;
        return true;
    } else if (strcmp(data_type, "enabled") == 0) {
        device->enabled = (value != 0.0f);
        if (!device->enabled) {
            device->target_speed = 0.0f;
        }
        return true;
    }
    
    return false;
}

// Temperature sensor mock functions
static float mock_temp_sensor_read(void* handle, const char* data_type) {
    if (!handle) return 0.0f;
    mock_temp_sensor_t* device = (mock_temp_sensor_t*)handle;
    
    // Add some random variation to make it more realistic
    static uint32_t noise_counter = 0;
    noise_counter++;
    float noise = sin(noise_counter * 0.1f) * 0.5f;
    
    if (strcmp(data_type, "temperature") == 0) {
        return device->temperature + noise;
    } else if (strcmp(data_type, "humidity") == 0) {
        return device->humidity + noise * 2.0f;
    } else if (strcmp(data_type, "active") == 0) {
        return device->active ? 1.0f : 0.0f;
    }
    
    return 0.0f;
}

static bool mock_temp_sensor_write(void* handle, const char* data_type, float value) {
    if (!handle) return false;
    mock_temp_sensor_t* device = (mock_temp_sensor_t*)handle;
    
    if (strcmp(data_type, "active") == 0) {
        device->active = (value != 0.0f);
        return true;
    }
    
    return false;
}

// Encoder mock functions
static float mock_encoder_read(void* handle, const char* data_type) {
    if (!handle) return 0.0f;
    mock_encoder_t* device = (mock_encoder_t*)handle;
    
    // Simulate encoder counting
    static uint32_t last_update = 0;
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - last_update) / 1000.0f;
    
    if (dt > 0 && dt < 1.0f) {
        // Simulate some movement
        device->position += device->velocity * dt;
        device->count = (uint32_t)device->position;
    }
    last_update = current_time;
    
    if (strcmp(data_type, "position") == 0) {
        return device->position;
    } else if (strcmp(data_type, "velocity") == 0) {
        return device->velocity;
    } else if (strcmp(data_type, "count") == 0) {
        return (float)device->count;
    }
    
    return 0.0f;
}

static bool mock_encoder_write(void* handle, const char* data_type, float value) {
    if (!handle) return false;
    mock_encoder_t* device = (mock_encoder_t*)handle;
    
    if (strcmp(data_type, "velocity") == 0) {
        device->velocity = value;
        return true;
    } else if (strcmp(data_type, "reset") == 0) {
        device->position = 0.0f;
        device->count = 0;
        return true;
    }
    
    return false;
}