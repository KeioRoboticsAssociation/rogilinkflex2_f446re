#include "device_interfaces.h"
#include "rogilinkflex2.hpp"
#include <string.h>
#include <math.h>

// Forward declaration
bool mock_device_init(void* handle, uint8_t device_id);

// External mock device functions
extern float mock_dynamixel_read(void* handle, const char* data_type);
extern bool mock_dynamixel_write(void* handle, const char* data_type, float value);
extern float mock_robomaster_read(void* handle, const char* data_type);
extern bool mock_robomaster_write(void* handle, const char* data_type, float value);
extern float mock_temp_sensor_read(void* handle, const char* data_type);
extern bool mock_temp_sensor_write(void* handle, const char* data_type, float value);
extern float mock_encoder_read(void* handle, const char* data_type);
extern bool mock_encoder_write(void* handle, const char* data_type, float value);

// Mock device interface registry for testing
const device_interface_t device_interfaces[DEVICE_COUNT] = {
    [DEVICE_DYNAMIXEL] = {
        .init = mock_device_init,
        .read = mock_dynamixel_read,
        .write = mock_dynamixel_write
    },
    [DEVICE_ROBOMASTER] = {
        .init = mock_device_init,
        .read = mock_robomaster_read,
        .write = mock_robomaster_write
    },
    [DEVICE_SENSOR_TEMP] = {
        .init = mock_device_init,
        .read = mock_temp_sensor_read,
        .write = mock_temp_sensor_write
    },
    [DEVICE_ENCODER] = {
        .init = mock_device_init,
        .read = mock_encoder_read,
        .write = mock_encoder_write
    }
};

// Mock init function that always succeeds
bool mock_device_init(void* handle, uint8_t device_id) {
    return true; // Mock devices are always initialized successfully
}