#include "device_interfaces.h"
#include "rogilinkflex2.hpp"
#include <string.h>
#include <math.h>

// External HAL handles (to be defined in main.c)
extern UART_HandleTypeDef huart1;  // For Dynamixel
extern CAN_HandleTypeDef hcan1;    // For Robomaster motors
extern I2C_HandleTypeDef hi2c1;    // For temperature sensors
extern TIM_HandleTypeDef htim2;    // For encoders

// Device interface registry
const device_interface_t device_interfaces[DEVICE_COUNT] = {
    [DEVICE_DYNAMIXEL] = {
        .init = dynamixel_init,
        .read = dynamixel_read_data,
        .write = dynamixel_write_data
    },
    [DEVICE_ROBOMASTER] = {
        .init = robomaster_init,
        .read = robomaster_read_data,
        .write = robomaster_write_data
    },
    [DEVICE_SENSOR_TEMP] = {
        .init = temp_sensor_init,
        .read = temp_sensor_read_data,
        .write = temp_sensor_write_data
    },
    [DEVICE_ENCODER] = {
        .init = encoder_init,
        .read = encoder_read_data,
        .write = encoder_write_data
    }
};

// ==== DYNAMIXEL IMPLEMENTATION ====

bool dynamixel_init(void* handle, uint8_t motor_id) {
    dynamixel_handle_t* dxl = (dynamixel_handle_t*)handle;
    if (!dxl) return false;
    
    dxl->uart = &huart1;
    dxl->motor_id = motor_id;
    dxl->last_comm_time = HAL_GetTick();
    dxl->initialized = true;
    
    // Send ping packet to check if motor is responding
    uint8_t ping_data = 0;
    return dynamixel_send_packet(dxl, 0x01, 0x0000, &ping_data, 0);
}

float dynamixel_read_data(void* handle, const char* data_type) {
    dynamixel_handle_t* dxl = (dynamixel_handle_t*)handle;
    if (!dxl || !dxl->initialized) return 0.0f;
    
    uint16_t address = 0;
    uint8_t length = 4;
    
    if (strcmp(data_type, "angle") == 0) {
        address = 132;  // Present Position
    } else if (strcmp(data_type, "speed") == 0) {
        address = 128;  // Present Velocity
    } else if (strcmp(data_type, "torque_limit") == 0) {
        address = 102;  // Torque Limit
    } else if (strcmp(data_type, "current") == 0) {
        address = 126;  // Present Current
    } else {
        return 0.0f;
    }
    
    // Send read packet
    uint8_t read_length[2] = {(uint8_t)(length & 0xFF), (uint8_t)(length >> 8)};
    if (!dynamixel_send_packet(dxl, 0x02, address, read_length, 2)) {
        return 0.0f;
    }
    
    // Read response
    uint8_t response_data[8];
    uint8_t response_length;
    if (!dynamixel_read_packet(dxl, response_data, &response_length)) {
        return 0.0f;
    }
    
    // Convert raw data to meaningful value
    if (response_length >= 4) {
        uint32_t raw_value = (response_data[3] << 24) | (response_data[2] << 16) | 
                            (response_data[1] << 8) | response_data[0];
        
        if (strcmp(data_type, "angle") == 0) {
            // Convert to degrees (assuming 4096 positions per revolution)
            return (float)raw_value * 360.0f / 4096.0f;
        } else if (strcmp(data_type, "speed") == 0) {
            // Convert to RPM
            return (float)(int32_t)raw_value * 0.229f;  // 0.229 RPM per unit
        } else {
            return (float)raw_value;
        }
    }
    
    return 0.0f;
}

bool dynamixel_write_data(void* handle, const char* data_type, float value) {
    dynamixel_handle_t* dxl = (dynamixel_handle_t*)handle;
    if (!dxl || !dxl->initialized) return false;
    
    uint16_t address = 0;
    uint32_t raw_value = 0;
    
    if (strcmp(data_type, "torque_limit") == 0) {
        address = 102;  // Torque Limit
        raw_value = (uint32_t)value;
    } else if (strcmp(data_type, "target_angle") == 0) {
        address = 116;  // Goal Position
        raw_value = (uint32_t)(value * 4096.0f / 360.0f);  // Convert degrees to position units
    } else if (strcmp(data_type, "target_speed") == 0) {
        address = 112;  // Goal Velocity
        raw_value = (uint32_t)(value / 0.229f);  // Convert RPM to velocity units
    } else {
        return false;
    }
    
    // Prepare data bytes
    uint8_t data[4] = {
        (uint8_t)(raw_value & 0xFF),
        (uint8_t)((raw_value >> 8) & 0xFF),
        (uint8_t)((raw_value >> 16) & 0xFF),
        (uint8_t)((raw_value >> 24) & 0xFF)
    };
    
    return dynamixel_send_packet(dxl, 0x03, address, data, 4);
}

bool dynamixel_send_packet(dynamixel_handle_t* dxl, uint8_t instruction, 
                          uint16_t address, uint8_t* data, uint8_t data_length) {
    if (!dxl || !dxl->uart) return false;
    
    uint8_t packet_length = data_length + 7;  // Header + ID + Length + Instruction + Address + CRC
    uint8_t packet[32];
    
    // Build packet
    packet[0] = 0xFF;  // Header
    packet[1] = 0xFF;  // Header
    packet[2] = 0xFD;  // Header
    packet[3] = 0x00;  // Reserved
    packet[4] = dxl->motor_id;  // ID
    packet[5] = (packet_length - 7) & 0xFF;        // Length L
    packet[6] = ((packet_length - 7) >> 8) & 0xFF; // Length H
    packet[7] = instruction;  // Instruction
    packet[8] = address & 0xFF;        // Address L
    packet[9] = (address >> 8) & 0xFF; // Address H
    
    // Copy data
    if (data && data_length > 0) {
        memcpy(&packet[10], data, data_length);
    }
    
    // Calculate CRC
    uint16_t crc = dynamixel_calculate_crc(packet, packet_length - 2);
    packet[packet_length - 2] = crc & 0xFF;
    packet[packet_length - 1] = (crc >> 8) & 0xFF;
    
    // Send packet
    HAL_StatusTypeDef status = HAL_UART_Transmit(dxl->uart, packet, packet_length, 100);
    
    dxl->last_comm_time = HAL_GetTick();
    return (status == HAL_OK);
}

bool dynamixel_read_packet(dynamixel_handle_t* dxl, uint8_t* data, uint8_t* length) {
    if (!dxl || !dxl->uart || !data || !length) return false;
    
    uint8_t packet[32];
    
    // Read response packet
    HAL_StatusTypeDef status = HAL_UART_Receive(dxl->uart, packet, sizeof(packet), 100);
    if (status != HAL_OK) return false;
    
    // Validate header
    if (packet[0] != 0xFF || packet[1] != 0xFF || packet[2] != 0xFD) {
        return false;
    }
    
    // Extract data length
    uint16_t packet_length = packet[5] | (packet[6] << 8);
    if (packet_length < 4) return false;
    
    // Copy parameter data (skip instruction, error, and CRC)
    uint8_t param_length = packet_length - 4;
    *length = param_length;
    if (param_length > 0) {
        memcpy(data, &packet[9], param_length);
    }
    
    return true;
}

uint16_t dynamixel_calculate_crc(uint8_t* data, uint8_t length) {
    uint16_t crc = 0x0000;
    uint16_t crc_table[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        // ... (complete CRC table would be here for production code)
    };
    
    for (uint8_t i = 0; i < length; i++) {
        uint16_t tbl_idx = ((crc >> 8) ^ data[i]) & 0xFF;
        crc = ((crc << 8) ^ crc_table[tbl_idx]) & 0xFFFF;
    }
    
    return crc;
}

// ==== ROBOMASTER (CAN) IMPLEMENTATION ====

bool robomaster_init(void* handle, uint16_t motor_id) {
    robomaster_handle_t* rm = (robomaster_handle_t*)handle;
    if (!rm) return false;
    
    rm->hcan = &hcan1;
    rm->motor_id = motor_id;
    rm->target_current = 0;
    rm->angle = 0.0f;
    rm->speed = 0.0f;
    rm->current = 0;
    rm->temperature = 0;
    rm->last_comm_time = HAL_GetTick();
    rm->initialized = true;
    
    return true;
}

float robomaster_read_data(void* handle, const char* data_type) {
    robomaster_handle_t* rm = (robomaster_handle_t*)handle;
    if (!rm || !rm->initialized) return 0.0f;
    
    if (strcmp(data_type, "angle") == 0) {
        return rm->angle;
    } else if (strcmp(data_type, "speed") == 0) {
        return rm->speed;
    } else if (strcmp(data_type, "current") == 0) {
        return (float)rm->current;
    } else if (strcmp(data_type, "temperature") == 0) {
        return (float)rm->temperature;
    }
    
    return 0.0f;
}

bool robomaster_write_data(void* handle, const char* data_type, float value) {
    robomaster_handle_t* rm = (robomaster_handle_t*)handle;
    if (!rm || !rm->initialized) return false;
    
    if (strcmp(data_type, "target_current") == 0) {
        rm->target_current = (int16_t)value;
        robomaster_send_current(rm, rm->target_current);
        return true;
    }
    
    return false;
}

void robomaster_send_current(robomaster_handle_t* rm, int16_t current) {
    if (!rm || !rm->hcan) return;
    
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8] = {0};
    uint32_t tx_mailbox;
    
    // Configure CAN message
    tx_header.StdId = 0x200;  // Standard ID for motor control
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.IDE = CAN_ID_STD;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;
    
    // Set current based on motor ID
    switch (rm->motor_id) {
        case 0x201:
            tx_data[0] = (current >> 8) & 0xFF;
            tx_data[1] = current & 0xFF;
            break;
        case 0x202:
            tx_data[2] = (current >> 8) & 0xFF;
            tx_data[3] = current & 0xFF;
            break;
        case 0x203:
            tx_data[4] = (current >> 8) & 0xFF;
            tx_data[5] = current & 0xFF;
            break;
        case 0x204:
            tx_data[6] = (current >> 8) & 0xFF;
            tx_data[7] = current & 0xFF;
            break;
        default:
            return;
    }
    
    HAL_CAN_AddTxMessage(rm->hcan, &tx_header, tx_data, &tx_mailbox);
    rm->last_comm_time = HAL_GetTick();
}

void robomaster_process_feedback(robomaster_handle_t* rm, uint8_t* data) {
    if (!rm || !data) return;
    
    // Process feedback data from CAN message
    rm->angle = ((int16_t)((data[0] << 8) | data[1])) * 360.0f / 8192.0f;  // Convert to degrees
    rm->speed = (int16_t)((data[2] << 8) | data[3]);  // RPM
    rm->current = (int16_t)((data[4] << 8) | data[5]);  // Current in mA
    rm->temperature = data[6];  // Temperature in °C
    
    rm->last_comm_time = HAL_GetTick();
}

// ==== TEMPERATURE SENSOR IMPLEMENTATION ====

bool temp_sensor_init(void* handle, uint8_t sensor_addr) {
    temp_sensor_handle_t* sensor = (temp_sensor_handle_t*)handle;
    if (!sensor) return false;
    
    sensor->hi2c = &hi2c1;
    sensor->sensor_addr = sensor_addr << 1;  // Convert to I2C address format
    sensor->temperature = 0.0f;
    sensor->last_read_time = HAL_GetTick();
    sensor->initialized = true;
    
    return true;
}

float temp_sensor_read_data(void* handle, const char* data_type) {
    temp_sensor_handle_t* sensor = (temp_sensor_handle_t*)handle;
    if (!sensor || !sensor->initialized) return 0.0f;
    
    if (strcmp(data_type, "temperature") == 0) {
        // Read temperature from sensor (assuming MCP9808 or similar)
        uint8_t reg_addr = 0x05;  // Temperature register
        uint8_t data[2];
        
        HAL_StatusTypeDef status = HAL_I2C_Mem_Read(sensor->hi2c, sensor->sensor_addr,
                                                   reg_addr, 1, data, 2, 100);
        
        if (status == HAL_OK) {
            // Convert raw data to temperature
            uint16_t raw_temp = (data[0] << 8) | data[1];
            sensor->temperature = (raw_temp & 0x0FFF) * 0.0625f;
            
            if (raw_temp & 0x1000) {
                sensor->temperature -= 256.0f;  // Handle negative temperatures
            }
            
            sensor->last_read_time = HAL_GetTick();
        }
        
        return sensor->temperature;
    }
    
    return 0.0f;
}

bool temp_sensor_write_data(void* handle, const char* data_type, float value) {
    // Temperature sensors are typically read-only
    return false;
}

// ==== ENCODER IMPLEMENTATION ====

bool encoder_init(void* handle, uint8_t channel) {
    encoder_handle_t* enc = (encoder_handle_t*)handle;
    if (!enc) return false;
    
    enc->htim = &htim2;
    enc->position = 0;
    enc->velocity = 0.0f;
    enc->last_position = 0;
    enc->last_time = HAL_GetTick();
    enc->initialized = true;
    
    // Start encoder timer
    HAL_TIM_Encoder_Start(enc->htim, TIM_CHANNEL_ALL);
    
    return true;
}

float encoder_read_data(void* handle, const char* data_type) {
    encoder_handle_t* enc = (encoder_handle_t*)handle;
    if (!enc || !enc->initialized) return 0.0f;
    
    if (strcmp(data_type, "position") == 0) {
        enc->position = __HAL_TIM_GET_COUNTER(enc->htim);
        return (float)enc->position;
    } else if (strcmp(data_type, "velocity") == 0) {
        encoder_update_velocity(enc);
        return enc->velocity;
    }
    
    return 0.0f;
}

bool encoder_write_data(void* handle, const char* data_type, float value) {
    encoder_handle_t* enc = (encoder_handle_t*)handle;
    if (!enc || !enc->initialized) return false;
    
    if (strcmp(data_type, "reset_position") == 0) {
        __HAL_TIM_SET_COUNTER(enc->htim, 0);
        enc->position = 0;
        enc->last_position = 0;
        return true;
    }
    
    return false;
}

void encoder_update_velocity(encoder_handle_t* enc) {
    if (!enc) return;
    
    uint32_t current_time = HAL_GetTick();
    uint32_t current_position = __HAL_TIM_GET_COUNTER(enc->htim);
    
    if (current_time != enc->last_time) {
        int32_t position_diff = (int32_t)current_position - (int32_t)enc->last_position;
        uint32_t time_diff = current_time - enc->last_time;
        
        // Calculate velocity in positions per second
        enc->velocity = (float)position_diff * 1000.0f / (float)time_diff;
        
        enc->last_position = current_position;
        enc->last_time = current_time;
    }
}