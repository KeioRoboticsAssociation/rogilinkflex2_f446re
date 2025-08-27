# RoboMaster Motor Library

A flexible C library for controlling RoboMaster motors (GM6020, M3508, M2006) via CAN bus on STM32 platforms, designed specifically for the RogiLinkFlex2 system.

## Features

- **Multi-Motor Support**: Control up to 8 RoboMaster motors simultaneously
- **Flexible Motor Types**: GM6020 (with built-in encoder), M3508, M2006
- **Multiple Control Modes**: Current, velocity, and position control with PID
- **Motor Manager**: Centralized management for multiple motors
- **Safety Features**: Current limiting, timeout detection, emergency stop
- **Real-time Feedback**: Angle, velocity, current, and temperature monitoring
- **Integration Ready**: Designed to integrate with RogiLinkFlex2 device interfaces

## Supported Motors

### GM6020 Brushless DC Motor
- **Resolution**: 13-bit encoder (8192 counts/revolution)
- **Control Range**: ±30000 current units
- **CAN IDs**: 
  - Control: 0x1FF (motors 1-4)
  - Feedback: 0x205-0x208 (motors 1-4)
- **Features**: Built-in position sensor, high precision control

### M3508 & M2006 (Future Support)
- Similar CAN protocol structure
- Different specifications and current ranges

## Quick Start

### 1. Basic Single Motor Control

```c
#include "robomaster_gm6020.h"

// Initialize motor manager
robomaster_motor_manager_t manager;
robomaster_manager_init(&manager, &hcan1);

// Add GM6020 motor
robomaster_motor_config_t config = GM6020_DEFAULT_CONFIG;
config.motor_id = 1;
config.hcan = &hcan1;
robomaster_manager_add_motor(&manager, &config);

// Get motor handle
robomaster_motor_t* motor = robomaster_manager_get_motor(&manager, 1);

// Control motor
robomaster_motor_send_current(motor, 5000.0f);  // Current control
robomaster_motor_set_velocity(motor, 100.0f);   // Velocity control
robomaster_motor_set_position(motor, 90.0f);    // Position control
```

### 2. Multiple Motor Coordinated Control

```c
// Initialize manager
robomaster_motor_manager_t manager;
robomaster_manager_init(&manager, &hcan1);

// Add multiple motors
for (uint8_t id = 1; id <= 4; id++) {
    robomaster_motor_config_t config = GM6020_DEFAULT_CONFIG;
    config.motor_id = id;
    config.hcan = &hcan1;
    robomaster_manager_add_motor(&manager, &config);
}

// Enable automatic current sending
robomaster_manager_enable_auto_send(&manager, true, 10);  // 10ms interval

// Main control loop
while (1) {
    // Set individual motor targets
    robomaster_motor_set_position(robomaster_manager_get_motor(&manager, 1), 45.0f);
    robomaster_motor_set_position(robomaster_manager_get_motor(&manager, 2), -45.0f);
    
    // Update all motors (handles control loops and CAN sending)
    robomaster_manager_update_all(&manager);
    
    HAL_Delay(10);
}
```

### 3. CAN Interrupt Handler

```c
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
        // Process motor feedback
        robomaster_manager_process_can_rx(&manager, rx_header.StdId, rx_data);
    }
}
```

## Configuration

### Motor Configuration Structure

```c
typedef struct {
    robomaster_motor_type_t type;         // Motor type (GM6020, M3508, M2006)
    uint8_t motor_id;                     // Motor ID (1-8)
    CAN_HandleTypeDef* hcan;              // CAN handle
    robomaster_control_mode_t control_mode; // Control mode
    
    // Control parameters
    float max_current;                    // Current limit
    float max_velocity;                   // Velocity limit (RPM)
    float position_deadband;              // Position deadband (degrees)
    
    // PID parameters
    float kp, ki, kd;                     // PID gains
    
    // Safety limits
    bool enable_current_limit;
    bool enable_velocity_limit;
    bool enable_position_limit;
    float min_position_deg, max_position_deg;
} robomaster_motor_config_t;
```

### Default GM6020 Configuration

```c
const robomaster_motor_config_t GM6020_DEFAULT_CONFIG = {
    .type = ROBOMASTER_MOTOR_GM6020,
    .control_mode = ROBOMASTER_CONTROL_CURRENT,
    .max_current = 24000.0f,              // 80% of max for safety
    .max_velocity = 600.0f,               // 600 RPM
    .position_deadband = 0.5f,            // 0.5 degrees
    .kp = 50.0f, .ki = 0.1f, .kd = 2.0f, // Default PID gains
    .enable_current_limit = true,
    .enable_velocity_limit = true,
    .enable_position_limit = false,
    .min_position_deg = -180.0f,
    .max_position_deg = 180.0f
};
```

## Control Modes

### 1. Current Control (Direct Torque)
```c
motor->config.control_mode = ROBOMASTER_CONTROL_CURRENT;
robomaster_motor_send_current(motor, 5000.0f);  // ±30000 range
```

### 2. Velocity Control (RPM)
```c
motor->config.control_mode = ROBOMASTER_CONTROL_VELOCITY;
robomaster_motor_set_velocity(motor, 100.0f);   // 100 RPM
```

### 3. Position Control (Degrees)
```c
motor->config.control_mode = ROBOMASTER_CONTROL_POSITION;
robomaster_motor_set_position(motor, 90.0f);    // 90 degrees
```

## Feedback Data

Access real-time motor feedback:

```c
robomaster_feedback_t feedback = robomaster_motor_get_feedback(motor);

float angle = feedback.angle_deg;              // Current angle (0-359.99°)
float continuous_angle = feedback.angle_continuous_deg;  // Multi-turn angle
float velocity = feedback.velocity_rpm;        // Velocity in RPM
float current = feedback.current_actual;       // Actual current
uint8_t temperature = feedback.temperature;    // Motor temperature (°C)
bool valid = feedback.data_valid;             // Data validity flag
```

## Safety Features

### Emergency Stop
```c
robomaster_motor_emergency_stop(motor);           // Single motor
robomaster_manager_emergency_stop_all(&manager); // All motors
```

### Health Monitoring
```c
bool healthy = robomaster_motor_is_healthy(motor);
robomaster_motor_state_t state = robomaster_motor_get_state(motor);
uint8_t healthy_count = robomaster_manager_get_healthy_motor_count(&manager);
```

### Error Recovery
```c
robomaster_motor_reset_errors(motor);           // Single motor
robomaster_manager_reset_all_errors(&manager);  // All motors
```

## CAN Protocol Details

### GM6020 CAN Communication

**Control Message (0x1FF)**:
- Motors 1-4 controlled simultaneously
- 8 bytes: [M1_H, M1_L, M2_H, M2_L, M3_H, M3_L, M4_H, M4_L]
- Current range: ±30000

**Feedback Messages**:
- Motor 1: 0x205, Motor 2: 0x206, Motor 3: 0x207, Motor 4: 0x208
- 8 bytes: [Angle_H, Angle_L, Velocity_H, Velocity_L, Current_H, Current_L, Temperature, 0]
- Angle: 0-8191 (13-bit), Velocity: RPM (signed), Current: actual current (signed)

## Integration with RogiLinkFlex2

The library provides device interface functions for seamless integration:

```c
// Device interface functions
float robomaster_gm6020_read_data(void* handle, const char* data_type);
bool robomaster_gm6020_write_data(void* handle, const char* data_type, float value);
bool robomaster_gm6020_init_device(void* handle, uint8_t hw_id);

// Supported data types:
// Read: "angle", "angle_continuous", "speed", "current", "temperature"
// Write: "current", "velocity", "position"
```

## File Structure

```
Lib/Robomaster/
├── Inc/
│   ├── robomaster_motor.h      # Base motor definitions and manager
│   └── robomaster_gm6020.h     # GM6020-specific functions
├── Src/
│   ├── robomaster_motor.c      # Base motor implementation
│   ├── robomaster_gm6020.c     # GM6020-specific implementation
│   └── robomaster_manager.c    # Motor manager implementation
├── example.c                   # Usage examples
└── README.md                   # This file
```

## Dependencies

- STM32 HAL Library (CAN, GPIO, Timer)
- Standard C library (math.h, string.h, stdio.h)
- FreeRTOS (optional, for advanced timing)

## Notes

- Configure CAN bus to 1 Mbps (CAN_1000KBPS)
- Ensure proper CAN termination resistors
- Motor IDs must be set via DIP switches on GM6020
- Maximum 4 GM6020 motors per CAN bus (due to protocol limitations)
- Always implement emergency stop mechanisms
- Monitor motor temperatures to prevent overheating

## Examples

See `example.c` for detailed usage examples including:
1. Single motor control
2. Multiple motor coordination
3. Position control with PID tuning
4. Velocity control with ramping
5. Health monitoring
6. RogiLinkFlex2 integration

For more information, refer to the DJI RoboMaster GM6020 official documentation.