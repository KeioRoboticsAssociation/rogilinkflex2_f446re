#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "robomaster_motor.h"

// GM6020 specific definitions
#define GM6020_ANGLE_RESOLUTION 8192        // 13-bit resolution (2^13)
#define GM6020_ANGLE_DEGREES_PER_COUNT (360.0f / GM6020_ANGLE_RESOLUTION)
#define GM6020_MAX_CURRENT 30000
#define GM6020_MIN_CURRENT -30000

// GM6020 CAN IDs
#define GM6020_TX_CAN_ID_BASE 0x1FF         // Control command CAN ID
#define GM6020_RX_CAN_ID_BASE 0x205         // Feedback CAN ID base (0x205-0x208)

// GM6020 feedback data indices
#define GM6020_ANGLE_HIGH_BYTE 0
#define GM6020_ANGLE_LOW_BYTE 1
#define GM6020_VELOCITY_HIGH_BYTE 2
#define GM6020_VELOCITY_LOW_BYTE 3
#define GM6020_CURRENT_HIGH_BYTE 4
#define GM6020_CURRENT_LOW_BYTE 5
#define GM6020_TEMPERATURE_BYTE 6

// GM6020 control data indices (in CAN message)
#define GM6020_CURRENT_1_HIGH_BYTE 0
#define GM6020_CURRENT_1_LOW_BYTE 1
#define GM6020_CURRENT_2_HIGH_BYTE 2
#define GM6020_CURRENT_2_LOW_BYTE 3
#define GM6020_CURRENT_3_HIGH_BYTE 4
#define GM6020_CURRENT_3_LOW_BYTE 5
#define GM6020_CURRENT_4_HIGH_BYTE 6
#define GM6020_CURRENT_4_LOW_BYTE 7

// GM6020 default configuration
extern const robomaster_motor_config_t GM6020_DEFAULT_CONFIG;

// GM6020 specific functions
bool gm6020_init(robomaster_motor_t* motor, uint8_t motor_id, CAN_HandleTypeDef* hcan);
bool gm6020_parse_feedback(robomaster_motor_t* motor, uint8_t* can_data);
bool gm6020_send_current_group(CAN_HandleTypeDef* hcan, float current1, float current2, float current3, float current4);
bool gm6020_send_current_single(robomaster_motor_t* motor, float current);

// GM6020 utility functions
uint16_t gm6020_get_raw_angle(uint8_t* can_data);
int16_t gm6020_get_raw_velocity(uint8_t* can_data);
int16_t gm6020_get_raw_current(uint8_t* can_data);
uint8_t gm6020_get_temperature(uint8_t* can_data);
uint32_t gm6020_get_rx_can_id(uint8_t motor_id);
float gm6020_raw_angle_to_degrees(uint16_t raw_angle);
float gm6020_normalize_angle(float angle_deg);
int16_t gm6020_current_to_raw(float current_ma);
float gm6020_raw_current_to_actual(int16_t raw_current);

// GM6020 advanced control functions
bool gm6020_set_position_with_feedforward(robomaster_motor_t* motor, float target_pos_deg, float feedforward_current);
bool gm6020_calibrate_zero_position(robomaster_motor_t* motor);
void gm6020_update_continuous_angle(robomaster_motor_t* motor);

// GM6020 diagnostics
void gm6020_print_status(robomaster_motor_t* motor);
bool gm6020_self_test(robomaster_motor_t* motor);

#ifdef __cplusplus
}
#endif