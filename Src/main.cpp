#include "main.hpp"
#include "rogilinkflex2.hpp"
#include <string.h>

extern UART_HandleTypeDef huart2;

// Function to send debug messages
void send_debug_message(const char* message) {
    char debug_json[256];
    snprintf(debug_json, sizeof(debug_json), 
             "{\"type\":\"debug\",\"message\":\"%s\"}\n", message);
    HAL_UART_Transmit(&huart2, (uint8_t*)debug_json, strlen(debug_json), HAL_MAX_DELAY);
}

void test_setup() {
    // Initialize RogiLinkFlex2 communication system
    rogilinkflex2_init();
    
    // Register mock devices for testing (no actual hardware required)
    rogilinkflex2_register_device(0, DEVICE_DYNAMIXEL, nullptr, 0, 1);
    rogilinkflex2_register_device(1, DEVICE_DYNAMIXEL, nullptr, 0, 2);
    rogilinkflex2_register_device(2, DEVICE_ROBOMASTER, nullptr, 0, 3);
    rogilinkflex2_register_device(3, DEVICE_SENSOR_TEMP, nullptr, 0, 4);
    
    // Send startup debug message
    send_debug_message("HAL initialized and ready");
}

void test_loop() {
    volatile static uint32_t last_test_time = 0;
    volatile static uint8_t test_phase = 0;
    volatile uint32_t current_time = HAL_GetTick();
    
    // Process RogiLinkFlex2 communication
    rogilinkflex2_process();
    
    // Send periodic test data every 5 seconds to demonstrate functionality
    if (current_time - last_test_time >= 5000) {
        switch (test_phase) {
            case 0:
                // Send test read response
                send_debug_message("Sending test read response");
                HAL_Delay(10); // Small delay between transmissions
                uart_send_response(0, "angle", 45.5f, "success");
                break;
            case 1:
                // Send smaller test periodic data (reduced from 4 devices to 2)
                {
                    send_debug_message("Sending test periodic data");
                    HAL_Delay(10); // Small delay between transmissions
                    uint8_t device_ids[] = {0, 1};
                    float values[] = {30.0f, 25.5f, 35.0f, 28.0f}; // angle, speed for each device
                    const char data_names[2][16] = {"angle", "speed"};
                    uart_send_periodic_data_multi("motor_data", 2, device_ids, data_names, values, 2);
                }
                break;
            case 2:
                // Send temperature data
                send_debug_message("Sending temperature data");
                HAL_Delay(10); // Small delay between transmissions
                uart_send_response(3, "temperature", 26.5f, "success");
                break;
            default:
                test_phase = 0;
                break;
        }
        
        test_phase++;
        if (test_phase > 2) test_phase = 0;
        last_test_time = current_time;
    }
    
    // Small delay to prevent excessive CPU usage
    HAL_Delay(1);
}

void setup() {
    test_setup();
}

void loop() {
    test_loop();
}