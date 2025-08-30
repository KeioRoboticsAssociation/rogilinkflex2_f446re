ソースファイルを格納するディレクトリです。

#include "main.hpp"
#include "rogilinkflex2.hpp"

// ---- HALのオブジェクト ----
extern UART_HandleTypeDef huart2;

// GPIO割り込み
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    
}

void setup() {
    // Initialize RogiLinkFlex2 communication system
    rogilinkflex2_init();
}

void loop() {
    // Process RogiLinkFlex2 communication
    rogilinkflex2_process();
    
    // Small delay to prevent excessive CPU usage
    HAL_Delay(1);
}