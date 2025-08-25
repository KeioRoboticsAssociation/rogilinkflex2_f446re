#include "main.hpp"
#include "UartLink.hpp"

// ---- HALのオブジェクト ----
extern UART_HandleTypeDef huart2;

// ---- 対ROS通信 ---- (不要な場合はUartLink周りを削除)
UartLink uart_link(&huart2, 0);

// UART受信割り込み
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        uart_link.interrupt();
    }
}

// GPIO割り込み
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    
}

void setup() {
    uart_link.start(); // ros2との通信を開始
}

void loop() {
    
}