#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "sdkconfig.h"
#include <gpio_num.h>
#include <portmacro.h>

void app_main(void) {
    // Configure the UART peripheral
    const uart_port_t uart_num = UART_NUM_1; // UART port number
    const int tx_pin = GPIO_NUM_1;           // UART TX pin
    const int rx_pin = GPIO_NUM_3;           // UART RX pin

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    // Install UART driver
    uart_driver_install(uart_num, 256, 0, 0, NULL, 0);
    uart_param_config(uart_num, &uart_config);
    uart_set_pin(uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Print "Hello, ESP32!"
    uart_write_bytes(uart_num, "Hello, ESP32!\n", 14);

    while (1) {
        // Main loop code goes here
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay for 1 second
    }
}