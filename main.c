#include <stdio.h>
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "esp_vfs_dev.h"
#include "esp_log.h"
#include "bmp280.h"
#define PIN_NUM_MISO GPIO_NUM_19
#define PIN_NUM_MOSI GPIO_NUM_23
#define PIN_NUM_CLK  GPIO_NUM_18
#define PIN_NUM_CS   GPIO_NUM_5

void init_spi() {
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };
    spi_bus_initialize(HSPI_HOST, &buscfg, 1);

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000,
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 7
    };
    spi_device_handle_t handle;
    spi_bus_add_device(HSPI_HOST, &devcfg, &handle);
}

void init_uart() {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, GPIO_NUM_17, GPIO_NUM_16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_1, 1024 * 2, 0, 0, NULL, 0);
}

void app_main() {
    init_spi();
    init_uart();

    bmp280_params_t params;
    bmp280_init_default_params(&params);
    bmp280_t bmp280;
    bmp280.i2c_dev.bus = HSPI_HOST;
    bmp280.i2c_dev.addr = BMP280_I2C_ADDRESS_0;

    while (1) {
        float pressure, temperature, humidity;
        if (bmp280_read_float(&bmp280, &temperature, &pressure, &humidity) == ESP_OK) {
            printf("Pressure: %.2f Pa, Temperature: %.2f C\n", pressure, temperature);

            char buffer[64];
            int len = snprintf(buffer, sizeof(buffer), "Pressure: %.2f Pa, Temperature: %.2f C\n", pressure, temperature);
            uart_write_bytes(UART_NUM_1, buffer, len);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
