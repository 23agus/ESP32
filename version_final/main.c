#include <stdio.h>
#include <string.h>
#include <math.h>
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "bmp280.h"
#include <stdint.h>

static const int RX_BUF_SIZE = 1024;
static const char *TAG = "BMP280_APP";  // Definir TAG para las funciones de log

// Se selecciona uart2 y sus configuracion
#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)
#define UART UART_NUM_2

void init_uart(void);
void send_bmp280(int tem, int hum);

void init_uart(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UART, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART, &uart_config);
    uart_set_pin(UART, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void send_bmp280(int tem, int hum) {
    static char Txdata[60];
    sprintf(Txdata, "Temperatura = %d Grados | Humedad = %d admosferas \r\n", tem, hum);
    uart_write_bytes(UART, Txdata, strlen(Txdata));
}

void app_main(void) {
    esp_err_t ret = spi_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI");
        return;
    }
    bmp280_init();
    init_uart();

    while (1) {
        uint8_t msb, lsb, xlsb;
        int32_t adc_T, adc_P;
        float temp, press;

        // Leer datos de temperatura
        msb = bmp280_spi_read(BMP280_REG_TEMP_MSB);
        lsb = bmp280_spi_read(BMP280_REG_TEMP_LSB);
        xlsb = bmp280_spi_read(BMP280_REG_TEMP_XLSB);
        adc_T = (msb << 12) | (lsb << 4) | (xlsb >> 4);

        // Leer datos de presión
        msb = bmp280_spi_read(BMP280_REG_PRESS_MSB);
        lsb = bmp280_spi_read(BMP280_REG_PRESS_LSB);
        xlsb = bmp280_spi_read(BMP280_REG_PRESS_XLSB);
        adc_P = (msb << 12) | (lsb << 4) | (xlsb >> 4);

        // Compensar lecturas
        temp = bmp280_compensate_T(adc_T) / 100.0;
        press = bmp280_compensate_P(adc_P) / 25600.0;
        //printf("Temperatura %.2f",temp);
        ESP_LOGI(TAG, "Temperatura: %.2f °C", temp);
        send_bmp280((int)temp, (int)press);

        vTaskDelay(pdMS_TO_TICKS(3000)); // Espera 5 segundos
    }
}



