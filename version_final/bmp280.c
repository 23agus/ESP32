#include "bmp280.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include <string.h>
#include <stdint.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

spi_device_handle_t spi;
static const char *TAG = "BMP280_SPI";
int32_t t_fine; //variable global para la compensación de temperatura y presión

// Definiciones de pines SPI
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5

// Estructura para almacenar los datos de calibración
struct bmp280_calib_param calib_data;

// Función para escribir en un registro del BMP280
void bmp280_spi_write(uint8_t reg, uint8_t value) {
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    uint8_t tx_data[2] = {reg & 0x7F, value};  // Dirección y valor (reg & 0x7F para el bit de escritura)
    t.length = 16;  // 8 bits de dirección y 8 bits de datos
    t.tx_buffer = tx_data;
    spi_device_transmit(spi, &t);  // Transmitir
}

// Función para leer de un registro del BMP280
uint8_t bmp280_spi_read(uint8_t reg) {
    spi_transaction_t t;
    uint8_t rx_data[2] = {0};

    memset(&t, 0, sizeof(t));
    uint8_t tx_data[2] = {reg | 0x80, 0};  // Dirección con el bit de lectura
    t.length = 16;  // 8 bits de dirección y 8 bits de datos
    t.tx_buffer = tx_data;
    t.rx_buffer = rx_data;
    spi_device_transmit(spi, &t);  // Transmitir

    return rx_data[1];  // Devolver solo el byte de datos
}

// Función para leer datos de calibración
void bmp280_read_calibration_data(void) {
    uint8_t calib[24];
    for (int i = 0; i < 24; ++i) {
        calib[i] = bmp280_spi_read(BMP280_REG_CALIB00 + i);
    }

    calib_data.dig_T1 = (calib[1] << 8) | calib[0];
    calib_data.dig_T2 = (calib[3] << 8) | calib[2];
    calib_data.dig_T3 = (calib[5] << 8) | calib[4];
    calib_data.dig_P1 = (calib[7] << 8) | calib[6];
    calib_data.dig_P2 = (calib[9] << 8) | calib[8];
    calib_data.dig_P3 = (calib[11] << 8) | calib[10];
    calib_data.dig_P4 = (calib[13] << 8) | calib[12];
    calib_data.dig_P5 = (calib[15] << 8) | calib[14];
    calib_data.dig_P6 = (calib[17] << 8) | calib[16];
    calib_data.dig_P7 = (calib[19] << 8) | calib[18];
    calib_data.dig_P8 = (calib[21] << 8) | calib[20];
    calib_data.dig_P9 = (calib[23] << 8) | calib[22];
}

void bmp280_init(void) {
    uint8_t id = bmp280_spi_read(BMP280_REG_ID);
    ESP_LOGI(TAG, "BMP280 ID: 0x%x", id);

    if (id != 0x58) {
        ESP_LOGE(TAG, "No se pudo encontrar un sensor BMP280 válido, verifica la conexión!");
        while (1) { vTaskDelay(1); }
    }

    // Leer datos de calibración
    bmp280_read_calibration_data();

    // Resetear el sensor
    bmp280_spi_write(BMP280_REG_RESET, 0xB6);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Configurar el sensor
    //bmp280_spi_write(BMP280_REG_CTRL_MEAS, BMP280_CTRL_MEAS_MODE_NORMAL_TEMP_OVRS | BMP280_CTRL_MEAS_MODE_NORMAL_PRES_OVRS);
    bmp280_spi_write(BMP280_REG_CTRL_MEAS, 0x2F);  // Normal mode, temp and pressure oversampling 1  0x57
    bmp280_spi_write(BMP280_REG_CONFIG, 0x48);     // Standby 1000ms, filter off   0x10
}

// Función para compensar la lectura de temperatura
int32_t bmp280_compensate_T(int32_t adc_T) {
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)calib_data.dig_T1 << 1))) *
           ((int32_t)calib_data.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)calib_data.dig_T1)) *
             ((adc_T >> 4) - ((int32_t)calib_data.dig_T1))) >> 12) *
           ((int32_t)calib_data.dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

// Función para compensar la lectura de presión
uint32_t bmp280_compensate_P(int32_t adc_P) {
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib_data.dig_P6;
    var2 = var2 + ((var1 * (int64_t)calib_data.dig_P5) << 17);
    var2 = var2 + (((int64_t)calib_data.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)calib_data.dig_P3) >> 8) +
           ((var1 * (int64_t)calib_data.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib_data.dig_P1) >> 33;
    if (var1 == 0) {
        return 0; // Evitar división por cero
    }
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)calib_data.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)calib_data.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)calib_data.dig_P7) << 4);
    return (uint32_t)p;
}

// Función para calcular la altitud
float calculate_altitude(float pressure, float sea_level_pressure) {
    return 44330.0 * (1.0 - pow(pressure / sea_level_pressure, 0.1903));
}

// Función para inicializar SPI
esp_err_t spi_init(void) {
    esp_err_t ret;
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000,  // Clock out a 1 MHz
        .mode = 0,                  // SPI mode 0
        .spics_io_num = PIN_NUM_CS, // CS pin
        .queue_size = 7,            // We want to be able to queue 7 transactions at a time
    };

    // Inicializar el bus SPI
    ret = spi_bus_initialize(VSPI_HOST, &buscfg, 1);
    // Agregar el dispositivo al bus
    ret = spi_bus_add_device(VSPI_HOST, &devcfg, &spi);
    return ret;
}


