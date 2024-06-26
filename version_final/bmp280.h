#ifndef __BMP280_H__
#define __BMP280_H__

#include "driver/spi_master.h"
#include <stdint.h>
#include "esp_system.h"

// Direcciones de registros del BMP280
#define BMP280_REG_TEMP_XLSB 0xFC
#define BMP280_REG_TEMP_LSB  0xFB
#define BMP280_REG_TEMP_MSB  0xFA
#define BMP280_REG_PRESS_XLSB 0xF9
#define BMP280_REG_PRESS_LSB  0xF8
#define BMP280_REG_PRESS_MSB  0xF7
#define BMP280_REG_CONFIG     0xF5
#define BMP280_REG_CTRL_MEAS  0xF4
#define BMP280_REG_STATUS     0xF3
#define BMP280_REG_RESET      0xE0
#define BMP280_REG_ID         0xD0
#define BMP280_REG_CALIB00    0x88

#define BMP280_CTRL_MEAS_MODE_NORMAL_TEMP_OVRS  0x20  // Modo normal y oversampling de temperatura x2
#define BMP280_CTRL_MEAS_MODE_NORMAL_PRES_OVRS  0x0F  // Modo normal y oversampling de presión x16

// Estructura para almacenar los datos de calibración
struct bmp280_calib_param {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
};

void bmp280_spi_write(uint8_t reg, uint8_t value);
uint8_t bmp280_spi_read(uint8_t reg);
void bmp280_read_calibration_data(void);
void bmp280_init(void);
int32_t bmp280_compensate_T(int32_t adc_T);
uint32_t bmp280_compensate_P(int32_t adc_P);
float calculate_altitude(float pressure, float sea_level_pressure);
esp_err_t spi_init(void);

#endif // __BMP280_H__

