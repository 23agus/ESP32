#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

static const int RX_BUF_SIZE = 1024;

// Se selecciona uart2 y sus configuracion
#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)
#define UART UART_NUM_2

int num=0;

void init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART, &uart_config);
    uart_set_pin(UART, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}


void send_bmp280 (int tem , int hum){
	char* Txdata= (char*) malloc(60);
	    	sprintf(Txdata,"Temperatura = %d Grados | Humedad = %d admosferas \r\n",tem,hum);
	    	uart_write_bytes(UART,Txdata, strlen(Txdata));
};

void app_main(void)
{
    init();
    //uart_write_bytes(UART,"fernet", 6);
    send_bmp280(24,2);

}
