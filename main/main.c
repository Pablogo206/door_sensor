#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/uart.h>
#include "nvs_flash.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"

//UART Variables
#define BUF_SIZE (1024)
//#define UART_PORT_NUM UART_NUM_0	//Esto usa el cable USB como uart
#define TXD_PIN               (GPIO_NUM_4)
#define RXD_PIN               (GPIO_NUM_5)
static const int RX_BUF_SIZE = 1024;

// scan parameters
static esp_ble_scan_params_t ble_scan_params = {
		.scan_type              = BLE_SCAN_TYPE_ACTIVE,
		.own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
		.scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
		.scan_interval          = 0xA0,	//Scanning interval 100 ms: Time/0.625 = N -> (hex)
		.scan_window            = 0xA0
	};

//Uart start
void init_uart(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}


uint8_t num_seq = 0x00;
uint8_t msg_err[3] = {0xAA,0xAA,0xAA};
// GAP callback
//Aqui se gestionan los eventos
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
		
		case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: 
				
			
			if(param->scan_param_cmpl.status == ESP_BT_STATUS_SUCCESS) {
				
				esp_ble_gap_start_scanning(60);
			}
			
			break;
		
		case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:	//Cuando llega aqui es cuando oficialmente ha empezado
			
			
			if(param->scan_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
				
			}
			
			
			break;
		
		case ESP_GAP_BLE_SCAN_RESULT_EVT:	//aqui llega cada vez que encuentra cualquier trama
			
			if(param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
				
				uint8_t *advertisement_raw = param->scan_rst.ble_adv;
				if(advertisement_raw[5] == 0xE4 && advertisement_raw[6]==0xFF){	//Y aqui filtramos para que sean las nuestras
					uint8_t *msg_adv = &advertisement_raw[9];
					for (int  i = 0; i < 17; i++)
					{
						printf("Indice: %d",i);
						printf(" array de recibido : %d \n",msg_adv[i]);
					}
					
					if(msg_adv[5]!=num_seq){
						printf("LLega aqui \n");
						uart_write_bytes(UART_NUM_1, (const char *) msg_adv, 8);
						num_seq = msg_adv[5];
					}
				}
				
				
			}
			else if(param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_CMPL_EVT){
				
				esp_ble_gap_start_scanning(60);
			}
				
			break;
		
		default:
			
			uart_write_bytes(UART_NUM_1, (const char *) msg_err, 3);
			
			break;
	}
}


void app_main() {
	
	
	// set components to log only errors
	esp_log_level_set("*", ESP_LOG_ERROR);
	
	// initialize nvs
	ESP_ERROR_CHECK(nvs_flash_init());
	
	
	// release memory reserved for classic BT (not used)
	ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
	
	// initialize the BT controller with the default config
	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
	
	// enable the BT controller in BLE mode
    esp_bt_controller_enable(ESP_BT_MODE_BLE);
	
	// initialize Bluedroid library
	esp_bluedroid_init();
    esp_bluedroid_enable();
	
	// register GAP callback function
	ESP_ERROR_CHECK(esp_ble_gap_register_callback(esp_gap_cb));
	
	// configure scan parameters
	esp_ble_gap_set_scan_params(&ble_scan_params);

	//init uart
	init_uart();
}