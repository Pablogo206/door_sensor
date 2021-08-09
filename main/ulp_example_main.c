#include <stdio.h>
#include "driver/uart.h"
#include "esp_system.h"
#include "string.h"
#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_sleep.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc_periph.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp32/ulp.h"
#include "ulp_main.h"
#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "esp_sleep.h"
#include "ulp_main.h"
#include "ulp_common.h"
#include "esp32/ulp.h"
#include "driver/rtc_io.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc_periph.h"
#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "lwip/err.h"
#include "lwip/sys.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "freertos/event_groups.h"
#include <string.h>

#include "nvs_flash.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"

#include <stdio.h>
#include <stdarg.h>


#include "esp_wifi.h"
//
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "nvs_flash.h"
#include "esp_err.h"
#include "esp_log.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#define PORT                   (3333)
#define HOST_IP_ADDR "172.20.10.2"
#define EXAMPLE_ESP_WIFI_SSID      "iPhone de Pablo"
#define EXAMPLE_ESP_WIFI_PASS      "99999999"
#define EXAMPLE_ESP_MAXIMUM_RETRY  (9)
#define SENSOR_IN             (GPIO_NUM_13)
/* FreeRTOS event group to signal when we are connected*/


/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1


#define TXD_PIN               (GPIO_NUM_4)
#define RXD_PIN               (GPIO_NUM_5)
#define SENSOR_IN             (GPIO_NUM_13)
#define uS_TO_S_FACTOR 1000000
#define TIME_TO_SLEEP  5
extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");
char payload [15];
char sec = 0;
bool led = false;


static esp_ble_adv_params_t ble_adv_params = {
	
	.adv_int_min = 0xA0,													//100 ms intervalo de mensajes
	.adv_int_max = 0xA0,
	.adv_type = ADV_TYPE_NONCONN_IND,
	.own_addr_type  = BLE_ADDR_TYPE_PUBLIC,
	.channel_map = ADV_CHNL_ALL,
	.adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

RTC_DATA_ATTR  uint8_t n_seq = 0;
												  
RTC_DATA_ATTR uint8_t adv_raw_data[17] = {0x02,0x01,0x06,0x03,0x03,0xE4,0xFF,0x08,0x16,0xE4,
								   0xFF,0xFF,0xE4,0x01,0x00,0x00,0x00};
								   		//UUID	  //ID//Seq//State//Bateria	
										
// GAP callback
void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {														//Arbol de eventos como el scanner
			
		case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT: 
			printf("Valor al enviarse %d \n",adv_raw_data[14]);	
			printf("ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT\n");
			esp_ble_gap_start_advertising(&ble_adv_params);
			break;			
		
		case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
			
			printf("ESP_GAP_BLE_ADV_START_COMPLETE_EVT\n");
			if(param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
				printf("Advertising started\n\n");								//cuando llega aqui, se esta promocionando
				vTaskDelay(1000 / portTICK_PERIOD_MS); 							//Pasado un segundo paramos la promocion
				esp_ble_gap_stop_advertising();
			}
			else printf("Unable to start advertising process, error code %d\n\n", param->scan_start_cmpl.status);
			break;
		case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:

			printf("Advertising stopped\n\n");
															//asignamos nuevo numero de secuencia

			ESP_ERROR_CHECK(esp_ble_gap_config_adv_data_raw(adv_raw_data, 17)); //vuelta a empezar
			break;
	
		default:
		
			printf("Event %d unhandled\n\n", event);
			break;
	}
}


void start_ulp_program(void){    
        esp_err_t err = ulp_load_binary(0, ulp_main_bin_start,
            (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
    ESP_ERROR_CHECK(err);

    /* GPIO used for pulse counting. */
    gpio_num_t gpio_num = SENSOR_IN;
    int rtcio_num = rtc_io_number_get(gpio_num);
    assert(rtc_gpio_is_valid_gpio(gpio_num) && "GPIO used for pulse counting must be an RTC IO");

    
    ulp_debounce_counter = 3;
    ulp_debounce_max_count = 3;
    ulp_next_edge = 1;
    ulp_io_number = rtcio_num; /* map from GPIO# to RTC_IO# */
    ulp_edge_count_to_wake_up = 3;

    /* Initialize selected GPIO as RTC IO, enable input, disable pullup and pulldown */
    rtc_gpio_deinit(gpio_num);
    rtc_gpio_set_direction(gpio_num, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pulldown_dis(gpio_num);
    rtc_gpio_pullup_dis(gpio_num);
    rtc_gpio_hold_en(gpio_num);

    /* Disconnect GPIO12 and GPIO15 to remove current drain through
     * pullup/pulldown resistors.
     * GPIO12 may be pulled high to select flash voltage.
     */
    rtc_gpio_isolate(GPIO_NUM_12);
    rtc_gpio_isolate(GPIO_NUM_15);
    esp_deep_sleep_disable_rom_logging(); // suppress boot messages

    /* Set ULP wake up period to T = 20ms.
     * Minimum pulse width has to be T * (ulp_debounce_counter + 1) = 80ms.
     */
    ulp_set_wakeup_period(1, 20000);

    /* Start the program */
    err = ulp_run(&ulp_entry - RTC_SLOW_MEM);
    ESP_ERROR_CHECK(err);
}

void bluetooth(){
    printf("BT broadcast\n\n");
	
	// set components to log only errors
	esp_log_level_set("*", ESP_LOG_ERROR);
	
	// initialize nvs
	ESP_ERROR_CHECK(nvs_flash_init());
	printf("- NVS init ok\n");
	
	// release memory reserved for classic BT (not used)
	ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
	printf("- Memory for classic BT released\n");
	
	// initialize the BT controller with the default config
	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
	printf("- BT controller init ok\n");
	
	// enable the BT controller in BLE mode
    esp_bt_controller_enable(ESP_BT_MODE_BLE);
	printf("- BT controller enabled in BLE mode\n");
	
	// initialize Bluedroid library
	esp_bluedroid_init();
    esp_bluedroid_enable();
	printf("- Bluedroid initialized and enabled\n");
	
	// register GAP callback function
	ESP_ERROR_CHECK(esp_ble_gap_register_callback(esp_gap_cb));
	printf("- GAP callback registered\n\n");
	printf("Valor a primer registro %d \n",adv_raw_data[14]);
	// configure the adv data
	ESP_ERROR_CHECK(esp_ble_gap_config_adv_data_raw(adv_raw_data, 17));
	printf("- ADV data configured\n\n");

}

void app_main(void)
{
    printf("Valor Previo %d \n",adv_raw_data[14]);
    adv_raw_data[14] = n_seq;
    n_seq = n_seq + 1;
    printf("Valor Previo %d \n",adv_raw_data[14]);
    if(gpio_get_level(SENSOR_IN) == 1){
        adv_raw_data[15] = 0x01;
        printf("Estado= %d \n",gpio_get_level(SENSOR_IN));
    }else{
        adv_raw_data[15] = 0x00;
        printf("Estado= %d \n",gpio_get_level(SENSOR_IN));
    }
	 esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    printf("Causa %d \n",cause);
    if (cause == 0) {
        printf("Not ULP wakeup, initializing ULP\n");
        start_ulp_program();
    } else {
        if(cause == 4){
            printf("ULP wakeup timer \n ");
            bluetooth();
            start_ulp_program();
        }else{
            if (cause == 6)
            {
                printf("ULP wakeup switch \n ");
                bluetooth();
                start_ulp_program();
            }
        }
    }
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    printf("Entering deep sleep\n\n");
    esp_sleep_enable_ulp_wakeup();
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    esp_deep_sleep_start();
    
}

