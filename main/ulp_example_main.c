/* ULP Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

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

#define TXD_PIN               (GPIO_NUM_4)
#define RXD_PIN               (GPIO_NUM_5)
#define SENSOR_IN             (GPIO_NUM_13)
#define uS_TO_S_FACTOR 1000000
#define TIME_TO_SLEEP  5
extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");


/* This function is called once after power-on reset, to load ULP program into
 * RTC memory and configure the ADC.
 */


/* This function is called every time before going into deep sleep.
 * It starts the ULP program and resets measurement counter.
 */


static const int RX_BUF_SIZE = 1024;
char sec = 0;
bool led = false;
struct Datos_sensor {
    char UUID[4];
    char ID_sensor[4];
    char secuencia[2];
    char estado[2];
    char BAT[2];
};
char src[14];

void start_ulp_program(void){    
        esp_err_t err = ulp_load_binary(0, ulp_main_bin_start,
            (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
    ESP_ERROR_CHECK(err);

    /* GPIO used for pulse counting. */
    gpio_num_t gpio_num = SENSOR_IN;
    int rtcio_num = rtc_io_number_get(gpio_num);
    assert(rtc_gpio_is_valid_gpio(gpio_num) && "GPIO used for pulse counting must be an RTC IO");

    /* Initialize some variables used by ULP program.
     * Each 'ulp_xyz' variable corresponds to 'xyz' variable in the ULP program.
     * These variables are declared in an auto generated header file,
     * 'ulp_main.h', name of this file is defined in component.mk as ULP_APP_NAME.
     * These variables are located in RTC_SLOW_MEM and can be accessed both by the
     * ULP and the main CPUs.
     *
     * Note that the ULP reads only the lower 16 bits of these variables.
     */
    ulp_debounce_counter = 3;
    ulp_debounce_max_count = 3;
    ulp_next_edge = 0;
    ulp_io_number = rtcio_num; /* map from GPIO# to RTC_IO# */
    ulp_edge_count_to_wake_up = 10;

    /* Initialize selected GPIO as RTC IO, enable input, disable pullup and pulldown */
    rtc_gpio_init(gpio_num);
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
void serialize(struct Datos_sensor *datos) {
    char buffer [14];
    snprintf(
        buffer,
        15,
        datos->UUID,
        datos->ID_sensor,
        datos->secuencia,
        datos->estado,
        datos->BAT
    );
    for (int i = 0;i<14;i++){
        printf("Indice del buffer: %d \n",buffer[i]);
        src[i] = buffer[i];
    };
}



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

int sendData(const char* logName, const char* data)
{
    
    const int len = strlen(data);
    printf("longitud %d \n",len);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}



void tx_task()
{
    
    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    sec = sec + 1;
    
    if(rtc_gpio_get_level(SENSOR_IN) == 1){
        printf("El número de estado es: %d  \n",rtc_gpio_get_level(SENSOR_IN));
        struct Datos_sensor dato = {.UUID = "FFE4", .ID_sensor = "0001", .secuencia = "01" , .estado = "01",.BAT = "01"};
        serialize(&dato);
        sendData(TX_TASK_TAG, src);
    }else{
        printf("El número de estado es: %d  \n",gpio_get_level(SENSOR_IN));
        struct Datos_sensor dato = {.UUID = "FFE4", .ID_sensor = "0001", .secuencia = "01" , .estado = "00",.BAT = "01"};
        serialize(&dato);
        sendData(TX_TASK_TAG, src);
    }
    


    

}


/*static void sensor_task(void *arg)
{
    
    const char* namespace = "plusecnt";
    const char* count_key = "count";

    ESP_ERROR_CHECK( nvs_flash_init() );
    nvs_handle_t handle;
    ESP_ERROR_CHECK( nvs_open(namespace, NVS_READWRITE, &handle));
    uint32_t pulse_count = 0;
    esp_err_t err = nvs_get_u32(handle, count_key, &pulse_count);
    assert(err == ESP_OK || err == ESP_ERR_NVS_NOT_FOUND);
    printf("Read state from NVS: %5d\n", pulse_count);
    
    vTaskDelay(500 / portTICK_PERIOD_MS);
}*/




void app_main(void)
{
   
    init_uart();
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    printf("Causa %d \n",cause);
    if (cause == 0) {
        printf("Not ULP wakeup, initializing ULP\n");
        start_ulp_program();
    } else {
        printf("ULP wakeup, saving pulse count\n");
        if(cause == 4){
            tx_task();
            
        }else{
            if (cause == 6)
            {
                tx_task();
                start_ulp_program();
            }
            
            
        }
        
        
    }
    printf("Entering deep sleep\n\n");
    esp_sleep_enable_ulp_wakeup();
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    esp_deep_sleep_start();
    
}

