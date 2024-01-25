#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "rom/ets_sys.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#include "DHT22.h"
#include "MQ135.h"
#include "DSM501A.h"
#include "driver/adc.h"
#define ADC1_TEST_CHANNEL (4)  // GPIO35 https://esp32.vn/idf/adc.html

#include "freertos/timers.h"
TimerHandle_t sensorTimer;
#define SENSOR_UPDATE_INTERVAL (3000 / portTICK_PERIOD_MS)

void sensorTimerCallback(TimerHandle_t xTimer) {
    printf("Sensor update\n");
}

void Reader_task(void *pvParameter)
{
    // DHT22
	setDHTgpio(GPIO_NUM_26);

    // MQ135
    adc1_config_width(ADC_WIDTH_BIT_12);  
    adc1_config_channel_atten(ADC1_TEST_CHANNEL, ADC_ATTEN_DB_0);

    // DSM501A
    float ratio_v2 = 0;
    float concentration_v2 = 0;
    float ratio_v1 = 0;
    float concentration_v1 = 0;

    gpio_config_t io_conf = {
        .pin_bit_mask = GPIO_INPUT_PIN_SEL,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    gpio_isr_handler_add(DSM501a_RED_PIN, change_v2_isr, (void*) DSM501a_RED_PIN);
    gpio_isr_handler_add(DSM501a_YELLOW_PIN, change_v1_isr, (void*) DSM501a_YELLOW_PIN);

	while(1) {
        // DHT22
		printf("Sensor Readings\n" );
		int ret = readDHT();
		
		errorHandler(ret);

		printf("Humidity: %.2f %%\n", getHumidity());
		printf("Temperature: %.2f degC\n", getTemperature());

		vTaskDelay(1000 / portTICK_PERIOD_MS);


        // MQ135
        Air_param_t aqi;
		printf("The adc1 value: %d\n", adc1_get_raw(ADC1_TEST_CHANNEL));
        getAQI_Correctval(&aqi, getHumidity(), getTemperature(), adc1_get_raw(ADC1_TEST_CHANNEL));
        vTaskDelay(1000 / portTICK_PERIOD_MS);
		printf("CO2: %d ppm\n", aqi.CO2);



        // DSM501A
        
        vTaskDelay(1000/ portTICK_PERIOD_MS);

        gpio_isr_handler_remove(DSM501a_RED_PIN);
        gpio_isr_handler_remove(DSM501a_YELLOW_PIN);

        unsigned long accumulated_pulses_v2 = pulses_v2;
        pulses_v2 = 0;
        previous_state_v2 = 1;

        unsigned long accumulated_pulses_v1 = pulses_v1;
        pulses_v1 = 0;
        previous_state_v1 = 1;

        ratio_v2 = (float)( (accumulated_pulses_v2) / (float)(60 * 10000) );
        ratio_v1 = (float)( (accumulated_pulses_v1) / (float)(60 * 10000) );

        concentration_v2 = 615.55 * ratio_v2;
        concentration_v1 = 615.55 * ratio_v1;

        gpio_isr_handler_add(DSM501a_RED_PIN, change_v2_isr, (void*) DSM501a_RED_PIN);
        gpio_isr_handler_add(DSM501a_YELLOW_PIN, change_v1_isr, (void*) DSM501a_YELLOW_PIN);

        printf("PM10: %.2f\n", concentration_v2);
        printf("PM2.5: %.2f\n\n", concentration_v1);
	}	
}

void app_main()
{
	nvs_flash_init();
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    // sensorTimer = xTimerCreate("SensorTimer", SENSOR_UPDATE_INTERVAL, pdTRUE, (void *)0, sensorTimerCallback);
    // if (sensorTimer == NULL) {
    //     printf("Error");
    // }
    // xTimerStart(sensorTimer, 0);
	xTaskCreate(&Reader_task, "Reader_task", 2048, NULL, 5, NULL );
}
