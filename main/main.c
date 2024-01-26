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
#include "esp_adc_cal.h"
static esp_adc_cal_characteristics_t adc1_chars;

#include <driver/i2c.h>
#include <esp_log.h>
#include "HD44780.h"

// #include <string.h>
// #define LCD_ADDR 0x27
// #define SDA_PIN  25
// #define SCL_PIN  33
// #define LCD_COLS 16
// #define LCD_ROWS 2

// void LCD_DemoTask(void* param)
// {
//     char num[20];
//     while (true) {
//         LCD_home();
//         LCD_clearScreen();
//         LCD_writeStr("16x2 I2C LCD");
//         vTaskDelay(3000 / portTICK_PERIOD_MS);
//         LCD_clearScreen();
//         LCD_writeStr("Lets Count 0-10!");
//         vTaskDelay(3000 / portTICK_PERIOD_MS);
//         LCD_clearScreen();
//         for (int i = 0; i <= 10; i++) {
//             LCD_setCursor(1, 1);
//             sprintf(num, "%d", i);
//             LCD_writeStr(num);
//             vTaskDelay(1000 / portTICK_PERIOD_MS);
//         }
  
//     }
    // setDHTgpio(GPIO_NUM_26);
    // char buff[20];
    // while (true) {
    //     int ret = readDHT();
 	// 	errorHandler(ret);
    //     memset(buff, 0, sizeof(buff));
    //     LCD_home();
    //     LCD_clearScreen();
    //     // LCD_writeStr("16x2 I2C LCD");
    //     // vTaskDelay(3000 / portTICK_PERIOD_MS);
    //     // LCD_clearScreen();
    //     // LCD_writeStr("Lets Count 0-10!");
    //     // vTaskDelay(3000 / portTICK_PERIOD_MS);
    //     // LCD_clearScreen();
    //     // for (int i = 0; i < 10; i++) {
    //     //     LCD_setCursor(1, 1);
    //     //     sprintf(num, "%d", i);
    //     //     LCD_writeStr(num);
    //     //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    //     // }

    //     LCD_setCursor(2, 1);
    //     memset(buff, 0, sizeof(buff));
    //     sprintf(buff, "Temp: %d C", (int)getTemperature());
    //     printf("Temp: %d C\n", (int)getTemperature());
        

    //     /// vTaskDelay(1000 / portTICK_PERIOD_MS);
    //     // LCD_clearScreen();
    //     // LCD_home();

    //     // i2c_lcd1602_write_string(lcd_info, buff);

    //     // memset(buff, 0, sizeof(buff));
    //     // sprintf(buff, "Hum: %d C", (int)getHumidity());

    //     // i2c_lcd1602_move_cursor(lcd_info, 0, 1);
    //     // i2c_lcd1602_write_string(lcd_info, buff);

    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }
// }

// void lcd1602_task(void * pvParameter)
// {
//     // Set up I2C
//     i2c_master_init();
//     i2c_port_t i2c_num = I2C_MASTER_NUM;
//     uint8_t address = CONFIG_LCD1602_I2C_ADDRESS;

//     // Set up the SMBus
//     smbus_info_t * smbus_info = smbus_malloc();
//     ESP_ERROR_CHECK(smbus_init(smbus_info, i2c_num, address));
//     ESP_ERROR_CHECK(smbus_set_timeout(smbus_info, 1000 / portTICK_PERIOD_MS));

//     // Set up the LCD1602 device with backlight off
//     i2c_lcd1602_info_t * lcd_info = i2c_lcd1602_malloc();
//     ESP_ERROR_CHECK(i2c_lcd1602_init(lcd_info, smbus_info, true,
//                                      LCD_NUM_ROWS, LCD_NUM_COLUMNS, LCD_NUM_VISIBLE_COLUMNS));

//     ESP_ERROR_CHECK(i2c_lcd1602_reset(lcd_info));

//     char buff[17];

//     setDHTgpio(GPIO_NUM_26);

//     // i2c_lcd1602_clear(lcd_info);
//     // i2c_lcd1602_set_cursor(lcd_info, true);
//     // uint8_t c = 0;
//     // uint8_t col = 0;
//     // uint8_t row = 0;
//     while (1)
//     {
//         int ret = readDHT();
		
// 		errorHandler(ret);

//         memset(buff, 0, sizeof(buff));
//         sprintf(buff, "Temp: %d C", (int)getTemperature());

//         i2c_lcd1602_clear(lcd_info);
//         i2c_lcd1602_home(lcd_info);

//         i2c_lcd1602_write_string(lcd_info, buff);

//         memset(buff, 0, sizeof(buff));
//         sprintf(buff, "Hum: %d C", (int)getHumidity());

//         i2c_lcd1602_move_cursor(lcd_info, 0, 1);
//         i2c_lcd1602_write_string(lcd_info, buff);

//         vTaskDelay(1000 / portTICK_PERIOD_MS);


//         // i2c_lcd1602_write_char(lcd_info, c);
//         // vTaskDelay(100 / portTICK_RATE_MS);
//         // ESP_LOGD(TAG, "col %d, row %d, char 0x%02x", col, row, c);
//         // ++c;
//         // ++col;
//         // if (col >= LCD_NUM_VISIBLE_COLUMNS)
//         // {
//         //     ++row;
//         //     if (row >= LCD_NUM_ROWS)
//         //     {
//         //         row = 0;
//         //     }
//         //     col = 0;
//         //     i2c_lcd1602_move_cursor(lcd_info, col, row);
//         // }
//     }

//     vTaskDelete(NULL);
// }

// void app_main()
// {
//     xTaskCreate(&lcd1602_task, "lcd1602_task", 4096, NULL, 5, NULL);
// }


// void app_main()
// {
//     nvs_flash_init();
//     vTaskDelay(2000 / portTICK_PERIOD_MS);
//     LCD_init(LCD_ADDR, SDA_PIN, SCL_PIN, LCD_COLS, LCD_ROWS);
//     xTaskCreate(&LCD_DemoTask, "Demo Task", 2048, NULL, 5, NULL);
// }



void Reader_task(void *pvParameter)
{
    // DHT22
	setDHTgpio(GPIO_NUM_26);

    // MQ135
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_0, ADC_WIDTH_BIT_12, 0, &adc1_chars);

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_0);  // GPIO32


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
		// printf("The adc1 value: %d\n", adc1_get_raw(ADC1_CHANNEL_4));
        getAQI_Correctval(&aqi, getHumidity(), getTemperature(), adc1_get_raw(ADC1_CHANNEL_4));
		printf("CO2: %d ppm\n", aqi.CO2);
        vTaskDelay(1000 / portTICK_PERIOD_MS);



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
	xTaskCreate(&Reader_task, "Reader_task", 2048, NULL, 5, NULL );
}
