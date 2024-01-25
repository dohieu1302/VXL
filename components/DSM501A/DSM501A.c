#include "DSM501A.h"
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "esp_timer.h"

volatile unsigned int previous_state_v2 = 1;
volatile unsigned int pulses_v2 = 0;
volatile unsigned long falling_edge_time_v2 = 0;

volatile unsigned int previous_state_v1 = 1;
volatile unsigned int pulses_v1 = 0;
volatile unsigned long falling_edge_time_v1 = 0;


void IRAM_ATTR change_v2_isr (void *arg) {
    uint32_t gpio_status = gpio_get_level((gpio_num_t)(uint32_t) arg);
    if(gpio_status == previous_state_v2) return;
    else if (gpio_status == 0) {
        falling_edge_time_v2 = esp_timer_get_time();
        previous_state_v2 = 0;
    } else {
        previous_state_v2 = 1;
        pulses_v2 += esp_timer_get_time() - falling_edge_time_v2;
    }
}

void IRAM_ATTR change_v1_isr (void *arg) {
    uint32_t gpio_status = gpio_get_level((gpio_num_t)(uint32_t) arg);
    if(gpio_status == previous_state_v1) return;
    else if (gpio_status == 0) {
        falling_edge_time_v1 = esp_timer_get_time();
        previous_state_v1 = 0;
    } else {
        previous_state_v1 = 1;
        pulses_v1 += esp_timer_get_time() - falling_edge_time_v1;
    }
}
