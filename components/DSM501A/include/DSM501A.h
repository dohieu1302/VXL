#ifndef DSM501A_SETUP_H
#define DSM501A_SETUP_H

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

#define DSM501a_RED_PIN         GPIO_NUM_34 // Dsm501a vout2 sensitivity in 1 micrometro
#define DSM501a_YELLOW_PIN      GPIO_NUM_35 // DSM501A Vout1 Sensitivity in 2.5 micrometro, if open (black) pin
#define GPIO_INPUT_PIN_SEL      (uint64_t)((1ULL<<DSM501a_RED_PIN) | (1ULL<<DSM501a_YELLOW_PIN))
#define ESP_INTR_FLAG_DEFAULT   0

extern volatile unsigned int previous_state_v2;
extern volatile unsigned int pulses_v2;
extern volatile unsigned long falling_edge_time_v2;

extern volatile unsigned int previous_state_v1;
extern volatile unsigned int pulses_v1;
extern volatile unsigned long falling_edge_time_v1;

void IRAM_ATTR change_v2_isr (void *arg);  //1 micrometro
void IRAM_ATTR change_v1_isr (void *arg);  //2.5 micrometro
// void dsm501a_task (void *pvParameters);

#endif /* DSM501A_SETUP_H */