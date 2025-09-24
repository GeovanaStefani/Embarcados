#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define LED1 GPIO_NUM_4
#define LED2 GPIO_NUM_5
#define LED3 GPIO_NUM_15
#define LED4 GPIO_NUM_16

#define DELAY_MS 500

static const gpio_num_t leds[] = {LED1, LED2, LED3, LED4};

void leds_init(void) {
    for (int i = 0; i < 4; i++) {
        gpio_reset_pin(leds[i]);
        gpio_set_direction(leds[i], GPIO_MODE_OUTPUT);
        gpio_set_level(leds[i], 0);
    }
}

void led_set(int index, int state) {
    gpio_set_level(leds[index], state);
}

void binary_phase(void) {
    for (int count = 0; count < 16; count++) {
        for (int i = 0; i < 4; i++) {
            int bit = (count >> i) & 0x01;
            led_set(i, bit);
        }
        vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
    }
}

void sweep_phase(void) {
    // ida
    for (int i = 0; i < 4; i++) {
        led_set(i, 1);
        vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
        led_set(i, 0);
    }
    // volta
    for (int i = 2; i >= 0; i--) {
        led_set(i, 1);
        vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
        led_set(i, 0);
    }
}

void app_main(void) {
    leds_init();

    while (1) {
        binary_phase();
        sweep_phase();
    }
}
