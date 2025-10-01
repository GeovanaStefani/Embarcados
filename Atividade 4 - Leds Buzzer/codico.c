#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"

#define LED1_GPIO 4
#define LED2_GPIO 5
#define LED3_GPIO 15
#define LED4_GPIO 16
#define BUZZER_GPIO 27

#define LED_FREQ       1000   
#define BUZZER_FREQ    1000  
#define LED_RES        LEDC_TIMER_8_BIT  
#define BUZZER_RES     LEDC_TIMER_10_BIT

#define DELAY_MS 10

#define DUTY_MAX   255
#define DUTY_MIN   0

ledc_channel_config_t led_channels[5];

void ledc_init() {
  
    ledc_timer_config_t led_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LED_RES,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = LED_FREQ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&led_timer);

  
    ledc_timer_config_t buzzer_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = BUZZER_RES,
        .timer_num        = LEDC_TIMER_1,
        .freq_hz          = BUZZER_FREQ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&buzzer_timer);


    int led_gpios[4] = {LED1_GPIO, LED2_GPIO, LED3_GPIO, LED4_GPIO};
    for (int i = 0; i < 4; i++) {
        led_channels[i].channel    = i;
        led_channels[i].duty       = 0;
        led_channels[i].gpio_num   = led_gpios[i];
        led_channels[i].speed_mode = LEDC_LOW_SPEED_MODE;
        led_channels[i].hpoint     = 0;
        led_channels[i].timer_sel  = LEDC_TIMER_0;
        ledc_channel_config(&led_channels[i]);
    }

    
    led_channels[4].channel    = 4;
    led_channels[4].duty       = 512; 
    led_channels[4].gpio_num   = BUZZER_GPIO;
    led_channels[4].speed_mode = LEDC_LOW_SPEED_MODE;
    led_channels[4].hpoint     = 0;
    led_channels[4].timer_sel  = LEDC_TIMER_1;
    ledc_channel_config(&led_channels[4]);
}

void set_led_duty(int led_index, int percent) {

    if (led_index < 0 || led_index > 3) return;
    int duty = (percent * DUTY_MAX) / 100;
    ledc_set_duty(led_channels[led_index].speed_mode,
                  led_channels[led_index].channel,
                  duty);
    ledc_update_duty(led_channels[led_index].speed_mode,
                     led_channels[led_index].channel);
}

void set_buzzer_freq(int freq) {
  
    ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_1, freq);
}

void fase1_fading_sincronizado() {
   
    for (int duty = DUTY_MIN; duty <= DUTY_MAX; duty++) {
        for (int i = 0; i < 4; i++) {
            ledc_set_duty(led_channels[i].speed_mode, led_channels[i].channel, duty);
            ledc_update_duty(led_channels[i].speed_mode, led_channels[i].channel);
        }
        vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
    }
    for (int duty = DUTY_MAX; duty >= DUTY_MIN; duty--) {
        for (int i = 0; i < 4; i++) {
            ledc_set_duty(led_channels[i].speed_mode, led_channels[i].channel, duty);
            ledc_update_duty(led_channels[i].speed_mode, led_channels[i].channel);
        }
        vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
    }
}

void fase2_fading_sequencial() {

    for (int i = 0; i < 4; i++) {
        for (int duty = DUTY_MIN; duty <= DUTY_MAX; duty++) {
            ledc_set_duty(led_channels[i].speed_mode, led_channels[i].channel, duty);
            ledc_update_duty(led_channels[i].speed_mode, led_channels[i].channel);
            vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
        }
        for (int duty = DUTY_MAX; duty >= DUTY_MIN; duty--) {
            ledc_set_duty(led_channels[i].speed_mode, led_channels[i].channel, duty);
            ledc_update_duty(led_channels[i].speed_mode, led_channels[i].channel);
            vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
        }
    }
}

void fase3_buzzer_test() {
    for (int freq = 500; freq <= 2000; freq += 20) {
        set_buzzer_freq(freq);
        vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
    }
    for (int freq = 2000; freq >= 500; freq -= 20) {
        set_buzzer_freq(freq);
        vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
    }
}

void app_main(void) {
    ledc_init();

    while (1) {
        printf("Fase 1: Fading Sincronizado\n");
        fase1_fading_sincronizado();

        printf("Fase 2: Fading Sequencial\n");
        fase2_fading_sequencial();

        printf("Fase 3: Teste Buzzer\n");
        fase3_buzzer_test();
    }
}
