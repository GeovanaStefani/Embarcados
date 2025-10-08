#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

#define TAG "CONTADOR"

#define LED0 GPIO_NUM_4
#define LED1 GPIO_NUM_5
#define LED2 GPIO_NUM_15
#define LED3 GPIO_NUM_16

#define BOTAO_A GPIO_NUM_18  
#define BOTAO_B GPIO_NUM_19 

#define DEBOUNCE_TIME_MS 200

static uint8_t contador = 0;

void atualizar_leds(uint8_t valor) {
    gpio_set_level(LED0, valor & 0x01);
    gpio_set_level(LED1, (valor >> 1) & 0x01);
    gpio_set_level(LED2, (valor >> 2) & 0x01);
    gpio_set_level(LED3, (valor >> 3) & 0x01);
}

void incrementar_por(uint8_t passo) {
    contador = (contador + passo) & 0x0F;
    ESP_LOGI(TAG, "Incremento: +%d => Contador: %d (0x%X)", passo, contador, contador);
    atualizar_leds(contador);
}

void tarefa_botoes(void *pvParameter) {
    int prevA = 1, prevB = 1;
    int curA, curB;
    int64_t lastA = 0, lastB = 0;

    while (1) {
        curA = gpio_get_level(BOTAO_A);
        curB = gpio_get_level(BOTAO_B);

        int64_t now = esp_timer_get_time() / 1000;

        if (prevA == 1 && curA == 0) {
            if (now - lastA > DEBOUNCE_TIME_MS) {
                incrementar_por(1);
                lastA = now;
            }
        }

        if (prevB == 1 && curB == 0) {
            if (now - lastB > DEBOUNCE_TIME_MS) {
                incrementar_por(2);
                lastB = now;
            }
        }

        prevA = curA;
        prevB = curB;

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void) {
    // Configura LEDs
    gpio_reset_pin(LED0); gpio_set_direction(LED0, GPIO_MODE_OUTPUT);
    gpio_reset_pin(LED1); gpio_set_direction(LED1, GPIO_MODE_OUTPUT);
    gpio_reset_pin(LED2); gpio_set_direction(LED2, GPIO_MODE_OUTPUT);
    gpio_reset_pin(LED3); gpio_set_direction(LED3, GPIO_MODE_OUTPUT);

    // Configura botões como input com pull-up (assumindo que botões conectam ao GND)
    gpio_reset_pin(BOTAO_A); gpio_set_direction(BOTAO_A, GPIO_MODE_INPUT); gpio_pullup_en(BOTAO_A);
    gpio_reset_pin(BOTAO_B); gpio_set_direction(BOTAO_B, GPIO_MODE_INPUT); gpio_pullup_en(BOTAO_B);

    // Inicializa LEDs
    atualizar_leds(contador);

    // Cria tarefa de leitura dos botões
    xTaskCreate(&tarefa_botoes, "tarefa_botoes", 2048, NULL, 5, NULL);
}
