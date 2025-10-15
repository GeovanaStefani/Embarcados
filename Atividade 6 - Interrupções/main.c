#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"

// Definição dos pinos usados
#define LED1_GPIO 4
#define LED2_GPIO 5
#define BUZZER_GPIO 37
#define BUTTON_A_GPIO 13
#define BUTTON_B_GPIO 14

// Identificação para logs
static const char *TAG = "INTERRUPT_SYSTEM";

// Variáveis globais
volatile bool buzzer_enabled = true;  // Controla se o buzzer está ativo
volatile bool led1_state = false;   // Controla o estado do LED1
volatile bool led2_state = false;   // Controla o estado do LED2
volatile bool buzzer_triggered = false; //  Controla ativamento do buzzer

// Controle de debounce
static uint64_t last_press_time_a = 0;
static uint64_t last_press_time_b = 0;

// Botão A, alterna LED1
static void IRAM_ATTR buttonA_isr_handler(void *arg) {
    uint64_t now = esp_timer_get_time();
    if (now - last_press_time_a > 200000) { // debounce de 200ms
        led1_state = !led1_state;
        gpio_set_level(LED1_GPIO, led1_state);
        last_press_time_a = now;
    }
}

// Botão B, aciona buzzer
static void IRAM_ATTR buttonB_isr_handler(void *arg) {
    uint64_t now = esp_timer_get_time();
    if (now - last_press_time_b > 200000) {
        if (buzzer_enabled) {
            buzzer_triggered = true;
        }
        last_press_time_b = now;
    }
}

// Timer (pisca LED2 a cada 2s)
void timer_callback(void *arg) {
    led2_state = !led2_state;
    gpio_set_level(LED2_GPIO, led2_state);
}

// Controla o buzzer 
void buzzer_task(void *arg) {
    while (1) {
        if (buzzer_triggered) {
            gpio_set_level(BUZZER_GPIO, 1);
            vTaskDelay(1500 / portTICK_PERIOD_MS); 
            gpio_set_level(BUZZER_GPIO, 0);
            buzzer_triggered = false; 
        }
        vTaskDelay(10 / portTICK_PERIOD_MS); 
    }
}

// --- Task que lê caracteres da UART ---
void uart_task(void *arg) {
    uint8_t data;
    while (1) {
        int len = uart_read_bytes(UART_NUM_0, &data, 1, 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            if (data == 'a') {
                buzzer_enabled = false;
            } else if (data == 'b') {
                buzzer_enabled = true;
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void app_main(void) {
    // Configuração dos LEDs e buzzer 
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LED1_GPIO) | (1ULL << LED2_GPIO) | (1ULL << BUZZER_GPIO),
    };
    gpio_config(&io_conf);

    // Configuração dos botões 
    gpio_config_t btn_conf = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BUTTON_A_GPIO) | (1ULL << BUTTON_B_GPIO),
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&btn_conf);

    // Instala serviço de interrupção
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_A_GPIO, buttonA_isr_handler, NULL);
    gpio_isr_handler_add(BUTTON_B_GPIO, buttonB_isr_handler, NULL);

    // Configura e inicia o timer de 2 segundos
    const esp_timer_create_args_t timer_args = {
        .callback = &timer_callback,
        .name = "led_timer"
    };
    esp_timer_handle_t timer_handler;
    esp_timer_create(&timer_args, &timer_handler);
    esp_timer_start_periodic(timer_handler, 2000000); // 2s

    // Configura UART 
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_0, &uart_config);
    uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0);

    // Cria as tasks 
    xTaskCreate(uart_task, "uart_task", 2048, NULL, 5, NULL);
    xTaskCreate(buzzer_task, "buzzer_task", 2048, NULL, 5, NULL);


    // Loop principal
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
