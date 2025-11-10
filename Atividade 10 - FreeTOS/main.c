#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "esp_log.h"
#include "esp_system.h"

static const char *TAG = "DAQ_NTC_FSM";

// CONFIGURAÇÃO DE HARDWARE

// Pinos de LEDs
#define LED_1_GPIO       GPIO_NUM_7
#define LED_2_GPIO       GPIO_NUM_5
#define LED_3_GPIO       GPIO_NUM_4
#define LED_4_GPIO       GPIO_NUM_6

// Pinos dos botões
#define BUTTON_A_GPIO    GPIO_NUM_13 // Incrementa temperatura de alarme
#define BUTTON_B_GPIO    GPIO_NUM_14 // Decrementa temperatura de alarme

// Pino do buzzer (PWM) e sensor NTC
#define BUZZER_GPIO      GPIO_NUM_37
#define NTC_GPIO         GPIO_NUM_9

// Configuração do barramento I2C para LCD
#define I2C_MASTER_SDA_IO   GPIO_NUM_1
#define I2C_MASTER_SCL_IO   GPIO_NUM_2
#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_FREQ_HZ  100000
#define LCD_ADDR            0x27

// Configuração do ADC
#define ADC_CHANNEL     ADC_CHANNEL_8
#define ADC_ATTEN       ADC_ATTEN_DB_11
#define ADC_WIDTH       ADC_WIDTH_BIT_12

// Parâmetros do termistor NTC
#define NTC_R_PULLUP   10000.0f
#define NTC_R0         10000.0f
#define NTC_T0_K       298.15f
#define NTC_BETA       3950.0f

// Parâmetros gerais do sistema
#define DEFAULT_ALARM_TEMP 25.0f
#define PWM_FREQ_HZ        2000
#define PWM_CHANNEL        LEDC_CHANNEL_0
#define PWM_TIMER          LEDC_TIMER_0
#define PWM_DUTY_ON        4000
#define PWM_DUTY_OFF       0
#define DEBOUNCE_MS 300

// Pinos conectados aos segmentos
#define SEG_A GPIO_NUM_18
#define SEG_B GPIO_NUM_19
#define SEG_C GPIO_NUM_20
#define SEG_D GPIO_NUM_21
#define SEG_E GPIO_NUM_11
#define SEG_F GPIO_NUM_12
#define SEG_G GPIO_NUM_3


#define MOUNT_POINT "/sdcard"

// VARIÁVEIS GLOBAIS

static float alarm_temp = DEFAULT_ALARM_TEMP;
static volatile float current_temp = 0.0f;
static esp_adc_cal_characteristics_t adc_chars;

// ESTADOS DA MÁQUINA
typedef enum {
    STATE_INIT = 0,
    STATE_READ_TEMP,
    STATE_UPDATE_LCD,
    STATE_UPDATE_LEDS,
    STATE_ALARM_CONTROL,
    STATE_LOG_SD
} system_state_t;

static system_state_t current_state = STATE_INIT;

static QueueHandle_t gpio_evt_queue = NULL;
static TickType_t last_press_a = 0;
static TickType_t last_press_b = 0;

void lcd_init(void);
void lcd_set_cursor(uint8_t row, uint8_t col);
void lcd_send_string(const char *str);
static esp_err_t i2c_master_init(void);
static void adc_init(void);
static uint32_t adc_read_mv(void);
static float ntc_to_temp(uint32_t mv);
static void pwm_init(void);
static void buzzer_on(void);
static void buzzer_off(void);
static void leds_init(void);
static void buttons_init(void);
static void button_task(void* arg);
static void sdcard_init(void);
static void log_to_sd(float temp, float alarm);
static void update_lcd(void);
static void update_leds(void);
static void update_buzzer(void);

// CONVERSÃO ANALÓGICA / TEMPERATURA 

static void adc_init(void) {
    adc1_config_width(ADC_WIDTH);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN);
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN, ADC_WIDTH, 1100, &adc_chars);
}
static uint32_t adc_read_mv(void){
    int raw = adc1_get_raw(ADC_CHANNEL);
    return esp_adc_cal_raw_to_voltage(raw, &adc_chars);
}
static float ntc_to_temp(uint32_t mv) {
    float v = mv / 1000.0f;
    float vcc = 3.3f;
    if (v <= 0.01f) return -273.15f;
    float r_ntc = (NTC_R_PULLUP * v) / (vcc - v);
    float invT = (1.0f/NTC_T0_K) + (1.0f/NTC_BETA) * logf(r_ntc / NTC_R0);
    return (1.0f/invT) - 273.15f;
}

// CONTROLE DO BUZZER (PWM)

static void pwm_init(void) {
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .timer_num = PWM_TIMER,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);
    ledc_channel_config_t channel = {
        .gpio_num = BUZZER_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = PWM_CHANNEL,
        .timer_sel = PWM_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&channel);
}
static void buzzer_on(void){
    ledc_set_duty(LEDC_LOW_SPEED_MODE,PWM_CHANNEL,PWM_DUTY_ON);
    ledc_update_duty(LEDC_LOW_SPEED_MODE,PWM_CHANNEL);
}
static void buzzer_off(void){
    ledc_set_duty(LEDC_LOW_SPEED_MODE,PWM_CHANNEL,PWM_DUTY_OFF);
    ledc_update_duty(LEDC_LOW_SPEED_MODE,PWM_CHANNEL);
}

// CONTROLE DOS LEDs

static void leds_init(void){
    gpio_config_t cfg = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL<<LED_1_GPIO)|(1ULL<<LED_2_GPIO)|(1ULL<<LED_3_GPIO)|(1ULL<<LED_4_GPIO)
    };
    gpio_config(&cfg);
    gpio_set_level(LED_1_GPIO,0);
    gpio_set_level(LED_2_GPIO,0);
    gpio_set_level(LED_3_GPIO,0);
    gpio_set_level(LED_4_GPIO,0);
}


// DISPLAY DE 7 SEGMENTOS (COMUM CÁTODO)

static const gpio_num_t segments[] = {
    SEG_A, SEG_B, SEG_C, SEG_D, SEG_E, SEG_F, SEG_G
};

// Mapas dos dígitos (1 = segmento ligado)
static const uint8_t digit_map[6][7] = {
    // a, b, c, d, e, f, g
    {1, 1, 1, 1, 1, 1, 0}, // 0
    {1, 1, 1, 1, 0, 0, 1}, // 3
    {1, 1, 1, 0, 0, 0, 1}, // 7
    {0, 0, 1, 1, 1, 1, 1}, // D
    {1, 0, 0, 1, 1, 1, 1}, // F
    {0, 0, 0, 0, 0, 0, 0}  // apagado
};

typedef enum {
    DISP_0,
    DISP_3,
    DISP_7,
    DISP_D,
    DISP_F,
    DISP_OFF
} disp_digit_t;

static void display_init(void) {
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << SEG_A) | (1ULL << SEG_B) | (1ULL << SEG_C) |
                        (1ULL << SEG_D) | (1ULL << SEG_E) | (1ULL << SEG_F) | (1ULL << SEG_G)
    };
    gpio_config(&io_conf);
    for (int i = 0; i < 7; i++) gpio_set_level(segments[i], 0);
}

static void display_show_digit(disp_digit_t d) {
    for (int i = 0; i < 7; i++) {
        gpio_set_level(segments[i], digit_map[d][i]);
    }
}

// Atualiza o display conforme temperatura
static void update_display(void) {
    float diff = alarm_temp - current_temp;
    static bool blink = false;

    if (current_temp >= alarm_temp) {
        blink = !blink;
        if (blink)
            display_show_digit(DISP_F);
        else
            display_show_digit(DISP_OFF);
    } else if (diff >= 20) display_show_digit(DISP_0);
    else if (diff >= 15) display_show_digit(DISP_3);
    else if (diff >= 10) display_show_digit(DISP_7);
    else if (diff >= 2) display_show_digit(DISP_D);
    else display_show_digit(DISP_F);
}


// BOTÕES COM INTERRUPÇÃO E DEBOUNCE

static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}
static void buttons_init(void){
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL<<BUTTON_A_GPIO)|(1ULL<<BUTTON_B_GPIO),
        .pull_up_en = 1
    };
    gpio_config(&io_conf);
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    if (gpio_evt_queue == NULL) {
        ESP_LOGE(TAG, "Falha ao criar fila dos botões!");
        return;
    }
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_A_GPIO, gpio_isr_handler, (void*)BUTTON_A_GPIO);
    gpio_isr_handler_add(BUTTON_B_GPIO, gpio_isr_handler, (void*)BUTTON_B_GPIO);
    xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);
}
static void handle_button(uint32_t gpio) {
    TickType_t now = xTaskGetTickCount();
    if (gpio == BUTTON_A_GPIO) {
        if ((now - last_press_a) > pdMS_TO_TICKS(DEBOUNCE_MS)) {
            last_press_a = now;
            alarm_temp += 5.0f;
            if (alarm_temp > 100.0f) alarm_temp = 100.0f;
        }
    } else if (gpio == BUTTON_B_GPIO) {
        if ((now - last_press_b) > pdMS_TO_TICKS(DEBOUNCE_MS)) {
            last_press_b = now;
            alarm_temp -= 5.0f;
            if (alarm_temp < 0.0f) alarm_temp = 0.0f;
        }
    }
}
static void button_task(void* arg) {
    uint32_t io_num;
    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            vTaskDelay(pdMS_TO_TICKS(20));
            if (gpio_get_level(io_num) == 0) handle_button(io_num);
        }
    }
}

// LCD VIA I2C

// Mapeamento do PCF8574
#define PCF8574_BACKLIGHT  (1 << 3)
#define PCF8574_EN         (1 << 2)
#define PCF8574_RW         (1 << 1)
#define PCF8574_RS         (1 << 0)

// Funções de envio ao LCD
static esp_err_t pcf_write_byte(uint8_t data){
    return i2c_master_write_to_device(I2C_MASTER_NUM, LCD_ADDR, &data, 1, 100 / portTICK_PERIOD_MS);
}
static void lcd_pulse_enable(uint8_t data){
    uint8_t en_high = data | PCF8574_EN;
    uint8_t en_low  = data & ~PCF8574_EN;
    pcf_write_byte(en_high);
    vTaskDelay(pdMS_TO_TICKS(1));
    pcf_write_byte(en_low);
    vTaskDelay(pdMS_TO_TICKS(1));
}
static void lcd_write4bits(uint8_t nibble, uint8_t flags){
    uint8_t data = (nibble & 0xF0) | (flags & (PCF8574_RS | PCF8574_BACKLIGHT));
    pcf_write_byte(data);
    lcd_pulse_enable(data);
}
static void lcd_send_cmd_byte(uint8_t cmd){
    uint8_t back = PCF8574_BACKLIGHT;
    lcd_write4bits(cmd & 0xF0, back);
    lcd_write4bits((cmd << 4) & 0xF0, back);
}
static void lcd_send_data_byte(uint8_t data){
    uint8_t back_rs = PCF8574_BACKLIGHT | PCF8574_RS;
    lcd_write4bits(data & 0xF0, back_rs);
    lcd_write4bits((data << 4) & 0xF0, back_rs);
}
void lcd_init(void){
    vTaskDelay(pdMS_TO_TICKS(50));
    uint8_t back = PCF8574_BACKLIGHT;
    lcd_write4bits(0x30, back);
    vTaskDelay(pdMS_TO_TICKS(5));
    lcd_write4bits(0x30, back);
    vTaskDelay(pdMS_TO_TICKS(1));
    lcd_write4bits(0x30, back);
    vTaskDelay(pdMS_TO_TICKS(1));
    lcd_write4bits(0x20, back);
    vTaskDelay(pdMS_TO_TICKS(1));
    lcd_send_cmd_byte(0x28);
    vTaskDelay(pdMS_TO_TICKS(1));
    lcd_send_cmd_byte(0x08);
    vTaskDelay(pdMS_TO_TICKS(1));
    lcd_send_cmd_byte(0x01);
    vTaskDelay(pdMS_TO_TICKS(2));
    lcd_send_cmd_byte(0x06);
    vTaskDelay(pdMS_TO_TICKS(1));
    lcd_send_cmd_byte(0x0C);
    vTaskDelay(pdMS_TO_TICKS(1));
}
void lcd_set_cursor(uint8_t row, uint8_t col){
    uint8_t row_addr[] = { 0x00, 0x40, 0x14, 0x54 };
    uint8_t addr = 0x80 | (row_addr[row] + col);
    lcd_send_cmd_byte(addr);
    vTaskDelay(pdMS_TO_TICKS(1));
}
void lcd_send_string(const char *str){
    while (*str) lcd_send_data_byte((uint8_t)*str++);
}
static esp_err_t i2c_master_init(void){
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) return err;
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// Inicialização do SDCard e gravação dos logs
static void sdcard_init(void) {
    esp_err_t ret;
    sdmmc_card_t *card;
    const char mount_point[] = MOUNT_POINT;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = GPIO_NUM_17,
        .miso_io_num = GPIO_NUM_16,
        .sclk_io_num = GPIO_NUM_15,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    ret = spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) return;
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = GPIO_NUM_10;
    slot_config.host_id = host.slot;
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) return;
    sdmmc_card_print_info(stdout, card);
}
static void log_to_sd(float temp, float alarm) {
    FILE *f = fopen(MOUNT_POINT "/log_temp.csv", "a");
    if (f == NULL) return;
    time_t now;
    time(&now);
    fprintf(f, "%ld,%.2f,%.2f\n", (long)now, temp, alarm);
    fclose(f);
}

// Atualização do display
static void update_lcd(void) {
    char l1[20], l2[20];
    snprintf(l1, sizeof(l1), "Temp: %.1f C", current_temp);
    snprintf(l2, sizeof(l2), "Alarm: %.1f C", alarm_temp);
    lcd_send_cmd_byte(0x01);
    vTaskDelay(pdMS_TO_TICKS(2));
    lcd_set_cursor(0,0);
    lcd_send_string(l1);
    lcd_set_cursor(1,0);
    lcd_send_string(l2);
}

// Atualização dos LEDs
static void update_leds(void) {
    float diff = alarm_temp - current_temp;
    if (current_temp >= alarm_temp) {
        static bool blink = false;
        blink = !blink;
        gpio_set_level(LED_1_GPIO, blink);
        gpio_set_level(LED_2_GPIO, blink);
        gpio_set_level(LED_3_GPIO, blink);
        gpio_set_level(LED_4_GPIO, blink);
    } else {
        gpio_set_level(LED_1_GPIO, (diff <= 20));
        gpio_set_level(LED_2_GPIO, (diff <= 15));
        gpio_set_level(LED_3_GPIO, (diff <= 10));
        gpio_set_level(LED_4_GPIO, (diff <= 2));
    }
}

// Controle do buzzer
static void update_buzzer(void) {
    if (current_temp >= alarm_temp) buzzer_on();
    else buzzer_off();
}

// FreeRTOS

typedef struct {
    float temp;
    float alarm;
} sensor_data_t;

static QueueHandle_t temp_queue;

// TASK DE LEITURA DO SENSOR
static void task_read_temp(void *arg) {
    for (;;) {
        current_temp = ntc_to_temp(adc_read_mv());
        sensor_data_t data = { current_temp, alarm_temp };
        xQueueSend(temp_queue, &data, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// TASK DE LCD
static void task_lcd(void *arg) {
    sensor_data_t data;
    for (;;) {
        if (xQueueReceive(temp_queue, &data, portMAX_DELAY)) {
            current_temp = data.temp;
            alarm_temp   = data.alarm;
            update_lcd();
        }
    }
}

// TASK DE LEDS E DISPLAY 7 SEG
static void task_leds(void *arg) {
    for (;;) {
        update_leds();
        update_display();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// TASK DO BUZZER
static void task_buzzer(void *arg) {
    for (;;) {
        update_buzzer();
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// TASK DE LOG NO SD
static void task_logger(void *arg) {
    for (;;) {
        log_to_sd(current_temp, alarm_temp);
        vTaskDelay(pdMS_TO_TICKS(5000)); // Log a cada 5s
    }
}

// FUNÇÃO PRINCIPAL
void app_main(void) {
    adc_init();
    leds_init();
    pwm_init();
    i2c_master_init();
    lcd_init();
    display_init();
    sdcard_init();
    buttons_init();

    temp_queue = xQueueCreate(5, sizeof(sensor_data_t));

    xTaskCreate(task_read_temp, "task_read_temp", 4096, NULL, 5, NULL);
    xTaskCreate(task_lcd, "task_lcd", 4096, NULL, 4, NULL);
    xTaskCreate(task_leds, "task_leds", 2048, NULL, 3, NULL);
    xTaskCreate(task_buzzer, "task_buzzer", 2048, NULL, 3, NULL);
    xTaskCreate(task_logger, "task_logger", 4096, NULL, 2, NULL);
    xTaskCreate(button_task, "task_buttons", 2048, NULL, 6, NULL);
}
