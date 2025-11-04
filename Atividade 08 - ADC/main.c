#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

static const char *TAG = "DAQ_NTC";


// CONFIGURAÇÃO DE HARDWARE

// Pinos de LEDs
#define LED_1_GPIO       GPIO_NUM_7
#define LED_2_GPIO       GPIO_NUM_5
#define LED_3_GPIO       GPIO_NUM_4
#define LED_4_GPIO       GPIO_NUM_6

// Pinos dos botões
#define BUTTON_A_GPIO    GPIO_NUM_13  // Incrementa temperatura de alarme
#define BUTTON_B_GPIO    GPIO_NUM_14  // Decrementa temperatura de alarme

// Pino do buzzer (PWM) e sensor NTC
#define BUZZER_GPIO      GPIO_NUM_37
#define NTC_GPIO         GPIO_NUM_38

// Configuração do barramento I2C para LCD
#define I2C_MASTER_SDA_IO   GPIO_NUM_8
#define I2C_MASTER_SCL_IO   GPIO_NUM_9
#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_FREQ_HZ  100000
#define LCD_ADDR            0x27

// Configuração do ADC
#define ADC_CHANNEL     ADC_CHANNEL_2
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
#define DEBOUNCE_MS        300


// VARIÁVEIS GLOBAIS

static float alarm_temp = DEFAULT_ALARM_TEMP;
static volatile float current_temp = 0.0f;
static esp_adc_cal_characteristics_t adc_chars;

static QueueHandle_t gpio_evt_queue = NULL;
static TickType_t last_press_a = 0;
static TickType_t last_press_b = 0;


// CONVERSÃO ANALÓGICA / TEMPERATURA 

static void adc_init(void) {
  adc1_config_width(ADC_WIDTH);
  adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN);
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN, ADC_WIDTH, 1100, &adc_chars);
}

static uint32_t adc_read_mv(void) {
  int raw = adc1_get_raw(ADC_CHANNEL);
  return esp_adc_cal_raw_to_voltage(raw, &adc_chars);
}

static float ntc_to_temp(uint32_t mv) {
  float v = mv / 1000.0f;
  float vcc = 3.3f;
  float r_ntc = (NTC_R_PULLUP * v) / (vcc - v);
  float invT = (1.0 / NTC_T0_K) + (1.0 / NTC_BETA) * logf(r_ntc / NTC_R0);
  return (1.0 / invT) - 273.15f;
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

static void buzzer_on(void) {
  ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL, PWM_DUTY_ON);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL);
}

static void buzzer_off(void) {
  ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL, PWM_DUTY_OFF);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL);
}


// CONTROLE DOS LEDs

static void leds_init(void) {
  gpio_config_t cfg = {
    .mode = GPIO_MODE_OUTPUT,
    .pin_bit_mask = (1ULL << LED_1_GPIO) | (1ULL << LED_2_GPIO) |
                    (1ULL << LED_3_GPIO) | (1ULL << LED_4_GPIO)
  };
  gpio_config(&cfg);
  gpio_set_level(LED_1_GPIO, 0);
  gpio_set_level(LED_2_GPIO, 0);
  gpio_set_level(LED_3_GPIO, 0);
  gpio_set_level(LED_4_GPIO, 0);
}


// BOTÕES COM INTERRUPÇÃO E DEBOUNCE

static void IRAM_ATTR gpio_isr_handler(void* arg) {
  uint32_t gpio_num = (uint32_t)arg;
  xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void buttons_init(void) {
  gpio_config_t io_conf = {
    .intr_type = GPIO_INTR_NEGEDGE,
    .mode = GPIO_MODE_INPUT,
    .pin_bit_mask = (1ULL << BUTTON_A_GPIO) | (1ULL << BUTTON_B_GPIO),
    .pull_up_en = 1
  };
  gpio_config(&io_conf);

  gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

  gpio_install_isr_service(0);
  gpio_isr_handler_add(BUTTON_A_GPIO, gpio_isr_handler, (void*)BUTTON_A_GPIO);
  gpio_isr_handler_add(BUTTON_B_GPIO, gpio_isr_handler, (void*)BUTTON_B_GPIO);
}

static void handle_button(uint32_t gpio) {
  TickType_t now = xTaskGetTickCount();

  if (gpio == BUTTON_A_GPIO && (now - last_press_a) > pdMS_TO_TICKS(DEBOUNCE_MS)) {
    last_press_a = now;
    alarm_temp += 5;
    if (alarm_temp > 100) alarm_temp = 100;
    ESP_LOGI(TAG, "A+ -> %.1f°C", alarm_temp);
  }
  else if (gpio == BUTTON_B_GPIO && (now - last_press_b) > pdMS_TO_TICKS(DEBOUNCE_MS)) {
    last_press_b = now;
    alarm_temp -= 5;
    if (alarm_temp < 0) alarm_temp = 0;
    ESP_LOGI(TAG, "B- -> %.1f°C", alarm_temp);
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
#define PCF8574_RS         (1 << 0)

static esp_err_t i2c_master_init(void) {
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

// Funções de envio ao LCD
static esp_err_t pcf_write_byte(uint8_t data) {
  return i2c_master_write_to_device(I2C_MASTER_NUM, LCD_ADDR, &data, 1, 100 / portTICK_PERIOD_MS);
}

static void lcd_pulse_enable(uint8_t data) {
  pcf_write_byte(data | PCF8574_EN);
  vTaskDelay(pdMS_TO_TICKS(1));
  pcf_write_byte(data & ~PCF8574_EN);
  vTaskDelay(pdMS_TO_TICKS(1));
}

static void lcd_write4bits(uint8_t nibble, uint8_t flags) {
  uint8_t data = (nibble & 0xF0) | (flags & (PCF8574_RS | PCF8574_BACKLIGHT));
  pcf_write_byte(data);
  lcd_pulse_enable(data);
}

static void lcd_send_cmd_byte(uint8_t cmd) {
  uint8_t back = PCF8574_BACKLIGHT;
  lcd_write4bits(cmd & 0xF0, back);
  lcd_write4bits((cmd << 4) & 0xF0, back);
}

static void lcd_send_data_byte(uint8_t data) {
  uint8_t back_rs = PCF8574_BACKLIGHT | PCF8574_RS;
  lcd_write4bits(data & 0xF0, back_rs);
  lcd_write4bits((data << 4) & 0xF0, back_rs);
}

void lcd_init(void) {
  vTaskDelay(pdMS_TO_TICKS(50));
  uint8_t back = PCF8574_BACKLIGHT;
  lcd_write4bits(0x30, back); vTaskDelay(pdMS_TO_TICKS(5));
  lcd_write4bits(0x30, back); vTaskDelay(pdMS_TO_TICKS(1));
  lcd_write4bits(0x30, back); vTaskDelay(pdMS_TO_TICKS(1));
  lcd_write4bits(0x20, back); vTaskDelay(pdMS_TO_TICKS(1));

  lcd_send_cmd_byte(0x28);
  lcd_send_cmd_byte(0x08);
  lcd_send_cmd_byte(0x01);
  vTaskDelay(pdMS_TO_TICKS(2));
  lcd_send_cmd_byte(0x06);
  lcd_send_cmd_byte(0x0C);
}

void lcd_set_cursor(uint8_t row, uint8_t col) {
  uint8_t row_addr[] = {0x00, 0x40, 0x14, 0x54};
  lcd_send_cmd_byte(0x80 | (row_addr[row] + col));
}

void lcd_send_string(const char *str) {
  while (*str) lcd_send_data_byte((uint8_t)*str++);
}


//  MONITOR PRINCIPAL 

static void monitor_task(void* arg) {
  bool buzzer_on_state = false;
  bool blink = false;

  for (;;) {
    uint32_t mv = adc_read_mv();
    current_temp = ntc_to_temp(mv);
    float diff = alarm_temp - current_temp;

    char l1[20], l2[20];
    sprintf(l1, "Temp: %.1f C", current_temp);
    sprintf(l2, "Alarm: %.1f C", alarm_temp);

    lcd_send_cmd_byte(0x01);
    vTaskDelay(pdMS_TO_TICKS(2));
    lcd_send_cmd_byte(0x80);
    lcd_send_string(l1);
    lcd_send_cmd_byte(0xC0);
    lcd_send_string(l2);

    if (current_temp >= alarm_temp) {
      if (!buzzer_on_state) {
        buzzer_on();
        buzzer_on_state = true;
      }
      blink = !blink;
      gpio_set_level(LED_1_GPIO, blink);
      gpio_set_level(LED_2_GPIO, blink);
      gpio_set_level(LED_3_GPIO, blink);
      gpio_set_level(LED_4_GPIO, blink);
    } else {
      if (buzzer_on_state) {
        buzzer_off();
        buzzer_on_state = false;
      }
      gpio_set_level(LED_1_GPIO, (diff <= 20));
      gpio_set_level(LED_2_GPIO, (diff <= 15));
      gpio_set_level(LED_3_GPIO, (diff <= 10));
      gpio_set_level(LED_4_GPIO, (diff <= 2));
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}


// FUNÇÃO PRINCIPAL 

void app_main(void) {
  ESP_LOGI(TAG, "Inicializando sistema...");

  adc_init();
  leds_init();
  pwm_init();
  buttons_init();
  i2c_master_init();

  vTaskDelay(pdMS_TO_TICKS(50));
  lcd_init();

  xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);
  xTaskCreate(monitor_task, "monitor_task", 4096, NULL, 5, NULL);
}
