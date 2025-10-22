#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

#define I2C_MASTER_SCL_IO 9        // Pino SCL do I2C
#define I2C_MASTER_SDA_IO 8        // Pino SDA do I2C
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000  // Frequência I2C

#define MPU6050_ADDR 0x68          // Endereço do sensor MPU6050
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B  // Registrador de aceleração

#define SSD1306_ADDR 0x3C          // Endereço do display SSD1306
#define LED_PIN 4                  // Pino do LED
#define SAMPLE_COUNT 10            // Número de amostras para média
#define DELTA_THRESHOLD 0.5f       // Limite para detectar alteração significativa

// Fonte simplificada para números 0-9 e ponto decimal
const uint8_t font5x7[][5] = {
    {0x3E,0x51,0x49,0x45,0x3E}, // '0'
    {0x00,0x42,0x7F,0x40,0x00}, // '1'
    {0x42,0x61,0x51,0x49,0x46}, // '2'
    {0x21,0x41,0x45,0x4B,0x31}, // '3'
    {0x18,0x14,0x12,0x7F,0x10}, // '4'
    {0x27,0x45,0x45,0x45,0x39}, // '5'
    {0x3C,0x4A,0x49,0x49,0x30}, // '6'
    {0x01,0x71,0x09,0x05,0x03}, // '7'
    {0x36,0x49,0x49,0x49,0x36}, // '8'
    {0x06,0x49,0x49,0x29,0x1E}, // '9'
    {0x00,0x60,0x60,0x00,0x00}  // '.'
};

// INICIALIZAÇÃO I2C 
static void i2c_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// -MPU6050
// Inicializa o sensor 
static void mpu6050_init() {
    uint8_t data[2] = {MPU6050_PWR_MGMT_1, 0x00};
    i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDR, data, 2, pdMS_TO_TICKS(100));
}

// Lê valores de aceleração nos eixos X, Y e Z
static void mpu6050_read_accel(float *ax, float *ay, float *az) {
    uint8_t reg = MPU6050_ACCEL_XOUT_H;
    uint8_t data[6];
    i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_ADDR, &reg, 1, data, 6, pdMS_TO_TICKS(100));
    int16_t raw_x = (data[0]<<8)|data[1];
    int16_t raw_y = (data[2]<<8)|data[3];
    int16_t raw_z = (data[4]<<8)|data[5];
    *ax = raw_x/16384.0*9.81;  // Converte para m/s²
    *ay = raw_y/16384.0*9.81;
    *az = raw_z/16384.0*9.81;
}

// DISPLAY SSD1306
// Envia comando para o display
static void ssd1306_cmd(uint8_t cmd){
    uint8_t buf[2]={0x00, cmd};
    i2c_master_write_to_device(I2C_MASTER_NUM, SSD1306_ADDR, buf, 2, pdMS_TO_TICKS(100));
}

// Envia dados em pixels para o display
static void ssd1306_data(uint8_t *data, size_t len){
    uint8_t buf[len+1];
    buf[0]=0x40;
    memcpy(buf+1,data,len);
    i2c_master_write_to_device(I2C_MASTER_NUM, SSD1306_ADDR, buf, len+1, pdMS_TO_TICKS(100));
}

// Inicializa o display
static void ssd1306_init() {
    uint8_t cmds[]={0xAE,0x20,0x00,0xB0,0xC8,0x00,0x10,0x40,0x81,0x7F,
                     0xA1,0xA6,0xA8,0x3F,0xA4,0xD3,0x00,0xD5,0x80,0xD9,
                     0xF1,0xDA,0x12,0xDB,0x40,0x8D,0x14,0xAF};
    for(int i=0;i<sizeof(cmds);i++) ssd1306_cmd(cmds[i]);
}

// Limpa a tela do display
static void ssd1306_clear() {
    uint8_t zeros[128]={0};
    for(int page=0; page<8; page++){
        ssd1306_cmd(0xB0 + page);
        ssd1306_cmd(0x00);
        ssd1306_cmd(0x10);
        ssd1306_data(zeros,128);
    }
}

// Desenha um caractere na tela
static void ssd1306_draw_char(uint8_t page, uint8_t col, char c){
    uint8_t index=0xFF;
    if(c>='0' && c<='9') index = c-'0';
    else if(c=='.') index=10;
    else return;
    ssd1306_cmd(0xB0 + page);
    ssd1306_cmd(0x00 + col%16);
    ssd1306_cmd(0x10 + col/16);
    ssd1306_data((uint8_t*)font5x7[index],5);
}

// Desenha uma string na tela
static void ssd1306_draw_string(uint8_t page, uint8_t col, const char* str){
    while(*str){
        ssd1306_draw_char(page,col,*str);
        col+=6; // espaçamento entre caracteres
        str++;
    }
}

// MÉDIA
// Calcula a média das últimas SAMPLE_COUNT leituras
static float media(float *buf){
    float soma=0;
    for(int i=0;i<SAMPLE_COUNT;i++) soma+=buf[i];
    return soma/SAMPLE_COUNT;
}

void app_main() {
    i2c_init();               // Inicializa I2C
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT); // Configura LED
    ssd1306_init();           // Inicializa display
    ssd1306_clear();          // Limpa display
    mpu6050_init();           // Inicializa sensor MPU6050

    float ax_buf[SAMPLE_COUNT]={0}, ay_buf[SAMPLE_COUNT]={0}, az_buf[SAMPLE_COUNT]={0};
    int idx=0;
    float last_avgx=0, last_avgy=0, last_avgz=0;
    bool led_already_on = false;

    while(1){
        float ax, ay, az;
        mpu6050_read_accel(&ax,&ay,&az);  // Lê aceleração

        // Atualiza buffers de média
        ax_buf[idx]=ax; ay_buf[idx]=ay; az_buf[idx]=az;
        idx=(idx+1)%SAMPLE_COUNT;

        float avgx=media(ax_buf);
        float avgy=media(ay_buf);
        float avgz=media(az_buf);

        // LED pisca apenas uma vez por alteração
        if((fabs(avgx - last_avgx) >= DELTA_THRESHOLD ||
            fabs(avgy - last_avgy) >= DELTA_THRESHOLD ||
            fabs(avgz - last_avgz) >= DELTA_THRESHOLD) && !led_already_on) {
            gpio_set_level(LED_PIN,1);
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level(LED_PIN,0);
            led_already_on = true; // marca que já piscou
        } else if(fabs(avgx - last_avgx) < DELTA_THRESHOLD &&
                  fabs(avgy - last_avgy) < DELTA_THRESHOLD &&
                  fabs(avgz - last_avgz) < DELTA_THRESHOLD) {
            led_already_on = false; // reseta flag
        }

        last_avgx=avgx; last_avgy=avgy; last_avgz=avgz;

        // Mostra no terminal
        printf("AVG X: %.2f | Y: %.2f | Z: %.2f\n", avgx,avgy,avgz);

        // Mostra no display
        ssd1306_clear();  // limpa tela antes de desenhar
        char buf[16];
        sprintf(buf,"X:%.2f",avgx);
        ssd1306_draw_string(0,0,buf);
        sprintf(buf,"Y:%.2f",avgy);
        ssd1306_draw_string(1,0,buf);
        sprintf(buf,"Z:%.2f",avgz);
        ssd1306_draw_string(2,0,buf);

        vTaskDelay(pdMS_TO_TICKS(200)); // espera 200ms
    }
}
