#include <stdio.h>
#include "driver/ledc.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "esp_timer.h"
#include <mpu6050.h>
#include "driver/i2c_master.h"
#include "math.h"
#include "driver/i2c.h"
#include "vl53l1x.h"


// Definições para o vl53l1x
#define I2C_MASTER_SCL_IO          9          // Pino SCL
#define I2C_MASTER_SDA_IO          8          // Pino SDA
#define I2C_MASTER_NUM             I2C_NUM_1
#define I2C_MASTER_FREQ_HZ         400000     // 400kHz
#define XSHUT_PIN -1                          // -1 se não estiver usando o pino XSHUT
#define SENSOR_ADDRESS 0x29                   // Endereço I2C padrão do VL53L1X

// Definições para o MPU6050
#define ADDR MPU6050_I2C_ADDRESS_LOW
// Max 1MHz for esp-idf, but device supports up to 1.7Mhz
#define I2C_FREQ_HZ (400000) //Padrão da biblioteca esp-idf-lib é 1000000 1MHz
#define I2C_SDA 12
#define I2C_SCL 13

// Definições dos pinos que são utilizados para os motores
#define MOTOR1 GPIO_NUM_1
#define MOTOR2 GPIO_NUM_2
#define MOTOR3 GPIO_NUM_3
#define MOTOR4 GPIO_NUM_4

#define PWM_MIN 0
#define PWM_MAX 255

static const char *TAG_MPU = "mpu6050_test";
static const char *TAG = "vespa";

mpu6050_dev_t dev_mpu6050 = { 0 };
QueueHandle_t xQueue_acel_gyro;
uint16_t altura;


static void ledc_pwm_init(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_8_BIT,
        .freq_hz          = 10000,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = MOTOR1,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ledc_channel.channel = LEDC_CHANNEL_1;
    ledc_channel.gpio_num = MOTOR2;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ledc_channel.channel = LEDC_CHANNEL_2;
    ledc_channel.gpio_num = MOTOR3;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ledc_channel.channel = LEDC_CHANNEL_3;
    ledc_channel.gpio_num = MOTOR4;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void pwm_set_duty(ledc_channel_t channel, uint32_t duty) 
{
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
}


void calcula_pwm_motores(float throttle, float roll, float pitch, float yaw,
                         uint8_t *motor1, uint8_t *motor2,
                         uint8_t *motor3, uint8_t *motor4)
{
    // Distribuição:
    // M1: frente-esquerda (CW)
    // M2: frente-direita (CCW)
    // M3: trás-direita (CW)
    // M4: trás-esquerda (CCW)

    // Cálculo base: Throttle +- correções PID
    float m1 = throttle + pitch + roll - yaw; // frente-esquerda (CW)
    float m2 = throttle + pitch - roll + yaw; // frente-direita (CCW)
    float m3 = throttle - pitch - roll - yaw; // trás-direita (CW)
    float m4 = throttle - pitch + roll + yaw; // trás-esquerda (CCW)

    // Clamping nos limites do PWM
    m1 = fminf(fmaxf(m1, PWM_MIN), PWM_MAX);
    m2 = fminf(fmaxf(m2, PWM_MIN), PWM_MAX);
    m3 = fminf(fmaxf(m3, PWM_MIN), PWM_MAX);
    m4 = fminf(fmaxf(m4, PWM_MIN), PWM_MAX);

    // Salvar nos ponteiros
    *motor1 = (uint8_t)m1;
    *motor2 = (uint8_t)m2;
    *motor3 = (uint8_t)m3;
    *motor4 = (uint8_t)m4;
}

void controle_principal(void *pvParameters)
{
    float pitch, roll, temp;
    mpu6050_acceleration_t accel = { 0 };
    mpu6050_rotation_t rotation = { 0 };

    while (1) {
        //ESP_ERROR_CHECK(mpu6050_get_temperature(&dev_mpu6050, &temp));
        ESP_ERROR_CHECK(mpu6050_get_motion(&dev_mpu6050, &accel, &rotation));

        pitch = atan2f(accel.x, sqrtf(accel.y * accel.y + accel.z * accel.z)); // Verificar se esta correto
        roll  = atan2f(accel.y, sqrtf(accel.x * accel.x + accel.z * accel.z));

        // accel.x, accel.y, accel.z, altura

        // Começar o controle aqui

        xQueueSend(xQueue_acel_gyro, &accel, 0); // Se trocar 0 por portMAX_DELAY fica parado aqui até ser lida a fila.
                                                 // Esta linha é somenta para dar uma amostragem dos dados do acelerometro no loop app_main
        vTaskDelay(pdMS_TO_TICKS(4)); // 250Hz
    }    
}

void mpu6050()
{
    ESP_ERROR_CHECK(mpu6050_init_desc(&dev_mpu6050, ADDR, 0, I2C_SDA, I2C_SCL));

    while (1) {
        esp_err_t res = i2c_dev_probe(&dev_mpu6050.i2c_dev, I2C_DEV_WRITE);
        if (res == ESP_OK) {
            ESP_LOGI(TAG_MPU, "Found MPU60x0 device");
            break;
        }
        ESP_LOGE(TAG_MPU, "MPU60x0 not found");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_ERROR_CHECK(mpu6050_init(&dev_mpu6050));

    mpu6050_set_dlpf_mode(&dev_mpu6050, MPU6050_DLPF_3); // Alterado para ligar o filtro, fazer testes para ver qual comportará melhor no Drone
    mpu6050_set_full_scale_accel_range(&dev_mpu6050, MPU6050_ACCEL_RANGE_8);
    mpu6050_set_full_scale_gyro_range(&dev_mpu6050, MPU6050_GYRO_RANGE_1000);

    ESP_LOGI(TAG_MPU, "Accel range: %d", dev_mpu6050.ranges.accel);
    ESP_LOGI(TAG_MPU, "Gyro range:  %d", dev_mpu6050.ranges.gyro);
}


void teste()
{
        pwm_set_duty(LEDC_CHANNEL_0, 20);
        pwm_set_duty(LEDC_CHANNEL_1, 20);
        pwm_set_duty(LEDC_CHANNEL_2, 20);
        pwm_set_duty(LEDC_CHANNEL_3, 20);  
        vTaskDelay(pdMS_TO_TICKS(1000)); 

        pwm_set_duty(LEDC_CHANNEL_0, 40);
        pwm_set_duty(LEDC_CHANNEL_1, 40);
        pwm_set_duty(LEDC_CHANNEL_2, 40);
        pwm_set_duty(LEDC_CHANNEL_3, 40); 
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        pwm_set_duty(LEDC_CHANNEL_0, 80);
        pwm_set_duty(LEDC_CHANNEL_1, 80);
        pwm_set_duty(LEDC_CHANNEL_2, 80);
        pwm_set_duty(LEDC_CHANNEL_3, 80); 
        vTaskDelay(pdMS_TO_TICKS(2000));   

        pwm_set_duty(LEDC_CHANNEL_0, 125);
        pwm_set_duty(LEDC_CHANNEL_1, 125);
        pwm_set_duty(LEDC_CHANNEL_2, 125);
        pwm_set_duty(LEDC_CHANNEL_3, 125); 
        vTaskDelay(pdMS_TO_TICKS(4000));  

        pwm_set_duty(LEDC_CHANNEL_0, 0);
        pwm_set_duty(LEDC_CHANNEL_1, 0);
        pwm_set_duty(LEDC_CHANNEL_2, 0);
        pwm_set_duty(LEDC_CHANNEL_3, 0); 
        vTaskDelay(pdMS_TO_TICKS(1000));

        pwm_set_duty(LEDC_CHANNEL_0, 60);
        pwm_set_duty(LEDC_CHANNEL_1, 0);
        pwm_set_duty(LEDC_CHANNEL_2, 0);
        pwm_set_duty(LEDC_CHANNEL_3, 0); 
        vTaskDelay(pdMS_TO_TICKS(1000));

        pwm_set_duty(LEDC_CHANNEL_0, 0);
        pwm_set_duty(LEDC_CHANNEL_1, 60);
        pwm_set_duty(LEDC_CHANNEL_2, 0);
        pwm_set_duty(LEDC_CHANNEL_3, 0); 
        vTaskDelay(pdMS_TO_TICKS(1000));        

        pwm_set_duty(LEDC_CHANNEL_0, 0);
        pwm_set_duty(LEDC_CHANNEL_1, 0);
        pwm_set_duty(LEDC_CHANNEL_2, 60);
        pwm_set_duty(LEDC_CHANNEL_3, 0); 
        vTaskDelay(pdMS_TO_TICKS(1000));    
        
        pwm_set_duty(LEDC_CHANNEL_0, 0);
        pwm_set_duty(LEDC_CHANNEL_1, 0);
        pwm_set_duty(LEDC_CHANNEL_2, 0);
        pwm_set_duty(LEDC_CHANNEL_3, 60); 
        vTaskDelay(pdMS_TO_TICKS(1000));   
}

void app_main(void)
{

    mpu6050_acceleration_t t_accel = { 0 };
    vl53l1x_t *sensor = NULL;
    uint16_t altura_anterior = 0;

    sensor = vl53l1x_config(I2C_MASTER_NUM, I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO, XSHUT_PIN, SENSOR_ADDRESS, 1);
    if (!sensor) {
        printf("Erro ao configurar o sensor\n");
        return;
    }
    
    const char *init_result = vl53l1x_init(sensor);
    if (init_result) {
        printf("Falha na inicialização: %s\n", init_result);
        return;
    }

    ledc_pwm_init();

    ESP_ERROR_CHECK(i2cdev_init());    

    xQueue_acel_gyro = xQueueCreate(1, sizeof(mpu6050_acceleration_t));    
    if (xQueue_acel_gyro == NULL) {
        ESP_LOGE("Queue acel gyro", "Falha ao criar a fila");
        return;
    }

    mpu6050();

    xTaskCreate(controle_principal, "controle_principal", configMINIMAL_STACK_SIZE * 6, NULL, 5, NULL);

    while (true) {
        xQueueReceive(xQueue_acel_gyro, &t_accel, 0);
        altura = (uint16_t) (altura_anterior + vl53l1x_readSingle(sensor, 1))/2;
        ESP_LOGI(TAG, "Accel: x=%.4f  y=%.4f  z=%.4f Dist=%d mm", t_accel.x, t_accel.y, t_accel.z, altura);
        //teste();
        vTaskDelay(pdMS_TO_TICKS(50)); 
        altura_anterior = altura;
    }  
}