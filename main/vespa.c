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
#define I2C_FREQ_HZ (1000000) // Padrão da biblioteca esp-idf-lib é 1000000 1MHz
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


typedef struct {
    float Kp, Ki, Kd;
    float integral, previous_error;
    float integral_max, integral_min;
} PID_t;

float roll, pitch, yaw;
uint16_t altitude; // Altitude medida
uint8_t throttle_base = 10.0; // Throttle base (0-255)
PID_t pid_altitude = {2.0, 0.01, 0.1, 0.0, 0.0, 240.0, 0.0}; // Kp, Ki, Kd, integral, previous_error, integral_max e integral_min
PID_t pid_roll = {2.5, 0.01, 0.05, 0.0, 0.0, 240.0, 0.0};
PID_t pid_pitch = {2.5, 0.01, 0.05, 0.0, 0.0, 240.0, 0.0};
PID_t pid_yaw = {2.0, 0.0, 0.02, 0.0, 0.0, 240.0, 0.0};



float pid_compute(PID_t *pid, float setpoint, float measured, float dt) {
    float error = setpoint - measured;
    
    pid->integral += error * dt;
    
    if (pid->integral > pid->integral_max) {
        pid->integral = pid->integral_max;
    } else if (pid->integral < pid->integral_min) {
        pid->integral = pid->integral_min;
    }
    
    float derivative = (error - pid->previous_error) / dt;
    pid->previous_error = error;
    
    return pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
}

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

void controle_principal(void *pvParameters)
{
    float temp;
    mpu6050_acceleration_t accel = { 0 };
    mpu6050_rotation_t rotation = { 0 };
    float ax, ay, az, gx, gy, gz;
    float dt = 0.010; // 10ms
    //float dt = 0.1; // 100ms    
    float alpha = 0.98;
    float setpoint_altitude = 180.0; // 180mm
    float setpoint_roll = 0.0, setpoint_pitch = 0.0, setpoint_yaw = 0.0;
    float dif_ax = 0.0, dif_ay = 0.0, dif_az = 0.0; 
    float dif_gx = 0.0, dif_gy = 0.0, dif_gz = 0.0; 

    // Calibrar acelerometro
    vTaskDelay(pdMS_TO_TICKS(100));
    for (uint8_t i = 0; i < 100; i++) {
        ESP_ERROR_CHECK(mpu6050_get_motion(&dev_mpu6050, &accel, &rotation));

        dif_ax += accel.x;
        dif_ay += accel.y;
        dif_az += accel.z;   

        dif_gx += rotation.x;
        dif_gy += rotation.y;
        dif_gz += rotation.z;                   
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    dif_ax /= 100.0f;
    dif_ay /= 100.0f;
    dif_az /= 100.0f;

    dif_az -= 1.0f; // Subtraí o valor de uma gravidade a grosso modo

    dif_gx /= 100.0f;
    dif_gy /= 100.0f;
    dif_gz /= 100.0f;    

    ESP_LOGI(TAG, "Accel: x=%.4f  y=%.4f  z=%.4f Giro: x=%.4f  y=%.4f  z=%.4f", dif_ax, dif_ay, dif_az, dif_gx, dif_gy, dif_gz);

    while (1) {
        //ESP_ERROR_CHECK(mpu6050_get_temperature(&dev_mpu6050, &temp));  // Temperatura
        ESP_ERROR_CHECK(mpu6050_get_motion(&dev_mpu6050, &accel, &rotation));

        ax = accel.x - dif_ax;
        ay = accel.y - dif_ay;
        az = accel.z - dif_az;
        gx = rotation.x - dif_gx;
        gy = rotation.y - dif_gy;
        gz = rotation.z - dif_gz;

        float roll_acc = atan2(ay, sqrt(ax * ax + az * az)) * 180 / M_PI;
        float pitch_acc = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / M_PI;
        roll = alpha * (roll + gx * dt) + (1 - alpha) * roll_acc;
        pitch = alpha * (pitch + gy * dt) + (1 - alpha) * pitch_acc;
        yaw = yaw + gz * dt; 

        float throttle = pid_compute(&pid_altitude, setpoint_altitude, altitude, dt);
        float roll_output = pid_compute(&pid_roll, setpoint_roll, roll, dt);
        float pitch_output = pid_compute(&pid_pitch, setpoint_pitch, pitch, dt);
        float yaw_output = pid_compute(&pid_yaw, setpoint_yaw, yaw, dt);

        float pwm_m1 = throttle_base + throttle + roll_output - pitch_output - yaw_output; // + pitch_output | Inverti os sinal do pitch em função
        float pwm_m2 = throttle_base + throttle - roll_output - pitch_output + yaw_output; // + pitch_output | da posição como coloquei no drone
        float pwm_m3 = throttle_base + throttle - roll_output + pitch_output - yaw_output; // - pitch_output
        float pwm_m4 = throttle_base + throttle + roll_output + pitch_output + yaw_output; // - pitch_output

        pwm_m1 = fmax(0, fmin(255, pwm_m1));
        pwm_m2 = fmax(0, fmin(255, pwm_m2));
        pwm_m3 = fmax(0, fmin(255, pwm_m3));
        pwm_m4 = fmax(0, fmin(255, pwm_m4));        
        
        //ESP_LOGI(TAG, "Accel: x=%.4f  y=%.4f  z=%.4f  Giro: x=%.4f  y=%.4f  z=%.4f Dist=%d mm", ax, ay, az, gx, gy, gz, altitude);
        //printf(">roll:%.4f\n>pitch:%.4f\n>yaw:%.4f\n>pidroll:%.4f\n>pidpitch:%.4f\n>pidyaw:%.4f\n", roll, pitch, yaw, roll_output, pitch_output, yaw_output);
        
        pwm_set_duty(LEDC_CHANNEL_0, pwm_m1);
        pwm_set_duty(LEDC_CHANNEL_1, pwm_m2);
        pwm_set_duty(LEDC_CHANNEL_2, pwm_m3);
        pwm_set_duty(LEDC_CHANNEL_3, pwm_m4);        

        xQueueSend(xQueue_acel_gyro, &accel, 0); // Se trocar 0 por portMAX_DELAY fica parado aqui até ser lida a fila.
                                                 // Esta linha é somenta para dar uma amostragem dos dados do acelerometro no loop app_main
        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz
        //vTaskDelay(pdMS_TO_TICKS(100)); // 10Hz        
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

    mpu6050_set_dlpf_mode(&dev_mpu6050, MPU6050_DLPF_3); // Ainda fazendo testes para ver qual comportará melhor no Drone
    mpu6050_set_full_scale_accel_range(&dev_mpu6050, MPU6050_ACCEL_RANGE_4); // Se saturar, alterar para +-8g
    mpu6050_set_full_scale_gyro_range(&dev_mpu6050, MPU6050_GYRO_RANGE_2000); 

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
    uint16_t altitude_anterior = 0;

    vTaskDelay(pdMS_TO_TICKS(100)); 
    
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
        altitude = (uint16_t) (altitude_anterior + vl53l1x_readSingle(sensor, 1))/2;
        ESP_LOGI(TAG, "Accel: x=%.4f  y=%.4f  z=%.4f Dist=%d mm", t_accel.x, t_accel.y, t_accel.z, altitude);
        //teste();
        vTaskDelay(pdMS_TO_TICKS(50)); 
        altitude_anterior = altitude;
    }  
}