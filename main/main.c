#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/adc.h"
#include "esp_log.h"

#define TAG "LIGHTBOT"

// Motor pins
#define IN1 GPIO_NUM_4  // Left PWM
#define IN2 GPIO_NUM_8  // Left DIR
#define IN3 GPIO_NUM_6  // Right PWM
#define IN4 GPIO_NUM_9  // Right DIR

// Light sensors
#define LEFT_SENSOR_ADC  ADC_CHANNEL_2  // GPIO2
#define RIGHT_SENSOR_ADC ADC_CHANNEL_3  // GPIO3

// PWM config
#define PWM_TIMER         LEDC_TIMER_0
#define PWM_FREQ_HZ       5000
#define PWM_RESOLUTION    LEDC_TIMER_8_BIT
#define PWM_LEFT_CH       LEDC_CHANNEL_0
#define PWM_RIGHT_CH      LEDC_CHANNEL_1
#define PWM_SPEED         130

// Behavior tuning
#define LIGHT_MIN_VALUE     3000
#define LIGHT_DIFF_MARGIN   500
#define LEFT_SENSOR_OFFSET   0
#define RIGHT_SENSOR_OFFSET  0

void init_pwm() {
    ledc_timer_config_t timer = {
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz = PWM_FREQ_HZ,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = PWM_TIMER
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t left = {
        .channel    = PWM_LEFT_CH,
        .duty       = 0,
        .gpio_num   = IN1,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel  = PWM_TIMER
    };
    ledc_channel_config(&left);

    ledc_channel_config_t right = {
        .channel    = PWM_RIGHT_CH,
        .duty       = 0,
        .gpio_num   = IN3,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel  = PWM_TIMER
    };
    ledc_channel_config(&right);
}

// motor control
void set_motors(uint8_t left, uint8_t right) {
    // LEFT motor
    if (left > 0) {
        gpio_set_level(IN2, 1);  // Forward
        ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_LEFT_CH, left);
    } else {
        gpio_set_level(IN2, 0);  // Brake
        ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_LEFT_CH, 0);
    }
    ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_LEFT_CH);

    // RIGHT motor
    if (right > 0) {
        gpio_set_level(IN4, 0);  // Forward
        ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_RIGHT_CH, right);
    } else {
        gpio_set_level(IN4, 0);  // Brake (ground IN4)
        ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_RIGHT_CH, 0);
    }
    ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_RIGHT_CH);
}

void stop_motors() {
    set_motors(0, 0);
}

void app_main(void) {
    // Init DIR pins
    gpio_set_direction(IN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(IN4, GPIO_MODE_OUTPUT);
    gpio_set_level(IN2, 0);
    gpio_set_level(IN4, 0);

    // Init PWM
    init_pwm();

    // Init ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(LEFT_SENSOR_ADC, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(RIGHT_SENSOR_ADC, ADC_ATTEN_DB_11);

    while (1) {
        int left = adc1_get_raw(LEFT_SENSOR_ADC) + LEFT_SENSOR_OFFSET;
        int right = adc1_get_raw(RIGHT_SENSOR_ADC) + RIGHT_SENSOR_OFFSET;
        int diff = left - right;

        ESP_LOGI(TAG, "L:%d R:%d Δ:%d", left, right, diff);

        if (left < LIGHT_MIN_VALUE && right < LIGHT_MIN_VALUE) {
            ESP_LOGI(TAG, "Too dark — STOP");
            stop_motors();
        } else if (abs(diff) < LIGHT_DIFF_MARGIN) {
            ESP_LOGI(TAG, "Go straight");
            set_motors(PWM_SPEED, PWM_SPEED);
        } else if (diff > 0) {
            ESP_LOGI(TAG, "Turn Left");
            set_motors(0, PWM_SPEED);
        } else {
            ESP_LOGI(TAG, "Turn Right");
            set_motors(PWM_SPEED, 0);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
