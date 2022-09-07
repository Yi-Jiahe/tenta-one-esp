#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "rotary_encoder.h"

static const int kEncoderSwitchPin = 25;
static const int kEncoderA = 33;
static const int kEncoderB = 32;

static const char *TAG = "main";

static void encoder_switch_reading_task(void *pvParameter)
{
    int prevEncoderState = 1;
    while (1) {
        int currEncoderState = gpio_get_level(kEncoderSwitchPin);
        if (currEncoderState != prevEncoderState) {
            ESP_LOGI(TAG, "Encoder Switch State: %d", currEncoderState);
            prevEncoderState = currEncoderState;
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

static void configure_gpio(void)
{
    ESP_LOGI(TAG, "Configuring GPIOs!");
    gpio_reset_pin(kEncoderSwitchPin);
    gpio_set_direction(kEncoderSwitchPin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(kEncoderSwitchPin, GPIO_PULLUP_ONLY);
}

static void encoder_counter_task(void *pvParameter)
{
    // Rotary encoder underlying device is represented by a PCNT unit in this example
    uint32_t pcnt_unit = 0;

    // Create rotary encoder instance
    rotary_encoder_config_t config = ROTARY_ENCODER_DEFAULT_CONFIG((rotary_encoder_dev_t)pcnt_unit, kEncoderA, kEncoderB);
    rotary_encoder_t *encoder = NULL;
    ESP_ERROR_CHECK(rotary_encoder_new_ec11(&config, &encoder));

    // Filter out glitch (1us)
    ESP_ERROR_CHECK(encoder->set_glitch_filter(encoder, 1));

    // Start encoder
    ESP_ERROR_CHECK(encoder->start(encoder));

    while (1) {
        ESP_LOGI(TAG, "Encoder value: %d", encoder->get_counter_value(encoder));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    configure_gpio();

    xTaskCreate(&encoder_switch_reading_task, "button_task", 2048, NULL, 10, NULL);
    xTaskCreate(&encoder_counter_task, "encoder_task", 2048, NULL, 10, NULL);
}
