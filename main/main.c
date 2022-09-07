#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const int kEncoderSwitchPin = 25;
// static const int kEncoderA = 33;
// static const int kEncoderB = 32;

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

void app_main(void)
{
    configure_gpio();

    xTaskCreate(&encoder_switch_reading_task, "button_task", 2048, NULL, 10, NULL);
}
