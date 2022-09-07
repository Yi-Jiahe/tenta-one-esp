#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_adc_cal.h"

#include "rotary_encoder.h"

static const int kEncoderSwitchPin = 25;
static const int kEncoderA = 33;
static const int kEncoderB = 32;

static const int kJoystickXADCChannel = ADC2_CHANNEL_7;
static const int kJoystickYADCChannel = ADC2_CHANNEL_9;
static const char *ADC_TAG = "ADC";

//ADC Attenuation
#define ADC_EXAMPLE_ATTEN           ADC_ATTEN_DB_11

//ADC Calibration
#if CONFIG_IDF_TARGET_ESP32
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_VREF
#elif CONFIG_IDF_TARGET_ESP32S2
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP
#elif CONFIG_IDF_TARGET_ESP32C3
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP
#elif CONFIG_IDF_TARGET_ESP32S3
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP_FIT
#endif

static int adc_raw[2][10];

static esp_adc_cal_characteristics_t adc2_chars;

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
        // ESP_LOGI(TAG, "Encoder value: %d", encoder->get_counter_value(encoder));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static bool adc_calibration_init(void)
{
    esp_err_t ret;
    bool cali_enable = false;

    ret = esp_adc_cal_check_efuse(ADC_EXAMPLE_CALI_SCHEME);
    if (ret == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "Calibration scheme not supported, skip software calibration");
    } else if (ret == ESP_ERR_INVALID_VERSION) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else if (ret == ESP_OK) {
        cali_enable = true;
        esp_adc_cal_characterize(ADC_UNIT_2, ADC_EXAMPLE_ATTEN, ADC_WIDTH_BIT_DEFAULT, 0, &adc2_chars);
    } else {
        ESP_LOGE(TAG, "Invalid arg");
    }

    return cali_enable;
}

static void joystick_read_task(void *pvParameter)
{
    esp_err_t ret = ESP_OK;
    uint32_t voltage[] = {0, 0};

    bool cali_enable = adc_calibration_init();
    ESP_ERROR_CHECK(adc2_config_channel_atten(kJoystickXADCChannel, ADC_EXAMPLE_ATTEN));
    ESP_ERROR_CHECK(adc2_config_channel_atten(kJoystickYADCChannel, ADC_EXAMPLE_ATTEN));

    while (1) {
        do {
            ret = adc2_get_raw(kJoystickXADCChannel, ADC_WIDTH_BIT_DEFAULT, &adc_raw[0][0]);
        } while (ret == ESP_ERR_INVALID_STATE);
        ESP_ERROR_CHECK(ret);
        if (cali_enable) {
            voltage[0] = esp_adc_cal_raw_to_voltage(adc_raw[0][0], &adc2_chars);
        }

        do {
            ret = adc2_get_raw(kJoystickYADCChannel, ADC_WIDTH_BIT_DEFAULT, &adc_raw[1][0]);
        } while (ret == ESP_ERR_INVALID_STATE);
        ESP_ERROR_CHECK(ret);

        if (cali_enable) {
            voltage[1] = esp_adc_cal_raw_to_voltage(adc_raw[1][0], &adc2_chars);
        }

        ESP_LOGI(ADC_TAG, "raw  data: %d\t%d", adc_raw[0][0], adc_raw[1][0]);
        // ESP_LOGI(ADC_TAG, "cali data: %d mV\t%d mV", voltage[0], voltage[1]);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void)
{
    configure_gpio();

    xTaskCreate(&encoder_switch_reading_task, "button_task", 2048, NULL, 10, NULL);
    xTaskCreate(&encoder_counter_task, "encoder_task", 2048, NULL, 10, NULL);
    xTaskCreate(&joystick_read_task, "joystick_task", 2048, NULL, 10, NULL);

}
