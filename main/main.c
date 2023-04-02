#define M_PI_8 0.39269908169

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_adc_cal.h"
#include "esp_bt.h"
#include "esp_bt_defs.h"
#if CONFIG_BT_BLE_ENABLED
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#endif
#include "esp_bt_main.h"
#include "esp_bt_device.h"

#include "rotary_encoder.h"

#include "esp_hidd.h"
#include "esp_hid_gap.h"
#include "report_maps.h"
#include "hid_cc_report.h"
#include "config.h"
#include "keylayouts.h"

static const char *ADC_TAG = "ADC";

// ADC Attenuation
#define ADC_EXAMPLE_ATTEN ADC_ATTEN_DB_11

static int adc_raw[2][10];

static const char *BLE_TAG = "BLE";

QueueHandle_t keyboard_q;

typedef struct
{
    xTaskHandle task_hdl;
    esp_hidd_dev_t *hid_dev;
    uint8_t protocol_mode;
    uint8_t *buffer;
} local_param_t;

static const char *TAG = "main";

static void configure_gpio(void)
{
    ESP_LOGI(TAG, "Configuring GPIOs!");
    gpio_reset_pin(kEncoderSwitchPin);
    gpio_set_direction(kEncoderSwitchPin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(kEncoderSwitchPin, GPIO_PULLUP_ONLY);
}

static void encoder_switch_reading_task(void *pvParameter)
{
    int prevEncoderState = 1;
    while (1)
    {
        int currEncoderState = gpio_get_level(kEncoderSwitchPin);
        if (currEncoderState != prevEncoderState)
        {
            ESP_LOGI(TAG, "Encoder Switch State: %d", currEncoderState);
            prevEncoderState = currEncoderState;
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
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

    int prevEncoderValue = 0;

    while (1)
    {
        
        uint8_t key_report[HID_KEYBOARD_IN_RPT_LEN] = { 0 };
        int i = 0;

        int currEncoderValue = encoder->get_counter_value(encoder);
        do {
            if (currEncoderValue == prevEncoderValue) {
                break;
            }
            if (currEncoderValue < prevEncoderValue) {
                key_report[i] = KEY_LEFT_BRACE;
                ESP_LOGI(TAG, "Encoder down, %02x", KEY_LEFT_BRACE);

            } else if (currEncoderValue > prevEncoderValue) {
                key_report[i] = KEY_RIGHT_BRACE;
                ESP_LOGI(TAG, "Encoder up %02x", KEY_RIGHT_BRACE);
            }
            prevEncoderValue = currEncoderValue;
        } while(0);

		// xQueueSend(keyboard_q, (void*)&key_report, (TickType_t) 0);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void joystick_read_task(void *pvParameter)
{
    esp_err_t ret = ESP_OK;

    ESP_ERROR_CHECK(adc2_config_channel_atten(kJoystickXADCChannel, ADC_EXAMPLE_ATTEN));
    ESP_ERROR_CHECK(adc2_config_channel_atten(kJoystickYADCChannel, ADC_EXAMPLE_ATTEN));

    bool selecting = false;
    double selected = 0;

    while (1)
    {
        do
        {
            ret = adc2_get_raw(kJoystickXADCChannel, ADC_WIDTH_BIT_DEFAULT, &adc_raw[0][0]);
        } while (ret == ESP_ERR_INVALID_STATE);
        ESP_ERROR_CHECK(ret);
        do
        {
            ret = adc2_get_raw(kJoystickYADCChannel, ADC_WIDTH_BIT_DEFAULT, &adc_raw[1][0]);
        } while (ret == ESP_ERR_INVALID_STATE);
        ESP_ERROR_CHECK(ret);

        // TODO: Include joystick angle offset

        int x = adc_raw[0][0] - 2048;
        int y = -1 * (adc_raw[1][0] - 2048);
        int magnitude = sqrt(x * x + y * y);
        double angle = atan2((double)y, (double)x);

        if (magnitude > 1024 * kJoystickDeadZone) {
            selecting = true;
            selected = angle;
        } else {
            if (selecting) {
                ESP_LOGI(ADC_TAG, "Selected Angle: %lf", selected);
                if (selected < -7 * M_PI_8 || selected > 7 * M_PI_8) {
                    ESP_LOGI(ADC_TAG, "Selected section: %d", 7);
                } else if (selected < -5 * M_PI_8) {
                    ESP_LOGI(ADC_TAG, "Selected section: %d", 6);
                } else if (selected < -3 * M_PI_8) {
                    ESP_LOGI(ADC_TAG, "Selected section: %d", 5);
                } else if (selected < -1 * M_PI_8) {
                    ESP_LOGI(ADC_TAG, "Selected section: %d", 4);
                } else if (selected < 1 * M_PI_8) {
                    ESP_LOGI(ADC_TAG, "Selected section: %d", 3);
                } else if (selected < 3 * M_PI_8) {
                    ESP_LOGI(ADC_TAG, "Selected section: %d", 2);
                } else if (selected < 5 * M_PI_8) {
                    ESP_LOGI(ADC_TAG, "Selected section: %d", 1);
                } else {
                    ESP_LOGI(ADC_TAG, "Selected section: %d", 8);
                }

                selecting = false;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

#if CONFIG_BT_BLE_ENABLED
static local_param_t s_ble_hid_param = {0};

static esp_hid_raw_report_map_t ble_report_maps[] = {
    {.data = hidapiReportMap,
     .len = sizeof(hidapiReportMap)},
    {.data = mediaReportMap,
     .len = sizeof(mediaReportMap)},
    {.data = keyboardReportMap,
     .len = sizeof(keyboardReportMap)}};

static esp_hid_device_config_t ble_hid_config = {
    .vendor_id = 0x16C0,
    .product_id = 0x05DF,
    .version = 0x0100,
    .device_name = "Tenta-one BLE",
    .manufacturer_name = "Espressif",
    .serial_number = "1234567890",
    .report_maps = ble_report_maps,
    .report_maps_len = 2};

void esp_hidd_send_consumer_value(uint8_t key_cmd, bool key_pressed)
{
    uint8_t buffer[HID_CC_IN_RPT_LEN] = {0, 0};
    esp_hidd_consumer_value_buffer(buffer, key_cmd, key_pressed);
    esp_hidd_dev_input_set(s_ble_hid_param.hid_dev, 1, HID_RPT_ID_CC_IN, buffer, HID_CC_IN_RPT_LEN);
    return;
}

void ble_hid_consume_keyboard_queue_task(void *pvParameters)
{
	keyboard_q = xQueueCreate(32, HID_KEYBOARD_IN_RPT_LEN * sizeof(uint8_t));

    if (keyboard_q != NULL)
    {
        // Empty queue if initialized (there might be something left from last connection)
        xQueueReset(keyboard_q);

        while (1)
        {
            uint8_t key_report[HID_KEYBOARD_IN_RPT_LEN] = { 0 };

            // pend on MQ, if timeout triggers, just wait again.
            if (xQueueReceive(keyboard_q, key_report, portMAX_DELAY))
            {

                esp_hidd_dev_input_set(s_ble_hid_param.hid_dev, 1,
                                    HID_RPT_ID_KEY_IN, key_report, HID_KEYBOARD_IN_RPT_LEN);
            }
        }
    }
    else
    {
        ESP_LOGE(BLE_TAG, "ble hid queue not initialized, retry in 1s");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void ble_hid_task_start_up(void)
{
    xTaskCreate(ble_hid_consume_keyboard_queue_task, "ble_hid_consume_keyboard_queue_task", 2 * 2048, NULL, configMAX_PRIORITIES - 3,
                &s_ble_hid_param.task_hdl);
}

void ble_hid_task_shut_down(void)
{
    if (s_ble_hid_param.task_hdl)
    {
        vTaskDelete(s_ble_hid_param.task_hdl);
        s_ble_hid_param.task_hdl = NULL;
    }
}

static void ble_hidd_event_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidd_event_t event = (esp_hidd_event_t)id;
    esp_hidd_event_data_t *param = (esp_hidd_event_data_t *)event_data;
    static const char *TAG = "HID_DEV_BLE";

    switch (event)
    {
    case ESP_HIDD_START_EVENT:
    {
        ESP_LOGI(TAG, "START");
        esp_hid_ble_gap_adv_start();
        break;
    }
    case ESP_HIDD_CONNECT_EVENT:
    {
        ESP_LOGI(TAG, "CONNECT");
        ble_hid_task_start_up(); // todo: this should be on auth_complete (in GAP)
        break;
    }
    case ESP_HIDD_PROTOCOL_MODE_EVENT:
    {
        ESP_LOGI(TAG, "PROTOCOL MODE[%u]: %s", param->protocol_mode.map_index, param->protocol_mode.protocol_mode ? "REPORT" : "BOOT");
        break;
    }
    case ESP_HIDD_CONTROL_EVENT:
    {
        ESP_LOGI(TAG, "CONTROL[%u]: %sSUSPEND", param->control.map_index, param->control.control ? "EXIT_" : "");
        break;
    }
    case ESP_HIDD_OUTPUT_EVENT:
    {
        ESP_LOGI(TAG, "OUTPUT[%u]: %8s ID: %2u, Len: %d, Data:", param->output.map_index, esp_hid_usage_str(param->output.usage), param->output.report_id, param->output.length);
        ESP_LOG_BUFFER_HEX(TAG, param->output.data, param->output.length);
        break;
    }
    case ESP_HIDD_FEATURE_EVENT:
    {
        ESP_LOGI(TAG, "FEATURE[%u]: %8s ID: %2u, Len: %d, Data:", param->feature.map_index, esp_hid_usage_str(param->feature.usage), param->feature.report_id, param->feature.length);
        ESP_LOG_BUFFER_HEX(TAG, param->feature.data, param->feature.length);
        break;
    }
    case ESP_HIDD_DISCONNECT_EVENT:
    {
        ESP_LOGI(TAG, "DISCONNECT: %s", esp_hid_disconnect_reason_str(esp_hidd_dev_transport_get(param->disconnect.dev), param->disconnect.reason));
        ble_hid_task_shut_down();
        esp_hid_ble_gap_adv_start();
        break;
    }
    case ESP_HIDD_STOP_EVENT:
    {
        ESP_LOGI(TAG, "STOP");
        break;
    }
    default:
        break;
    }
    return;
}
#endif

#if CONFIG_BT_HID_DEVICE_ENABLED
static local_param_t s_bt_hid_param = {0};

static esp_hid_raw_report_map_t bt_report_maps[] = {
    {.data = mouseReportMap,
     .len = sizeof(mouseReportMap)},
};

static esp_hid_device_config_t bt_hid_config = {
    .vendor_id = 0x16C0,
    .product_id = 0x05DF,
    .version = 0x0100,
    .device_name = "Tenta-one BT Classic",
    .manufacturer_name = "Espressif",
    .serial_number = "1234567890",
    .report_maps = bt_report_maps,
    .report_maps_len = 1};

// send the buttons, change in x, and change in y
void send_mouse(uint8_t buttons, char dx, char dy, char wheel)
{
    static uint8_t buffer[4] = {0};
    buffer[0] = buttons;
    buffer[1] = dx;
    buffer[2] = dy;
    buffer[3] = wheel;
    esp_hidd_dev_input_set(s_bt_hid_param.hid_dev, 0, 0, buffer, 4);
}

void bt_hid_demo_task(void *pvParameters)
{
    static const char *help_string = "########################################################################\n"
                                     "BT hid mouse demo usage:\n"
                                     "You can input these value to simulate mouse: 'q', 'w', 'e', 'a', 's', 'd', 'h'\n"
                                     "q -- click the left key\n"
                                     "w -- move up\n"
                                     "e -- click the right key\n"
                                     "a -- move left\n"
                                     "s -- move down\n"
                                     "d -- move right\n"
                                     "h -- show the help\n"
                                     "########################################################################\n";
    printf("%s\n", help_string);
    char c;
    while (1)
    {
        c = fgetc(stdin);
        switch (c)
        {
        case 'q':
            send_mouse(1, 0, 0, 0);
            break;
        case 'w':
            send_mouse(0, 0, -10, 0);
            break;
        case 'e':
            send_mouse(2, 0, 0, 0);
            break;
        case 'a':
            send_mouse(0, -10, 0, 0);
            break;
        case 's':
            send_mouse(0, 0, 10, 0);
            break;
        case 'd':
            send_mouse(0, 10, 0, 0);
            break;
        case 'h':
            printf("%s\n", help_string);
            break;
        default:
            break;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void bt_hid_task_start_up(void)
{
    xTaskCreate(bt_hid_demo_task, "bt_hid_demo_task", 2 * 1024, NULL, configMAX_PRIORITIES - 3, &s_bt_hid_param.task_hdl);
    return;
}

void bt_hid_task_shut_down(void)
{
    if (s_bt_hid_param.task_hdl)
    {
        vTaskDelete(s_bt_hid_param.task_hdl);
        s_bt_hid_param.task_hdl = NULL;
    }
}

static void bt_hidd_event_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidd_event_t event = (esp_hidd_event_t)id;
    esp_hidd_event_data_t *param = (esp_hidd_event_data_t *)event_data;
    static const char *TAG = "HID_DEV_BT";

    switch (event)
    {
    case ESP_HIDD_START_EVENT:
    {
        if (param->start.status == ESP_OK)
        {
            ESP_LOGI(TAG, "START OK");
            ESP_LOGI(TAG, "Setting to connectable, discoverable");
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        }
        else
        {
            ESP_LOGE(TAG, "START failed!");
        }
        break;
    }
    case ESP_HIDD_CONNECT_EVENT:
    {
        if (param->connect.status == ESP_OK)
        {
            ESP_LOGI(TAG, "CONNECT OK");
            ESP_LOGI(TAG, "Setting to non-connectable, non-discoverable");
            esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
            bt_hid_task_start_up();
        }
        else
        {
            ESP_LOGE(TAG, "CONNECT failed!");
        }
        break;
    }
    case ESP_HIDD_PROTOCOL_MODE_EVENT:
    {
        ESP_LOGI(TAG, "PROTOCOL MODE[%u]: %s", param->protocol_mode.map_index, param->protocol_mode.protocol_mode ? "REPORT" : "BOOT");
        break;
    }
    case ESP_HIDD_OUTPUT_EVENT:
    {
        ESP_LOGI(TAG, "OUTPUT[%u]: %8s ID: %2u, Len: %d, Data:", param->output.map_index, esp_hid_usage_str(param->output.usage), param->output.report_id, param->output.length);
        ESP_LOG_BUFFER_HEX(TAG, param->output.data, param->output.length);
        break;
    }
    case ESP_HIDD_FEATURE_EVENT:
    {
        ESP_LOGI(TAG, "FEATURE[%u]: %8s ID: %2u, Len: %d, Data:", param->feature.map_index, esp_hid_usage_str(param->feature.usage), param->feature.report_id, param->feature.length);
        ESP_LOG_BUFFER_HEX(TAG, param->feature.data, param->feature.length);
        break;
    }
    case ESP_HIDD_DISCONNECT_EVENT:
    {
        if (param->disconnect.status == ESP_OK)
        {
            ESP_LOGI(TAG, "DISCONNECT OK");
            bt_hid_task_shut_down();
            ESP_LOGI(TAG, "Setting to connectable, discoverable again");
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        }
        else
        {
            ESP_LOGE(TAG, "DISCONNECT failed!");
        }
        break;
    }
    case ESP_HIDD_STOP_EVENT:
    {
        ESP_LOGI(TAG, "STOP");
        break;
    }
    default:
        break;
    }
    return;
}
#endif

void app_main(void)
{
    esp_err_t ret;

    configure_gpio();

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "setting hid gap, mode:%d", HID_DEV_MODE);
    ret = esp_hid_gap_init(HID_DEV_MODE);
    ESP_ERROR_CHECK(ret);

#if CONFIG_BT_BLE_ENABLED
    ret = esp_hid_ble_gap_adv_init(ESP_HID_APPEARANCE_GENERIC, ble_hid_config.device_name);
    ESP_ERROR_CHECK(ret);

    if ((ret = esp_ble_gatts_register_callback(esp_hidd_gatts_event_handler)) != ESP_OK)
    {
        ESP_LOGE(TAG, "GATTS register callback failed: %d", ret);
        return;
    }
    ESP_LOGI(TAG, "setting ble device");
    ESP_ERROR_CHECK(
        esp_hidd_dev_init(&ble_hid_config, ESP_HID_TRANSPORT_BLE, ble_hidd_event_callback, &s_ble_hid_param.hid_dev));
#endif

#if CONFIG_BT_HID_DEVICE_ENABLED
    ESP_LOGI(TAG, "setting device name");
    esp_bt_dev_set_device_name(bt_hid_config.device_name);
    ESP_LOGI(TAG, "setting cod major, peripheral");
    esp_bt_cod_t cod;
    cod.major = ESP_BT_COD_MAJOR_DEV_PERIPHERAL;
    esp_bt_gap_set_cod(cod, ESP_BT_SET_COD_MAJOR_MINOR);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "setting bt device");
    ESP_ERROR_CHECK(
        esp_hidd_dev_init(&bt_hid_config, ESP_HID_TRANSPORT_BT, bt_hidd_event_callback, &s_bt_hid_param.hid_dev));
#endif

    xTaskCreate(&encoder_switch_reading_task, "button_task", 2048, NULL, 10, NULL);
    xTaskCreate(&encoder_counter_task, "encoder_task", 2048, NULL, 10, NULL);
    xTaskCreate(&joystick_read_task, "joystick_task", 2048, NULL, 10, NULL);
}
