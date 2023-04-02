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

#include "esp_hidd_prf_api.h"
#include "esp_hidd.h"
#include "hid_dev.h"

#include "rotary_encoder.h"

#include "config.h"

static const char *ADC_TAG = "ADC";

// ADC Attenuation
#define ADC_EXAMPLE_ATTEN ADC_ATTEN_DB_11

static int adc_raw[2][10];

static const char *BLE_TAG = "BLE";

static uint16_t hid_conn_id = 0;
static bool sec_conn = false;

#define HIDD_DEVICE_NAME            "TENTA_ONE"
static uint8_t hidd_service_uuid128[] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x00,
};

static esp_ble_adv_data_t hidd_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x03c0,       //HID Generic,
    .manufacturer_len = 0,
    .p_manufacturer_data =  NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(hidd_service_uuid128),
    .p_service_uuid = hidd_service_uuid128,
    .flag = 0x6,
};

static esp_ble_adv_params_t hidd_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x30,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    switch(event) {
        case ESP_HIDD_EVENT_REG_FINISH: {
            if (param->init_finish.state == ESP_HIDD_INIT_OK) {
                //esp_bd_addr_t rand_addr = {0x04,0x11,0x11,0x11,0x11,0x05};
                esp_ble_gap_set_device_name(HIDD_DEVICE_NAME);
                esp_ble_gap_config_adv_data(&hidd_adv_data);

            }
            break;
        }
        case ESP_BAT_EVENT_REG: {
            break;
        }
        case ESP_HIDD_EVENT_DEINIT_FINISH:
	     break;
		case ESP_HIDD_EVENT_BLE_CONNECT: {
            ESP_LOGI(BLE_TAG, "ESP_HIDD_EVENT_BLE_CONNECT");
            hid_conn_id = param->connect.conn_id;
            break;
        }
        case ESP_HIDD_EVENT_BLE_DISCONNECT: {
            sec_conn = false;
            ESP_LOGI(BLE_TAG, "ESP_HIDD_EVENT_BLE_DISCONNECT");
            esp_ble_gap_start_advertising(&hidd_adv_params);
            break;
        }
        case ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT: {
            ESP_LOGI(BLE_TAG, "%s, ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT", __func__);
            ESP_LOG_BUFFER_HEX(BLE_TAG, param->vendor_write.data, param->vendor_write.length);
            break;
        }
        case ESP_HIDD_EVENT_BLE_LED_REPORT_WRITE_EVT: {
            ESP_LOGI(BLE_TAG, "ESP_HIDD_EVENT_BLE_LED_REPORT_WRITE_EVT");
            ESP_LOG_BUFFER_HEX(BLE_TAG, param->led_write.data, param->led_write.length);
            break;
        }
        default:
            break;
    }
    return;
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&hidd_adv_params);
        break;
     case ESP_GAP_BLE_SEC_REQ_EVT:
        for(int i = 0; i < ESP_BD_ADDR_LEN; i++) {
             ESP_LOGD(BLE_TAG, "%x:",param->ble_security.ble_req.bd_addr[i]);
        }
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
	 break;
     case ESP_GAP_BLE_AUTH_CMPL_EVT:
        sec_conn = true;
        esp_bd_addr_t bd_addr;
        memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
        ESP_LOGI(BLE_TAG, "remote BD_ADDR: %08x%04x",\
                (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3],
                (bd_addr[4] << 8) + bd_addr[5]);
        ESP_LOGI(BLE_TAG, "address type = %d", param->ble_security.auth_cmpl.addr_type);
        ESP_LOGI(BLE_TAG, "pair status = %s",param->ble_security.auth_cmpl.success ? "success" : "fail");
        if(!param->ble_security.auth_cmpl.success) {
            ESP_LOGE(BLE_TAG, "fail reason = 0x%x",param->ble_security.auth_cmpl.fail_reason);
        }
        break;
    default:
        break;
    }
}

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
        int currEncoderValue = encoder->get_counter_value(encoder);
        do {
            if (currEncoderValue == prevEncoderValue) {
                break;
            }
            if (currEncoderValue < prevEncoderValue) {
                ESP_LOGI(TAG, "Encoder down");
                uint8_t keys = {HID_KEY_LEFT_BRKT};
                esp_hidd_send_keyboard_value(hid_conn_id, 0, &keys, 1);
                esp_hidd_send_keyboard_value(hid_conn_id, 0, NULL, 0);
            } else if (currEncoderValue > prevEncoderValue) {
                ESP_LOGI(TAG, "Encoder up");
                uint8_t keys = {HID_KEY_RIGHT_BRKT};
                esp_hidd_send_keyboard_value(hid_conn_id, 0, &keys, 1);
                esp_hidd_send_keyboard_value(hid_conn_id, 0, NULL, 0);
            }
            prevEncoderValue = currEncoderValue;
        } while(0);

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
                uint8_t keys[1];

                if (selected < -7 * M_PI_8 || selected > 7 * M_PI_8) {
                    ESP_LOGI(ADC_TAG, "Selected section: %d", 7);
                    keys[0] = HID_KEY_7;
                } else if (selected < -5 * M_PI_8) {
                    ESP_LOGI(ADC_TAG, "Selected section: %d", 6);
                    keys[0] = HID_KEY_6;
                } else if (selected < -3 * M_PI_8) {
                    ESP_LOGI(ADC_TAG, "Selected section: %d", 5);
                    keys[0] = HID_KEY_5;
                } else if (selected < -1 * M_PI_8) {
                    ESP_LOGI(ADC_TAG, "Selected section: %d", 4);
                    keys[0] = HID_KEY_4;
                } else if (selected < 1 * M_PI_8) {
                    ESP_LOGI(ADC_TAG, "Selected section: %d", 3);
                    keys[0] = HID_KEY_3;
                } else if (selected < 3 * M_PI_8) {
                    ESP_LOGI(ADC_TAG, "Selected section: %d", 2);
                    keys[0] = HID_KEY_2;
                } else if (selected < 5 * M_PI_8) {
                    ESP_LOGI(ADC_TAG, "Selected section: %d", 1);
                    keys[0] = HID_KEY_1;
                } else {
                    ESP_LOGI(ADC_TAG, "Selected section: %d", 8);
                    keys[0] = HID_KEY_8;
                }

                esp_hidd_send_keyboard_value(hid_conn_id, 0, &keys, 1);
                esp_hidd_send_keyboard_value(hid_conn_id, 0, NULL, 0);

                selecting = false;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

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

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(BLE_TAG, "%s initialize controller failed\n", __func__);
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(BLE_TAG, "%s enable controller failed\n", __func__);
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(BLE_TAG, "%s init bluedroid failed\n", __func__);
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(BLE_TAG, "%s init bluedroid failed\n", __func__);
        return;
    }

    if((ret = esp_hidd_profile_init()) != ESP_OK) {
        ESP_LOGE(BLE_TAG, "%s init bluedroid failed\n", __func__);
    }

    ///register the callback function to the gap module
    esp_ble_gap_register_callback(gap_event_handler);
    esp_hidd_register_callbacks(hidd_event_callback);

    /* set the security iocap & auth_req & key size & init key response key parameters to the stack*/
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;     //bonding with peer device after authentication
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;           //set the IO capability to No output No input
    uint8_t key_size = 16;      //the key size should be 7~16 bytes
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    /* If your BLE device act as a Slave, the init_key means you hope which types of key of the master should distribute to you,
    and the response key means which key you can distribute to the Master;
    If your BLE device act as a master, the response key means you hope which types of key of the slave should distribute to you,
    and the init key means which key you can distribute to the slave. */
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

    xTaskCreate(&encoder_switch_reading_task, "button_task", 2048, NULL, 10, NULL);
    xTaskCreate(&encoder_counter_task, "encoder_task", 2048, NULL, 10, NULL);
    xTaskCreate(&joystick_read_task, "joystick_task", 2048, NULL, 10, NULL);
}
