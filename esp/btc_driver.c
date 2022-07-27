#include <esp_bt.h>
#include <esp_log.h>
#include <esp_bt_main.h>
#include <esp_gap_bt_api.h>
#include <esp_spp_api.h>
#include <esp_bt_device.h>
#include <freertos/task.h>
#include <nvs_flash.h>
#include "btc_driver.h"

#define BT_TAG "Bluetooth"
#define BT_NAME "SPSU"

static uint32_t sppClient = 0;
void (*onReceiveCallback)(esp_spp_cb_param_t*) = NULL;
void (*onConnectCallback)(void) = NULL;

void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    ESP_LOGI("BT_GAP","Event Occurred: %d",event);
}

void esp_bt_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event)
    {
        case ESP_SPP_SRV_OPEN_EVT:
            if (param->srv_open.status != ESP_SPP_SUCCESS)
                break;
            sppClient = param->srv_open.handle;

            ESP_LOGI("BT_EVT","Connection opened on handle:%d",sppClient);

            if (onConnectCallback != NULL)
                onConnectCallback();
            break;

        case ESP_SPP_SRV_STOP_EVT:
            if (param->close.status == ESP_SPP_SUCCESS)
                sppClient = 0;
            break;

        case ESP_SPP_DATA_IND_EVT:
            if (onReceiveCallback)
                onReceiveCallback(param);
            break;
        default:
            break;
    }
}


void config_bluetooth(void (*callback)(esp_spp_cb_param_t*),void (*conCallback)(void))
{
    nvs_flash_init();

    onReceiveCallback = callback;
    onConnectCallback = conCallback;

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_err_t retCode;

    esp_bt_controller_config_t bluetoothConfig = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    if ((retCode = esp_bt_controller_init(&bluetoothConfig)) != ESP_OK)
    {
        ESP_LOGE(BT_TAG, "An error occurred while initializing bluetooth module: %s\n",  esp_err_to_name(retCode));
        return;
    }

    if ((retCode = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK)
    {
        ESP_LOGE(BT_TAG, "An error occurred while enabling the bluetooth module: %s\n",  esp_err_to_name(retCode));
        return;
    }

    if ((retCode = esp_bluedroid_init()) != ESP_OK)
    {
        ESP_LOGE(BT_TAG, "An error occurred while initializing Bluedroid: %s\n",  esp_err_to_name(retCode));
        return;
    }

    if ((retCode = esp_bluedroid_enable()) != ESP_OK)
    {
        ESP_LOGE(BT_TAG, "An error occurred while enabling Bluedroid: %s\n",  esp_err_to_name(retCode));
        return;
    }

    if ((retCode = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK)
    {
        ESP_LOGE(BT_TAG, "An error occurred while registering the GAP Service callback: %s\n", esp_err_to_name(retCode));
        return;
    }

    if ((retCode = esp_spp_register_callback(esp_bt_spp_cb)) != ESP_OK)
    {
        ESP_LOGE(BT_TAG,"SPP Callback register failed");
        return;
    }

    if ((retCode = esp_spp_init(ESP_SPP_MODE_CB)) != ESP_OK)
    {
        ESP_LOGE(BT_TAG,"SPP Init failed");
        return;
    }

    esp_spp_start_srv(ESP_SPP_SEC_NONE,ESP_SPP_ROLE_SLAVE,0,"SPSU");

    ESP_LOGI(BT_TAG, "Defining bluetooth device name to: %s",BT_NAME);

    if ((retCode = esp_bt_dev_set_device_name(BT_NAME)) != ESP_OK)
    {
        ESP_LOGE(BT_TAG, "An error occurred while defining the bluetooth device name: %s\n", esp_err_to_name(retCode));
        return;
    }

    ESP_LOGI(BT_TAG, "Enabling COD Major peripheral");

    esp_bt_cod_t cod;
    cod.major = ESP_BT_COD_MAJOR_DEV_MISC;

    if ((retCode = esp_bt_gap_set_cod(cod ,ESP_BT_SET_COD_MAJOR_MINOR)) != ESP_OK)
    {
        ESP_LOGE(BT_TAG, "An error occurred while enabling COD Major peripheral: %s\n", esp_err_to_name(retCode));
        return;
    }

    vTaskDelay(2000 / portTICK_PERIOD_MS);

    if ((retCode = esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE,ESP_BT_GENERAL_DISCOVERABLE)) != ESP_OK)
    {
        ESP_LOGE(BT_TAG, "An error occurred while enabling Bluetooth discover mode: %s\n", esp_err_to_name(retCode));
        return;
    }
}

bool send_data(int dataLen,uint8_t* data)
{
    if (!sppClient)
        return false;

    if (esp_spp_write(sppClient,dataLen,data) == ESP_OK)
        return true;

    return false;
}