/************************************************************************************
 * @name: PWMonitor                                                                *
 * @brief: ESP-32 DevKit4 Variable Duty Cycle PWM Generator with Bluetooth Control *
 * @version V0.1                                                                   *
 * @author: André Costa - https://github.com/Ner027                                *
 * @date: 2022                                                                     *
 * University Of Minho - Guimarães                                                 *
 ************************************************************************************/

#include <sys/cdefs.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>
#include <encoder.h>
#include <esp_log.h>
#include <hal/i2c_types.h>
#include <driver/i2c.h>
#include <esp_spp_api.h>
#include "ssd1306_driver.h"
#include "pwm_driver.h"
#include "btc_driver.h"


//Pin Definitions
#define SDA_PIN 19
#define SCL_PIN 18
#define RE_A_GPIO   23
#define RE_B_GPIO   22
#define RE_BTN_GPIO 21
#define EV_QUEUE_LEN 5

//Constants
#define DC_MIN 30
#define DC_MAX 70
#define FR_MIN 5000
#define FR_MAX 15000
#define FR_STEP 100
#define ENC_TAG "ENC"

static QueueHandle_t event_queue;
static rotary_encoder_t rotaryEncoder;
static uint8_t currentDuty = 70;
static uint16_t currentFrequency = 7500;
static bool dutySelected = true;

static void update_duty(uint8_t newDuty);
static void update_frequency(uint16_t newFrequency);
static void update_values();
static void bt_rec_callback(esp_spp_cb_param_t* params);
static void update_out(bool isButton);
static void update_bt_data();
static void bt_connect_callback();

/// \name config_iwc
/// \brief Initializes I2C protocol in master mode
void config_i2c()
{
    i2c_config_t i2c_config =
            {
                    .mode = I2C_MODE_MASTER,
                    .sda_io_num = SDA_PIN,
                    .scl_io_num = SCL_PIN,
                    .sda_pullup_en = GPIO_PULLUP_ENABLE,
                    .scl_pullup_en = GPIO_PULLUP_ENABLE,
                    .master.clk_speed = 1000000
            };
    i2c_param_config(I2C_NUM_0, &i2c_config);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}


/// \name vTask_gpioListener
/// \brief Task that handles everything related to the rotary encoder input
/// \param arg Ignored
_Noreturn void vTask_gpioListener(void *arg)
{
    //Create a FreeRTOS queue to receive the events from the encoder
    event_queue = xQueueCreate(EV_QUEUE_LEN, sizeof(rotary_encoder_event_t));

    //Try and init the encoder while checking for errors
    ESP_ERROR_CHECK(rotary_encoder_init(event_queue));

    //Initialize the config struct
    memset(&rotaryEncoder, 0, sizeof(rotary_encoder_t));
    rotaryEncoder.pin_a = RE_A_GPIO;
    rotaryEncoder.pin_b = RE_B_GPIO;
    rotaryEncoder.pin_btn = RE_BTN_GPIO;

    //Try and add a new encoder while checking for errors
    ESP_ERROR_CHECK(rotary_encoder_add(&rotaryEncoder));

    rotary_encoder_event_t e;

    //Keep querying the queue for new events
    while (1)
    {
        //Wait indefinitely until something is received
        xQueueReceive(event_queue, &e, portMAX_DELAY);
        //When an event occurs check its type
        switch (e.type)
        {
            //If the button was clicked change the selection from duty to frequency or vice-versa
            case RE_ET_BTN_CLICKED:
                dutySelected = !dutySelected;
                break;
            //When the encoder value changes depending on what is selected update either duty or frequency
            // NOTE:Frequency is updated in steps defined by the FR_STEP macro
            case RE_ET_CHANGED:
                if (dutySelected)
                    update_duty(currentDuty - e.diff);
                else update_frequency(currentFrequency - (e.diff * FR_STEP));
                break;
            //When the button gets pressed for a long time push the current duty and frequency values to the output
            case RE_ET_BTN_LONG_PRESSED:
                update_out(true);
                break;
            default:
                break;
        }
    }
    //Safety measure in case this loop for some reason ever exits (which should never happen)
    vTaskDelete(NULL);
}

void app_main()
{
    config_pwm();
    config_i2c();
    config_bluetooth(bt_rec_callback,bt_connect_callback);
    config_display();
    reset_display();
    print_display(25,2,"70");
    print_display(25,38,"7500");
    update_display();
    xTaskCreate(vTask_gpioListener, ENC_TAG, configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
}

/// \name update_duty
/// \brief This function tries to update the current duty cycle value to a new one, this value is updated on both
/// the app and the SSD1306 display,however it does not update the value on the output
/// \param newDuty Duty cycle to try and change to
static void update_duty(uint8_t newDuty)
{
    //Clamp the new value between the limits
    if (newDuty > DC_MAX)
        newDuty = DC_MAX;
    else if(newDuty < DC_MIN)
        newDuty = DC_MIN;

    //If the new duty is the same as the current one don't spend time updating the display and app
    if (currentDuty == newDuty)
        return;

    currentDuty = newDuty;

    update_values();
}

/// \name update_frequency
/// \brief This function tries to update the current frequency value to a new one, this value is updated on both
/// the app and the SSD1306 display,however it does not update the value on the output
/// \param newFrequency Frequency to try and change to
static void update_frequency(uint16_t newFrequency)
{
    //Clamp the new value between the limits
    if (newFrequency > FR_MAX)
        newFrequency = FR_MAX;
    else if (newFrequency < FR_MIN)
        newFrequency = FR_MIN;

    //If the new frequency is the same as the current one don't spend time updating the display and app
    if (newFrequency == currentFrequency)
        return;

    currentFrequency = newFrequency;

    update_values();
}

/// \name update_values
/// \brief This function updates the values on the SSD1306 display and on the app
//TODO:Implement update value via bluetooth
static void update_values()
{
    //Set the display to use the default image
    reset_display();

    char buffer[16];

    //Convert the values to strings and print to screen
    snprintf(buffer,16,"%d",currentDuty);
    print_display(25,2,buffer);

    snprintf(buffer,16,"%d",currentFrequency);
    print_display(25,38,buffer);

    update_display();
}

static void bt_rec_callback(esp_spp_cb_param_t* param)
{
    int newDuty = 0, newFreq = 0;
    struct spp_data_ind_evt_param data = param->data_ind;

    ESP_LOGI("BT","Rec:%s\n",data.data);

    sscanf((char*)data.data,"d=%d,f=%d",&newDuty,&newFreq);

    update_duty(newDuty);
    update_frequency(newFreq);

    update_out(false);
}

static void bt_connect_callback()
{
    update_bt_data();
}

static void update_out(bool isButton)
{
    update_output(currentFrequency,currentDuty);

    if (isButton)
        update_bt_data();
}

static void update_bt_data()
{
    char buffer[16];
    snprintf(buffer,16,"df=%d,%d\n",currentDuty,currentFrequency);
    send_data((int)strlen(buffer),(uint8_t*)buffer);
}
