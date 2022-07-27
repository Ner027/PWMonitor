#include <driver/ledc.h>
#include <esp_log.h>
#include <math.h>
#include "pwm_driver.h"

void config_pwm()
{
    ledc_timer_config_t timerConfig =
            {
                    .clk_cfg = LEDC_USE_APB_CLK,
                    .duty_resolution = LEDC_TIMER_8_BIT,
                    .timer_num = LEDC_TIMER_0,
                    .speed_mode = LEDC_HIGH_SPEED_MODE,
                    .freq_hz = 7500,
            };

    ledc_timer_config(&timerConfig);

    ledc_channel_config_t ledcChannelConfig =
            {
                    .speed_mode = LEDC_HIGH_SPEED_MODE,
                    .channel = LEDC_CHANNEL_0,
                    .gpio_num = GPIO_NUM_33,
                    .duty = 1435,
                    .hpoint = 0,
                    .timer_sel = LEDC_TIMER_0
            };

    ledc_channel_config(&ledcChannelConfig);

    ledc_set_duty(LEDC_HIGH_SPEED_MODE,LEDC_CHANNEL_0,1435);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE,LEDC_CHANNEL_0);
}

/// \name update_output
///\brief This function updates the output PWM signal to use the passed values
/// \param freq Frequency to set
/// \param duty Duty to set
void update_output(uint16_t freq,uint8_t duty)
{
    uint8_t dutyRlVal = (uint32_t) round(duty * 2.55);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE,LEDC_CHANNEL_0,dutyRlVal);
    ledc_set_freq(LEDC_HIGH_SPEED_MODE,LEDC_TIMER_0,freq);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE,LEDC_CHANNEL_0);

    ESP_LOGI("PWM","Frequency set to:%u Duty set to:%u",freq,dutyRlVal);
}