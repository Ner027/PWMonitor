#include "ssd1306_driver.h"
#include "ssd1306_const.h"

#include "driver/i2c.h"
#include <esp_log.h>
#include <string.h>
#include "media.h"

uint8_t displayGrid[DSP_WIDTH][DSP_RC];

void config_display()
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);

    i2c_master_write_byte(cmd, OLED_CMD_SET_CHARGE_PUMP, true);
    i2c_master_write_byte(cmd, 0x14, true);

    i2c_master_write_byte(cmd,0x20,true);
    i2c_master_write_byte(cmd,0x02,true);

    i2c_master_write_byte(cmd, OLED_CMD_SET_SEGMENT_REMAP, true); // reverse left-right mapping
    i2c_master_write_byte(cmd, OLED_CMD_SET_COM_SCAN_MODE, true);

    i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_ON, true);
    i2c_master_stop(cmd);

    esp_err_t retCode = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);

    if (retCode == ESP_OK)
        ESP_LOGI(TAG_DSP, "Display up and running!");
    else ESP_LOGE(TAG_DSP, "Failed to set up the display,exited with code: %x", retCode);

    i2c_cmd_link_delete(cmd);
}


void vTask_updateDisplay(void* params)
{
    i2c_cmd_handle_t cmd;

    for (uint8_t i = 0; i < 8; i++)
    {
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_SINGLE, true);
        i2c_master_write_byte(cmd, 0xB0 | i, true);
        i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);

        for (uint8_t j = 0; j < 128; j++)
            i2c_master_write_byte(cmd, displayGrid[j][i], true);

        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
    }

    vTaskDelete(NULL);
}

void set_pixel(uint8_t x, uint8_t y,uint8_t state)
{
    if (state) displayGrid[x][y >> 3] |= (1 << (y & 7));
    else displayGrid[x][y >> 3] &= ~(1 << (y & 7));
}

uint8_t coords_to_pixel_val(uint8_t x, uint8_t y,uint8_t index)
{
    return ((dspChars[index][x][(y >> 3)]) >> (y & 7)) & 0x1;
}


void write_char(uint8_t x, uint8_t y, char c)
{
    if (c < 48 || c > 57)
        return;

    if (x > DSP_WIDTH || y > DSP_HEIGHT)
        return;

    uint8_t index = c - 48;

    for (uint8_t i = 0; i < 24; ++i)
        for (uint8_t j = 0; j < 13 ; ++j)
            set_pixel(x + j,y + i, coords_to_pixel_val(j,i,index));
}

void print_display(uint8_t x, uint8_t y,const char* str)
{
    for (uint8_t i = 0; str[i] != '\0'; ++i)
    {
        uint8_t delta = 16;

        if (str[i] == '1' || str[i + 1] == '1')
            delta = 12;

        write_char(x,y,str[i]);

        x+= delta;
    }
}


void clear_display()
{
    memset(displayGrid,0, sizeof(displayGrid));
    //update_display();
}

void set_display(uint8_t  newGrid[DSP_WIDTH][DSP_RC])
{
    memcpy(displayGrid,newGrid,sizeof(displayGrid));
}

void reset_display()
{
    memcpy(displayGrid,dspImage,sizeof(displayGrid));
}

void update_display()
{
    xTaskCreate(vTask_updateDisplay,
                "UpdateDisplay",
                2048,
                NULL,
                6,
                NULL);

}
