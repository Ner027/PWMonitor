#ifndef BT_DISCOVERY_SSD1306_DRIVER_H
#define BT_DISCOVERY_SSD1306_DRIVER_H

#define TAG_DSP "Display"
#define DSP_WIDTH 128
#define DSP_HEIGHT 64
#define DSP_RC 8

typedef unsigned char uint8_t;

void config_display();
void clear_display();
void update_display();
void set_display(uint8_t  newGrid[DSP_WIDTH][DSP_RC]);
void print_display(uint8_t x, uint8_t y,const char* str);
void reset_display();

#endif
