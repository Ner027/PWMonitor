#ifndef PWMONITOR_BTC_DRIVER_H
#define PWMONITOR_BTC_DRIVER_H
void config_bluetooth(void (*callback)(esp_spp_cb_param_t*),void(*conCallback)(void));
bool send_data(int dataLen,uint8_t* data);
#endif
