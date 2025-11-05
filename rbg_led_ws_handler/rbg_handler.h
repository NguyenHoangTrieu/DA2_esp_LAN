#ifndef RBG_HANDLER_H
#define RBG_HANDLER_H
#include "driver/rmt_tx.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"


#define RMT_LED_STRIP_GPIO_NUM 48 // GPIO 48 for WS2812 data
#define RMT_LED_STRIP_RESOLUTION_HZ 10000000
#define NUM_LED_RGB 1

extern void init_led_strip(void);
void show_led_color(uint8_t r, uint8_t g, uint8_t b);
extern void led_on(void);
extern void led_off(void);
extern void led_show_red(void);
extern void led_show_blue(void);
extern void led_show_green(void);
extern void led_show_white(void);
extern void led_toggle_white(void);
extern void led_show_yellow(void);
extern void led_show_orange(void);
#endif // RBG_HANDLER_H