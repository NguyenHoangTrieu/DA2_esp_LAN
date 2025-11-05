#include "rbg_handler.h"

static rmt_channel_handle_t led_chan = NULL;
static rmt_encoder_handle_t simple_encoder = NULL;
static uint8_t led_strip_pixels[NUM_LED_RGB * 3]; // 3 byte/LED GRB

// -- Signal timing for WS2812 --
static const rmt_symbol_word_t ws2812_zero = {
    .level0 = 1,
    .duration0 = 0.3 * RMT_LED_STRIP_RESOLUTION_HZ / 1000000, // T0H=0.3us
    .level1 = 0,
    .duration1 = 0.9 * RMT_LED_STRIP_RESOLUTION_HZ / 1000000, // T0L=0.9us
};
static const rmt_symbol_word_t ws2812_one = {
    .level0 = 1,
    .duration0 = 0.9 * RMT_LED_STRIP_RESOLUTION_HZ / 1000000, // T1H=0.9us
    .level1 = 0,
    .duration1 = 0.3 * RMT_LED_STRIP_RESOLUTION_HZ / 1000000, // T1L=0.3us
};
static const rmt_symbol_word_t ws2812_reset = {
    .level0 = 0,
    .duration0 = RMT_LED_STRIP_RESOLUTION_HZ / 1000000 * 50 / 2,
    .level1 = 0,
    .duration1 = RMT_LED_STRIP_RESOLUTION_HZ / 1000000 * 50 / 2,
};

// --- Encoder callback encode byte to symbols ---
static size_t ws2812_encoder_callback(const void *data, size_t data_size,
                                      size_t symbols_written,
                                      size_t symbols_free,
                                      rmt_symbol_word_t *symbols, bool *done,
                                      void *arg) {
  if (symbols_free < 8)
    return 0;
  size_t data_pos = symbols_written / 8;
  uint8_t *data_bytes = (uint8_t *)data;

  if (data_pos < data_size) {
    size_t symbol_pos = 0;
    for (int bitmask = 0x80; bitmask != 0; bitmask >>= 1) {
      symbols[symbol_pos++] =
          (data_bytes[data_pos] & bitmask) ? ws2812_one : ws2812_zero;
    }
    return symbol_pos;
  } else {
    symbols[0] = ws2812_reset;
    *done = 1;
    return 1;
  }
}

void init_led_strip(void) {
  // Channel RMT
  rmt_tx_channel_config_t tx_chan_config = {
      .clk_src = RMT_CLK_SRC_DEFAULT,
      .gpio_num = RMT_LED_STRIP_GPIO_NUM,
      .mem_block_symbols = 64,
      .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
      .trans_queue_depth = 4,
  };
  ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));

  // Encoder callback
  rmt_simple_encoder_config_t enc_cfg = {.callback = ws2812_encoder_callback};
  ESP_ERROR_CHECK(rmt_new_simple_encoder(&enc_cfg, &simple_encoder));

  ESP_ERROR_CHECK(rmt_enable(led_chan));

  // Turn off LED initially
  memset(led_strip_pixels, 0, sizeof(led_strip_pixels));
  rmt_transmit_config_t tx_config = {.loop_count = 0};
  ESP_ERROR_CHECK(rmt_transmit(led_chan, simple_encoder, led_strip_pixels,
                               sizeof(led_strip_pixels), &tx_config));
  ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
}

// Set Color LED: 0-255 (R, G, B)
void show_led_color(uint8_t r, uint8_t g, uint8_t b) {
  //  GRB for WS2812
  led_strip_pixels[0] = g;
  led_strip_pixels[1] = r;
  led_strip_pixels[2] = b;

  rmt_transmit_config_t tx_config = {.loop_count = 0};
  ESP_ERROR_CHECK(rmt_transmit(led_chan, simple_encoder, led_strip_pixels,
                               sizeof(led_strip_pixels), &tx_config));
  ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
}

// Shortcut changes colors
void led_on(void) { show_led_color(255, 40, 180); }     // Pink: ready
void led_show_blue(void) { show_led_color(0, 0, 255); } // Blue: flashing
void led_show_green(void) {
  show_led_color(0, 255, 0);
} // Green: flash successful
void led_show_red(void) { show_led_color(255, 0, 0); }       // Red: error
void led_show_white(void) { show_led_color(255, 255, 255); } // White: IDLE
void led_show_yellow(void) { show_led_color(255, 255, 0); }  // Yellow: warning
void led_show_orange(void) { show_led_color(255, 165, 0); }  // Orange: busy
void led_off(void) { show_led_color(0, 0, 0); }

void led_toggle_white(void) {
  static bool led_on = false;
  if (led_on) {
    led_off();
  } else {
    led_show_white();
  }
  led_on = !led_on;
}