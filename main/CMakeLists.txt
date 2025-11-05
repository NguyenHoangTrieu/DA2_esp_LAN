idf_component_register(SRCS "DA2_esp_LAN.c"
                            "../rbg_led_ws_handler/rbg_handler.c" 
                    PRIV_REQUIRES 
                                esp_driver_uart
                                esp_driver_gpio 
                                esp_driver_rmt
                    INCLUDE_DIRS "."
                                "../rbg_led_ws_handler")
