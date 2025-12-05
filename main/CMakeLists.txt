idf_component_register(SRCS "DA2_esp_LAN.c"
                            "../rbg_led_ws_handler/rbg_handler.c"
                            "../Application/FOTA_LAN/src/fota_lan_handler.c"
                            "../Application/MCU_WAN_Handler/src/mcu_wan_handler.c"
                            "../Application/Config_Handler/src/config_handler.c"
                            "../Application/Config_Handler/src/config_load_save.c"
                            "../Application/CAN_Handler/src/can_handler.c"
                            "../Application/LoRa_Handler/src/lora_tdma_connect.c"
                            "../BSP/MCU_WAN_Communication/src/wan_comm.c"
                            "../BSP/LoRa_Module_Communicaiton/src/lora_e32_comm.c"
                            "../BSP/CAN_Communication/src/can_driver.c"
                            "../Middleware/LoRa_Handler/src/lora_tdma_handler.c"
                    PRIV_REQUIRES 
                                esp_wifi
                                nvs_flash
                                mqtt
                                esp_adc
                                esp_driver_i2c
                                esp_driver_uart
                                esp_driver_gpio
                                esp_driver_spi
                                esp_driver_rmt
                                esp_driver_spi
                                esp_driver_twai
                                usb
                                mbedtls
                                esp_driver_usb_serial_jtag
                                esp_http_client
                                app_update
                                esp_https_ota
                                esp_netif
                                esp_psram
                    INCLUDE_DIRS "."
                                "../Application/FOTA_LAN/include"
                                "../Application/MCU_WAN_Handler/include"
                                "../Application/Config_Handler/include"
                                "../Application/CAN_Handler/include"
                                "../Application/LoRa_Handler/include"
                                "../BSP/MCU_WAN_Communication/include"
                                "../BSP/LoRa_Module_Communicaiton/include"
                                "../BSP/CAN_Communication/include"
                                "../Middleware/LoRa_Handler/include"
                                "../rbg_led_ws_handler"
                    EMBED_TXTFILES ${project_dir}/server_certs/ca_cert.pem)

