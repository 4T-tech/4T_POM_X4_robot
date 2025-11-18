idf_component_register(SRCS "BLEt.c" "reg_protocol.c" "leg_ik.c" "Dog_Sport.c" "MadgwickAHRS.c" "LIS2MDL.c" "my_uart.c" "main.c" "LSM6DSOW.c" "my_timer.c" "USB_CDC.c"
                    PRIV_REQUIRES spi_flash
                    PRIV_REQUIRES esp_driver_gpio
                    PRIV_REQUIRES esp_driver_uart
                    PRIV_REQUIRES esp_driver_i2c
                    PRIV_REQUIRES esp_driver_gptimer
                    PRIV_REQUIRES esp_driver_usb_serial_jtag
                    PRIV_REQUIRES esp_timer
                    PRIV_REQUIRES nvs_flash
                    PRIV_REQUIRES bt
                    INCLUDE_DIRS "")
