F1_TARGETS  += $(TARGET)
FEATURES    = VCP
FLASH_SIZE  = 128

TARGET_SRC = \
            drivers/accgyro_mpu.c \
            drivers/accgyro_mpu6050.c \
            drivers/rx_nrf24l01.c \
            drivers/barometer_bmp280.c \
            sensors/barometer.c \
            rx/nrf24_syma.c    