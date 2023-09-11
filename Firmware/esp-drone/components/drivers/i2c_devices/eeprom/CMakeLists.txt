idf_component_register(SRCS "eeprom.c"
                       INCLUDE_DIRS "." "include"
                       REQUIRES crazyflie spi_flash)