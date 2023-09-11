idf_component_register(SRCS "wifi_esp32.c"
                      INCLUDE_DIRS "." "include"
                      REQUIRES crazyflie platform config esp_wifi)