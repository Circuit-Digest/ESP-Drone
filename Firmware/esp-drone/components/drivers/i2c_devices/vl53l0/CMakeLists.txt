idf_component_register(SRCS "vl53l0x.c" "zranger.c"
                       INCLUDE_DIRS "." "include"
                     REQUIRES i2c_bus crazyflie platform config)