idf_component_register(SRCS "hmc5883l.c"
                       INCLUDE_DIRS "." "include"
                       REQUIRES i2c_bus 
                       PRIV_REQUIRES crazyflie platform)