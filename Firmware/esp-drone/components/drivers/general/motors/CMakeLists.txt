idf_component_register(SRCS "motors_def_cf2.c" "motors.c"
                       INCLUDE_DIRS "." "include"
                        REQUIRES crazyflie platform config)