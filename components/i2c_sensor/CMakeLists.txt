set(i2c_sensor_srcs "src/i2c_sensor.c")
idf_component_register(SRCS "${i2c_sensor_srcs}"
                       INCLUDE_DIRS "include"
                       REQUIRES "nvs_flash"
                       REQUIRES "bt")
                       
