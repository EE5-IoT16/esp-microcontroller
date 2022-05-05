set(pulse_counter_srcs "src/pulse_counter.c")

idf_component_register(SRCS "${pulse_counter_srcs}"
                       INCLUDE_DIRS "include"
                       REQUIRES "nvs_flash"
                       REQUIRES "bt")
