set(blufi_srcs "src/blufi_main.c"
               "src/blufi_security.c"
               "src/blufi_init.c")

idf_component_register(SRCS "${blufi_srcs}"
                    INCLUDE_DIRS "include"
                    REQUIRES "nvs_flash"
                    REQUIRES "bt")
