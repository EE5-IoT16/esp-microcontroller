# Embed the server root certificate into the final binary
#
# (If this was a component, we would set COMPONENT_EMBED_TXTFILES here.)
set(http_client_srcs "src/http_client.c")

idf_component_register(SRCS "${http_client_srcs}"
                    INCLUDE_DIRS "include"
                    EMBED_TXTFILES howsmyssl_com_root_cert.pem
                    postman_root_cert.pem
                    REQUIRES "nvs_flash"
                    REQUIRES "esp-tls"
                    REQUIRES "esp_http_client"
                    )


                   