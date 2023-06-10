set(app_src Sim7090G_module.c)
set(pri_req)
idf_component_register(SRCS ${app_src}
                    INCLUDE_DIRS "."
                    REQUIRES ${pri_req} esp_event nvs_flash esp_timer driver)

