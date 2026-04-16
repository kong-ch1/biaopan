set(BSP_COMPONENTS_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../components")

idf_component_register(
    SRCS "main.c"
         "watch_face.c"
         "app_video.c"
         "watch_zh_patch_font.c"
         "${BSP_COMPONENTS_DIR}/qma6100p/qma6100p.c"
         "${BSP_COMPONENTS_DIR}/sensors/bmi270/src/bmi270.c"

    INCLUDE_DIRS "."
                 "${BSP_COMPONENTS_DIR}/qma6100p/include"
                 "${BSP_COMPONENTS_DIR}/sensors/bmi270/include"
    PRIV_INCLUDE_DIRS "${BSP_COMPONENTS_DIR}/sensors/bmi270/include_priv"
    REQUIRES esp_driver_i2c esp_driver_spi esp_driver_i2s esp_lcd esp_driver_cam esp_lvgl_port esp_event esp_netif esp_wifi nvs_flash esp_http_client)




