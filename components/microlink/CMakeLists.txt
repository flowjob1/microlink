set(ML_CELLULAR_SRC "src/ml_cellular.c")
set(ML_AT_SOCKET_SRC "src/ml_at_socket.c")
if(CONFIG_ML_BOARD_WAVESHARE_ESP32S3_A7670E_USB)
    set(ML_CELLULAR_SRC "src/ml_cellular_usb.c")
    set(ML_AT_SOCKET_SRC "")
endif()

set(SRCS
    "src/microlink.c"
    "src/ml_coord.c"
    "src/ml_derp.c"
    "src/ml_net_io.c"
    "src/ml_wg_mgr.c"
    "src/ml_stun.c"
    "src/ml_noise.c"
    "src/ml_h2.c"
    "src/ml_udp.c"
    "src/ml_tcp.c"
    "src/ml_peer_nvs.c"
    "src/nacl_box.c"
    "src/x25519.c"
    "src/ml_zerocopy.c"
    ${ML_CELLULAR_SRC}
    ${ML_AT_SOCKET_SRC}
    "src/ml_net_switch.c"
    "src/ml_config_httpd.c"
)

set(REQUIRES
    nvs_flash
    esp_wifi
    esp_netif
    esp_event
    esp_timer
    lwip
    mbedtls
    cjson
    driver
    esp_driver_uart
    esp_driver_tsens
    esp_http_server
)

set(PRIV_REQUIRES wireguard_lwip)
list(APPEND PRIV_REQUIRES espressif__esp_modem espressif__esp_modem_usb_dte)

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS
        "include"
        "src"
    REQUIRES ${REQUIRES}
    PRIV_REQUIRES ${PRIV_REQUIRES}
)
