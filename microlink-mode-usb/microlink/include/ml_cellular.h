/**
 * @file ml_cellular.h
 * @brief MicroLink cellular modem abstraction
 *
 * Supports UART-attached SIM7600/SIM7670 boards and the Waveshare
 * ESP32-S3-A7670E-4G board using esp_modem over USB PPP.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Configuration
 * ========================================================================== */

typedef struct {
    int tx_pin;                 /* ESP32 TX -> modem RXD (UART boards only) */
    int rx_pin;                 /* ESP32 RX <- modem TXD (UART boards only) */
    int baud_rate;              /* UART baud rate (UART boards only) */
    const char *apn;            /* APN for data connection (NULL = auto) */
    const char *sim_pin;        /* SIM PIN code (NULL = no PIN required) */
    const char *ppp_user;       /* PPP username (NULL = empty) */
    const char *ppp_pass;       /* PPP password (NULL = empty) */
} ml_cellular_config_t;

/* Default config for legacy UART boards; UART pins are ignored on USB boards. */
#define ML_CELLULAR_DEFAULT_CONFIG() { \
    .tx_pin = 43,               \
    .rx_pin = 44,               \
    .baud_rate = 115200,        \
    .apn = NULL,                \
    .sim_pin = NULL,            \
    .ppp_user = NULL,           \
    .ppp_pass = NULL,           \
}

/* ============================================================================
 * Cellular state
 * ========================================================================== */

typedef enum {
    ML_CELL_STATE_OFF,
    ML_CELL_STATE_INIT,
    ML_CELL_STATE_AT_OK,
    ML_CELL_STATE_SIM_READY,
    ML_CELL_STATE_REGISTERED,
    ML_CELL_STATE_PPP_CONNECTING,
    ML_CELL_STATE_PPP_CONNECTED,
    ML_CELL_STATE_DATA_CONNECTED,
    ML_CELL_STATE_ERROR,
} ml_cellular_state_t;

typedef enum {
    ML_DATA_MODE_NONE,
    ML_DATA_MODE_PPP,
    ML_DATA_MODE_AT_SOCKET,
} ml_cellular_data_mode_t;

/* ============================================================================
 * Module info
 * ========================================================================== */

typedef struct {
    char imei[20];
    char iccid[24];
    char model[32];
    char firmware[64];
    char operator_name[32];
    int rssi;
    int rssi_dbm;
    bool sim_ready;
    bool registered;
    bool data_connected;
} ml_cellular_info_t;

/* ============================================================================
 * Public API
 * ========================================================================== */

esp_err_t ml_cellular_init(const ml_cellular_config_t *config);
void ml_cellular_deinit(void);

esp_err_t ml_cellular_ppp_start(void);
esp_err_t ml_cellular_ppp_stop(void);

esp_err_t ml_cellular_data_start(void);
esp_err_t ml_cellular_data_stop(void);

esp_err_t ml_cellular_connect(void);
ml_cellular_data_mode_t ml_cellular_get_data_mode(void);
bool ml_cellular_is_data_mode(void);

ml_cellular_state_t ml_cellular_get_state(void);
esp_err_t ml_cellular_get_info(ml_cellular_info_t *info);
const char *ml_cellular_get_imei(void);

int ml_cellular_send_at(const char *cmd, char *response, size_t resp_size, int timeout_ms);

#ifdef __cplusplus
}
#endif
