/**
 * @file ml_cellular_usb.c
 * @brief MicroLink cellular driver for Waveshare ESP32-S3-A7670E-4G
 *
 * Uses esp_modem over USB for PPP data mode.
 */

#include "sdkconfig.h"

#if CONFIG_ML_ENABLE_CELLULAR && CONFIG_ML_BOARD_WAVESHARE_ESP32S3_A7670E_USB

#include "ml_cellular.h"

#include <stdio.h>
#include <string.h>

#include "esp_event.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_modem_api.h"
#include "esp_modem_usb_c_api.h"
#include "esp_modem_usb_config.h"
#include "esp_netif.h"
#include "esp_netif_ppp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

static const char *TAG = "ml_cell_usb";

#define MODEM_BOOT_DELAY_MS      10000
#define MODEM_READY_TIMEOUT_MS   30000
#define NETWORK_WAIT_TIMEOUT_MS  60000
#define PPP_CONNECT_TIMEOUT_MS   120000

#define PPP_GOT_IP_BIT           BIT0
#define PPP_LOST_IP_BIT          BIT1
#define PPP_CONNECT_FAIL_BIT     BIT2
#define PPP_AUTH_FAIL_BIT        BIT3
#define USB_DEVICE_GONE_BIT      BIT4

static struct {
    ml_cellular_config_t config;
    volatile ml_cellular_state_t state;
    ml_cellular_info_t info;
    ml_cellular_data_mode_t data_mode;
    esp_netif_t *ppp_netif;
    esp_modem_dce_t *dce;
    EventGroupHandle_t events;
    esp_event_handler_instance_t ip_event_handler;
    esp_event_handler_instance_t ppp_status_handler;
    bool usb_device_gone;
} s_cell = {
    .state = ML_CELL_STATE_OFF,
    .data_mode = ML_DATA_MODE_NONE,
};

static char s_cmd_buffer[512];
static size_t s_cmd_buffer_len;

static void reset_runtime_state(void)
{
    memset(&s_cell.info, 0, sizeof(s_cell.info));
    s_cell.data_mode = ML_DATA_MODE_NONE;
    s_cell.usb_device_gone = false;
}

static esp_err_t collect_line_cb(uint8_t *data, size_t len)
{
    if (data == NULL || len == 0 || s_cmd_buffer_len >= sizeof(s_cmd_buffer) - 1) {
        return ESP_OK;
    }

    size_t remaining = sizeof(s_cmd_buffer) - s_cmd_buffer_len - 1;
    size_t copy_len = len < remaining ? len : remaining;
    memcpy(s_cmd_buffer + s_cmd_buffer_len, data, copy_len);
    s_cmd_buffer_len += copy_len;
    if (s_cmd_buffer_len < sizeof(s_cmd_buffer) - 1) {
        s_cmd_buffer[s_cmd_buffer_len++] = '\n';
    }
    s_cmd_buffer[s_cmd_buffer_len] = '\0';
    return ESP_OK;
}

static esp_err_t modem_command_collect(const char *command, char *response, size_t response_size, uint32_t timeout_ms)
{
    if (s_cell.dce == NULL || command == NULL || response == NULL || response_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    s_cmd_buffer_len = 0;
    s_cmd_buffer[0] = '\0';

    esp_err_t err = esp_modem_command(s_cell.dce, command, collect_line_cb, timeout_ms);
    if (err != ESP_OK) {
        response[0] = '\0';
        return err;
    }

    snprintf(response, response_size, "%s", s_cmd_buffer);
    return ESP_OK;
}

static bool extract_value(const char *response, const char *prefix, char *out, size_t out_size)
{
    const char *start = NULL;
    const char *end = NULL;

    if (response == NULL || out == NULL || out_size == 0) {
        return false;
    }

    if (prefix != NULL && prefix[0] != '\0') {
        start = strstr(response, prefix);
        if (start != NULL) {
            start += strlen(prefix);
        }
    } else {
        start = response;
    }

    if (start == NULL) {
        return false;
    }

    while (*start == ' ' || *start == ':' || *start == '\r' || *start == '\n' || *start == '"') {
        start++;
    }

    end = start;
    while (*end != '\0' && *end != '\r' && *end != '\n' && *end != '"') {
        end++;
    }

    if (end <= start) {
        return false;
    }

    size_t len = (size_t)(end - start);
    if (len >= out_size) {
        len = out_size - 1;
    }

    memcpy(out, start, len);
    out[len] = '\0';
    return len > 0;
}

static bool response_has_registration(const char *response)
{
    return response != NULL && (strstr(response, ",1") != NULL || strstr(response, ",5") != NULL);
}

static void update_signal_quality(void)
{
    int rssi = 99;
    int ber = 0;

    if (s_cell.dce != NULL && esp_modem_get_signal_quality(s_cell.dce, &rssi, &ber) == ESP_OK) {
        s_cell.info.rssi = rssi;
        s_cell.info.rssi_dbm = (rssi == 99) ? 0 : (-113 + rssi * 2);
    }
}

static esp_err_t wait_for_command_channel(void)
{
    char module_name[sizeof(s_cell.info.model)] = { 0 };
    uint32_t waited_ms = 0;

    while (waited_ms < MODEM_READY_TIMEOUT_MS) {
        if (esp_modem_get_module_name(s_cell.dce, module_name) == ESP_OK) {
            snprintf(s_cell.info.model, sizeof(s_cell.info.model), "%s", module_name);
            s_cell.state = ML_CELL_STATE_AT_OK;
            return ESP_OK;
        }

        vTaskDelay(pdMS_TO_TICKS(500));
        waited_ms += 500;
    }

    return ESP_ERR_TIMEOUT;
}

static esp_err_t read_static_modem_info(void)
{
    char response[sizeof(s_cmd_buffer)] = { 0 };
    int act = 0;

    if (esp_modem_get_imei(s_cell.dce, s_cell.info.imei) != ESP_OK) {
        s_cell.info.imei[0] = '\0';
    }

    if (modem_command_collect("AT+CGMR", response, sizeof(response), 3000) == ESP_OK) {
        extract_value(response, NULL, s_cell.info.firmware, sizeof(s_cell.info.firmware));
    }

    if (modem_command_collect("AT+CICCID", response, sizeof(response), 3000) == ESP_OK) {
        if (!extract_value(response, "+ICCID", s_cell.info.iccid, sizeof(s_cell.info.iccid))) {
            extract_value(response, NULL, s_cell.info.iccid, sizeof(s_cell.info.iccid));
        }
    }

    if (esp_modem_get_operator_name(s_cell.dce, s_cell.info.operator_name, &act) != ESP_OK) {
        s_cell.info.operator_name[0] = '\0';
    }

    update_signal_quality();
    return ESP_OK;
}

static esp_err_t ensure_sim_ready(void)
{
    bool pin_ready = false;

    if (esp_modem_read_pin(s_cell.dce, &pin_ready) != ESP_OK) {
        return ESP_FAIL;
    }

    if (pin_ready) {
        s_cell.info.sim_ready = true;
        s_cell.state = ML_CELL_STATE_SIM_READY;
        return ESP_OK;
    }

    if (s_cell.config.sim_pin == NULL || s_cell.config.sim_pin[0] == '\0') {
        ESP_LOGE(TAG, "SIM requires a PIN but none is configured");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(esp_modem_set_pin(s_cell.dce, s_cell.config.sim_pin), TAG, "Failed to unlock SIM");
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_RETURN_ON_ERROR(esp_modem_read_pin(s_cell.dce, &pin_ready), TAG, "Failed to re-read SIM PIN status");
    if (!pin_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    s_cell.info.sim_ready = true;
    s_cell.state = ML_CELL_STATE_SIM_READY;
    return ESP_OK;
}

static esp_err_t wait_for_network_registration(void)
{
    char response[sizeof(s_cmd_buffer)] = { 0 };
    uint32_t waited_ms = 0;

    while (waited_ms < NETWORK_WAIT_TIMEOUT_MS) {
        if (modem_command_collect("AT+CEREG?", response, sizeof(response), 3000) == ESP_OK &&
            response_has_registration(response)) {
            s_cell.info.registered = true;
            s_cell.state = ML_CELL_STATE_REGISTERED;
            read_static_modem_info();
            return ESP_OK;
        }

        if (modem_command_collect("AT+CREG?", response, sizeof(response), 3000) == ESP_OK &&
            response_has_registration(response)) {
            s_cell.info.registered = true;
            s_cell.state = ML_CELL_STATE_REGISTERED;
            read_static_modem_info();
            return ESP_OK;
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
        waited_ms += 2000;
    }

    return ESP_ERR_TIMEOUT;
}

static void usb_terminal_error_handler(esp_modem_terminal_error_t err)
{
    if (err == ESP_MODEM_TERMINAL_DEVICE_GONE) {
        ESP_LOGW(TAG, "USB modem disconnected");
        s_cell.usb_device_gone = true;
        s_cell.info.data_connected = false;
        s_cell.data_mode = ML_DATA_MODE_NONE;
        s_cell.state = ML_CELL_STATE_ERROR;
        if (s_cell.events != NULL) {
            xEventGroupSetBits(s_cell.events, USB_DEVICE_GONE_BIT | PPP_CONNECT_FAIL_BIT);
        }
        return;
    }

    ESP_LOGW(TAG, "USB terminal error: %d", err);
}

static void ppp_status_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_base;
    (void)event_data;

    if (s_cell.events == NULL) {
        return;
    }

    if (event_id == NETIF_PPP_ERRORAUTHFAIL) {
        xEventGroupSetBits(s_cell.events, PPP_AUTH_FAIL_BIT);
    } else if (event_id == NETIF_PPP_ERRORCONNECT || event_id == NETIF_PPP_CONNECT_FAILED) {
        xEventGroupSetBits(s_cell.events, PPP_CONNECT_FAIL_BIT);
    } else if (event_id < NETIF_PP_PHASE_OFFSET && event_id != NETIF_PPP_ERRORNONE) {
        xEventGroupSetBits(s_cell.events, PPP_CONNECT_FAIL_BIT);
    }
}

static void ppp_ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_base;

    if (s_cell.ppp_netif == NULL || event_data == NULL) {
        return;
    }

    if (event_id == IP_EVENT_PPP_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        if (event->esp_netif != s_cell.ppp_netif) {
            return;
        }

        s_cell.info.data_connected = true;
        s_cell.data_mode = ML_DATA_MODE_PPP;
        s_cell.state = ML_CELL_STATE_PPP_CONNECTED;
        if (s_cell.events != NULL) {
            xEventGroupSetBits(s_cell.events, PPP_GOT_IP_BIT);
        }
    } else if (event_id == IP_EVENT_PPP_LOST_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        if (event->esp_netif != s_cell.ppp_netif) {
            return;
        }

        s_cell.info.data_connected = false;
        s_cell.data_mode = ML_DATA_MODE_NONE;
        s_cell.state = s_cell.info.registered ? ML_CELL_STATE_REGISTERED : ML_CELL_STATE_AT_OK;
        if (s_cell.events != NULL) {
            xEventGroupSetBits(s_cell.events, PPP_LOST_IP_BIT);
        }
    }
}

static void unregister_event_handlers(void)
{
    if (s_cell.ip_event_handler != NULL) {
        esp_event_handler_instance_unregister(IP_EVENT, ESP_EVENT_ANY_ID, s_cell.ip_event_handler);
        s_cell.ip_event_handler = NULL;
    }

    if (s_cell.ppp_status_handler != NULL) {
        esp_event_handler_instance_unregister(NETIF_PPP_STATUS, ESP_EVENT_ANY_ID, s_cell.ppp_status_handler);
        s_cell.ppp_status_handler = NULL;
    }
}

static void destroy_runtime_resources(void)
{
    unregister_event_handlers();

    if (s_cell.dce != NULL) {
        esp_modem_destroy(s_cell.dce);
        s_cell.dce = NULL;
    }

    if (s_cell.ppp_netif != NULL) {
        esp_netif_destroy(s_cell.ppp_netif);
        s_cell.ppp_netif = NULL;
    }

    if (s_cell.events != NULL) {
        vEventGroupDelete(s_cell.events);
        s_cell.events = NULL;
    }
}

esp_err_t ml_cellular_init(const ml_cellular_config_t *config)
{
    esp_err_t err;
    esp_modem_dce_config_t dce_config;
    struct esp_modem_usb_term_config usb_config = ESP_MODEM_A7670_USB_CONFIG();
    esp_modem_dte_config_t dte_config;
    esp_netif_config_t netif_config = ESP_NETIF_DEFAULT_PPP();

    if (s_cell.state != ML_CELL_STATE_OFF) {
        return ESP_ERR_INVALID_STATE;
    }

    if (config != NULL) {
        s_cell.config = *config;
    } else {
        ml_cellular_config_t defaults = ML_CELLULAR_DEFAULT_CONFIG();
        s_cell.config = defaults;
    }

    reset_runtime_state();
    s_cell.state = ML_CELL_STATE_INIT;

    s_cell.events = xEventGroupCreate();
    if (s_cell.events == NULL) {
        s_cell.state = ML_CELL_STATE_ERROR;
        return ESP_ERR_NO_MEM;
    }

    s_cell.ppp_netif = esp_netif_new(&netif_config);
    if (s_cell.ppp_netif == NULL) {
        destroy_runtime_resources();
        s_cell.state = ML_CELL_STATE_ERROR;
        return ESP_FAIL;
    }

    err = esp_event_handler_instance_register(IP_EVENT, ESP_EVENT_ANY_ID, ppp_ip_event_handler, NULL,
                                              &s_cell.ip_event_handler);
    if (err != ESP_OK) {
        destroy_runtime_resources();
        s_cell.state = ML_CELL_STATE_ERROR;
        return err;
    }

    err = esp_event_handler_instance_register(NETIF_PPP_STATUS, ESP_EVENT_ANY_ID, ppp_status_event_handler, NULL,
                                              &s_cell.ppp_status_handler);
    if (err != ESP_OK) {
        destroy_runtime_resources();
        s_cell.state = ML_CELL_STATE_ERROR;
        return err;
    }

    dce_config = (esp_modem_dce_config_t)ESP_MODEM_DCE_DEFAULT_CONFIG(
        (s_cell.config.apn != NULL && s_cell.config.apn[0] != '\0') ? s_cell.config.apn : "");
    usb_config.timeout_ms = MODEM_READY_TIMEOUT_MS;
    dte_config = (esp_modem_dte_config_t)ESP_MODEM_DTE_DEFAULT_USB_CONFIG(usb_config);

    ESP_LOGI(TAG, "Waiting for A7670E USB modem");
    s_cell.dce = esp_modem_new_dev_usb(ESP_MODEM_DCE_SIM7600, &dte_config, &dce_config, s_cell.ppp_netif);
    if (s_cell.dce == NULL) {
        destroy_runtime_resources();
        s_cell.state = ML_CELL_STATE_ERROR;
        return ESP_FAIL;
    }

    err = esp_modem_set_error_cb(s_cell.dce, usb_terminal_error_handler);
    if (err != ESP_OK) {
        destroy_runtime_resources();
        s_cell.state = ML_CELL_STATE_ERROR;
        return err;
    }

    ESP_LOGI(TAG, "USB modem connected, waiting %d ms for boot", MODEM_BOOT_DELAY_MS);
    vTaskDelay(pdMS_TO_TICKS(MODEM_BOOT_DELAY_MS));

    err = wait_for_command_channel();
    if (err != ESP_OK) {
        destroy_runtime_resources();
        s_cell.state = ML_CELL_STATE_ERROR;
        return err;
    }

    err = ensure_sim_ready();
    if (err != ESP_OK) {
        destroy_runtime_resources();
        s_cell.state = ML_CELL_STATE_ERROR;
        return err;
    }

    err = wait_for_network_registration();
    if (err != ESP_OK) {
        destroy_runtime_resources();
        s_cell.state = ML_CELL_STATE_ERROR;
        return err;
    }

    ESP_LOGI(TAG, "Cellular modem ready: %s IMEI=%s Operator=%s RSSI=%d",
             s_cell.info.model,
             s_cell.info.imei,
             s_cell.info.operator_name,
             s_cell.info.rssi);
    return ESP_OK;
}

void ml_cellular_deinit(void)
{
    if (s_cell.state == ML_CELL_STATE_OFF) {
        return;
    }

    ml_cellular_ppp_stop();
    destroy_runtime_resources();
    reset_runtime_state();
    memset(&s_cell.config, 0, sizeof(s_cell.config));
    s_cell.state = ML_CELL_STATE_OFF;
}

esp_err_t ml_cellular_ppp_start(void)
{
    EventBits_t bits;
    const char *ppp_user = (s_cell.config.ppp_user != NULL) ? s_cell.config.ppp_user : "";
    const char *ppp_pass = (s_cell.config.ppp_pass != NULL) ? s_cell.config.ppp_pass : "";
    esp_netif_auth_type_t auth = (ppp_user[0] != '\0' || ppp_pass[0] != '\0')
        ? NETIF_PPP_AUTHTYPE_PAP
        : NETIF_PPP_AUTHTYPE_NONE;
    esp_netif_ppp_config_t ppp_cfg = {
        .ppp_phase_event_enabled = false,
        .ppp_error_event_enabled = true,
    };

    if (s_cell.dce == NULL || s_cell.ppp_netif == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_cell.state < ML_CELL_STATE_REGISTERED) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_cell.info.data_connected && s_cell.data_mode == ML_DATA_MODE_PPP) {
        return ESP_OK;
    }

    xEventGroupClearBits(s_cell.events,
                         PPP_GOT_IP_BIT | PPP_LOST_IP_BIT | PPP_CONNECT_FAIL_BIT |
                         PPP_AUTH_FAIL_BIT | USB_DEVICE_GONE_BIT);

    ESP_RETURN_ON_ERROR(esp_netif_ppp_set_auth(s_cell.ppp_netif, auth, ppp_user, ppp_pass), TAG,
                        "Failed to configure PPP authentication");
    ESP_RETURN_ON_ERROR(esp_netif_ppp_set_params(s_cell.ppp_netif, &ppp_cfg), TAG,
                        "Failed to configure PPP event settings");

    if (s_cell.config.apn != NULL && s_cell.config.apn[0] != '\0') {
        ESP_RETURN_ON_ERROR(esp_modem_set_apn(s_cell.dce, s_cell.config.apn), TAG, "Failed to set APN");
    }

    s_cell.state = ML_CELL_STATE_PPP_CONNECTING;
    ESP_RETURN_ON_ERROR(esp_modem_set_mode(s_cell.dce, ESP_MODEM_MODE_DATA), TAG, "Failed to enter PPP data mode");

    bits = xEventGroupWaitBits(s_cell.events,
                               PPP_GOT_IP_BIT | PPP_CONNECT_FAIL_BIT | PPP_AUTH_FAIL_BIT | USB_DEVICE_GONE_BIT,
                               pdFALSE, pdFALSE, pdMS_TO_TICKS(PPP_CONNECT_TIMEOUT_MS));

    if ((bits & PPP_GOT_IP_BIT) != 0) {
        return ESP_OK;
    }

    if ((bits & USB_DEVICE_GONE_BIT) == 0) {
        esp_modem_set_mode(s_cell.dce, ESP_MODEM_MODE_COMMAND);
    }

    s_cell.info.data_connected = false;
    s_cell.data_mode = ML_DATA_MODE_NONE;
    s_cell.state = s_cell.info.registered ? ML_CELL_STATE_REGISTERED : ML_CELL_STATE_ERROR;

    if ((bits & PPP_AUTH_FAIL_BIT) != 0) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    if ((bits & PPP_CONNECT_FAIL_BIT) != 0) {
        return ESP_FAIL;
    }
    if ((bits & USB_DEVICE_GONE_BIT) != 0) {
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_ERR_TIMEOUT;
}

esp_err_t ml_cellular_ppp_stop(void)
{
    if (s_cell.dce == NULL) {
        return ESP_OK;
    }

    if (esp_modem_get_mode(s_cell.dce) == ESP_MODEM_MODE_DATA) {
        esp_err_t err = esp_modem_set_mode(s_cell.dce, ESP_MODEM_MODE_COMMAND);
        if (err != ESP_OK && !s_cell.usb_device_gone) {
            return err;
        }
    }

    s_cell.info.data_connected = false;
    if (!s_cell.usb_device_gone) {
        s_cell.state = s_cell.info.registered ? ML_CELL_STATE_REGISTERED : ML_CELL_STATE_AT_OK;
    }
    s_cell.data_mode = ML_DATA_MODE_NONE;
    return ESP_OK;
}

esp_err_t ml_cellular_data_start(void)
{
    ESP_LOGW(TAG, "AT socket bridge is not supported on the A7670E USB transport");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t ml_cellular_data_stop(void)
{
    if (s_cell.data_mode == ML_DATA_MODE_AT_SOCKET) {
        s_cell.data_mode = ML_DATA_MODE_NONE;
    }
    return ESP_OK;
}

esp_err_t ml_cellular_connect(void)
{
    esp_err_t err = ml_cellular_ppp_start();
    if (err == ESP_OK) {
        s_cell.data_mode = ML_DATA_MODE_PPP;
    } else {
        s_cell.data_mode = ML_DATA_MODE_NONE;
    }
    return err;
}

ml_cellular_data_mode_t ml_cellular_get_data_mode(void)
{
    return s_cell.data_mode;
}

bool ml_cellular_is_data_mode(void)
{
    return s_cell.data_mode == ML_DATA_MODE_AT_SOCKET;
}

ml_cellular_state_t ml_cellular_get_state(void)
{
    return s_cell.state;
}

esp_err_t ml_cellular_get_info(ml_cellular_info_t *info)
{
    if (info == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    update_signal_quality();
    *info = s_cell.info;
    return ESP_OK;
}

const char *ml_cellular_get_imei(void)
{
    return s_cell.info.imei[0] ? s_cell.info.imei : NULL;
}

int ml_cellular_send_at(const char *cmd, char *response, size_t resp_size, int timeout_ms)
{
    if (cmd == NULL || response == NULL || resp_size == 0 || s_cell.dce == NULL) {
        return -1;
    }

    if (esp_modem_get_mode(s_cell.dce) == ESP_MODEM_MODE_DATA) {
        ESP_LOGW(TAG, "Cannot send AT commands while PPP is active");
        return -1;
    }

    if (modem_command_collect(cmd, response, resp_size, (uint32_t)timeout_ms) != ESP_OK) {
        return -1;
    }

    return (int)strlen(response);
}

#endif /* CONFIG_ML_ENABLE_CELLULAR && CONFIG_ML_BOARD_WAVESHARE_ESP32S3_A7670E_USB */
