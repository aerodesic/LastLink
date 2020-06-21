#undef CONFIG_LASTLINK_CONNECT_ETHERNET
#define CONFIG_LASTLINK_CONNECT_WIFI
#undef CONFIG_LASTLINK_CONNECT_IPV6

/* Common functions for protocol examples, to establish Wi-Fi or Ethernet connection.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "esp_netif.h"

#ifdef CONFIG_LASTLINK_CONNECT_ETHERNET
#define LASTLINK_INTERFACE get_lastlink_netif()
#endif

#ifdef CONFIG_LASTLINK_CONNECT_WIFI
#define LASTLINK_INTERFACE get_lastlink_netif()
#endif

/**
 * @brief Configure Wi-Fi or Ethernet, connect, wait for IP
 *
 * This all-in-one helper function is used in protocols examples to
 * reduce the amount of boilerplate in the example.
 *
 * It is not intended to be used in real world applications.
 * See examples under examples/wifi/getting_started/ and examples/ethernet/
 * for more complete Wi-Fi or Ethernet initialization code.
 *
 * Read "Establishing Wi-Fi or Ethernet Connection" section in
 * examples/protocols/README.md for more information about this function.
 *
 * @return ESP_OK on successful connection
 */
esp_err_t network_connect(void);

/**
 * Counterpart to network_connect, de-initializes Wi-Fi or Ethernet
 */
esp_err_t network_disconnect(void);

/**
 * @brief Configure stdin and stdout to use blocking I/O
 *
 * This helper function is used in ASIO examples. It wraps installing the
 * UART driver and configuring VFS layer to use UART driver for console I/O.
 */
esp_err_t network_configure_stdin_stdout(void);

/**
 * @brief Returns esp-netif pointer created by network_connect()
 *
 * @note If multiple interfaces active at once, this API return NULL
 * In that case the get_lastlink_netif_from_desc() should be used
 * to get esp-netif pointer based on interface description
 */
esp_netif_t *get_lastlink_netif(void);

/**
 * @brief Returns esp-netif pointer created by network_connect() described by
 * the supplied desc field
 *
 * @param desc Textual interface of created network interface, for example "sta"
 * indicate default WiFi station, "eth" default Ethernet interface.
 *
 */
esp_netif_t *get_lastlink_netif_from_desc(const char *desc);

#ifdef __cplusplus
}
#endif
