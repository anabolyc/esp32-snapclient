/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "ping/ping_sock.h"
#include "lwip/inet.h"
#include <lwip/sockets.h>
#include "esp_wifi.h"

/* Access player playback state to avoid interrupting active playback.
 * WARNING: This extern bool is accessed without synchronization. The player
 * task may modify it while we read it, creating a TOCTOU race. For now we
 * accept this as the window is small and consequences are minor (at worst,
 * takeover happens slightly before/after intended). A proper fix would use
 * atomic operations or include playerstarted in our semaphore-protected state.
 */
extern bool playerstarted;

#if CONFIG_SNAPCLIENT_USE_SPI_ETHERNET
#include "driver/spi_master.h"
#endif

#include "network_interface.h"
#include "settings_manager.h"

static const char *TAG = "ETH_IF";

static uint8_t eth_port_cnt = 0;

static esp_netif_ip_info_t ip_info = {{0}, {0}, {0}};
static bool connected = false;
static SemaphoreHandle_t connIpSemaphoreHandle = NULL;
/* Track takeover intent and whether we changed the default netif to prefer Ethernet */
static bool we_changed_default_netif = false;
static bool want_eth_takeover = false;

// Ethernet mode: 0=Disabled (default), 1=DHCP, 2=Static
static int32_t current_eth_mode = 0;

/* State guards for static IP application */
static bool static_ip_in_progress = false;
static bool static_ip_pending = false;
static esp_netif_t *static_ip_netif = NULL;  // Protected netif pointer for static IP task
static TaskHandle_t static_ip_task_handle = NULL;  // Track task for cleanup on disconnect

/* Forward declaration for reconnect request */
extern void app_request_reconnect(void);

/**
 * @brief Cleanup Ethernet drivers and free handles on initialization failure
 */
static void eth_cleanup_drivers(esp_eth_handle_t *handles, uint8_t count) {
    if (!handles) return;

    for (int i = 0; i < count; i++) {
        if (handles[i]) {
            esp_eth_stop(handles[i]);
            esp_eth_driver_uninstall(handles[i]);
        }
    }
    free(handles);
}

/**
 * @brief Auto-disable Ethernet and persist to NVS on initialization failure
 * This allows the device to boot with WiFi fallback instead of reboot-looping
 */
static void eth_auto_disable_and_persist(void) {
    ESP_LOGW(TAG, "Ethernet init failed - auto-disabling to allow boot");
    current_eth_mode = 0;

    esp_err_t err = settings_set_eth_mode(0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to persist eth_mode=0 to NVS: %s", esp_err_to_name(err));
        ESP_LOGW(TAG, "Ethernet disabled for this boot only - may retry on reboot");
    } else {
        ESP_LOGI(TAG, "Ethernet disabled and saved to NVS. Re-enable via web UI when hardware is ready.");
    }
}

// Gateway ping state
static SemaphoreHandle_t ping_done_sem = NULL;
static bool ping_success = false;

#if CONFIG_SNAPCLIENT_SPI_ETHERNETS_NUM
#define SPI_ETHERNETS_NUM CONFIG_SNAPCLIENT_SPI_ETHERNETS_NUM
#else
#define SPI_ETHERNETS_NUM 0
#endif

#if CONFIG_SNAPCLIENT_USE_INTERNAL_ETHERNET
#define INTERNAL_ETHERNETS_NUM 1
#else
#define INTERNAL_ETHERNETS_NUM 0
#endif

#define INIT_SPI_ETH_MODULE_CONFIG(eth_module_config, num)                     \
  do {                                                                         \
    eth_module_config[num].spi_cs_gpio =                                       \
        CONFIG_SNAPCLIENT_ETH_SPI_CS##num##_GPIO;                              \
    eth_module_config[num].int_gpio =                                          \
        CONFIG_SNAPCLIENT_ETH_SPI_INT##num##_GPIO;                             \
    eth_module_config[num].phy_reset_gpio =                                    \
        CONFIG_SNAPCLIENT_ETH_SPI_PHY_RST##num##_GPIO;                         \
    eth_module_config[num].phy_addr = CONFIG_SNAPCLIENT_ETH_SPI_PHY_ADDR##num; \
  } while (0)

typedef struct {
  uint8_t spi_cs_gpio;
  uint8_t int_gpio;
  int8_t phy_reset_gpio;
  uint8_t phy_addr;
  uint8_t *mac_addr;
} spi_eth_module_config_t;

#if CONFIG_SNAPCLIENT_USE_INTERNAL_ETHERNET
/**
 * @brief Internal ESP32 Ethernet initialization
 *
 * @param[out] mac_out optionally returns Ethernet MAC object
 * @param[out] phy_out optionally returns Ethernet PHY object
 * @return
 *          - esp_eth_handle_t if init succeeded
 *          - NULL if init failed
 */
static esp_eth_handle_t eth_init_internal(esp_eth_mac_t **mac_out,
                                          esp_eth_phy_t **phy_out) {
  esp_eth_handle_t ret = NULL;

  // Init common MAC and PHY configs to default
  eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
  eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();

  // Update PHY config based on board specific configuration
  phy_config.phy_addr = CONFIG_SNAPCLIENT_ETH_PHY_ADDR;
  phy_config.reset_gpio_num = CONFIG_SNAPCLIENT_ETH_PHY_RST_GPIO;

  // Init vendor specific MAC config to default
  eth_esp32_emac_config_t esp32_emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
  // Update vendor specific MAC config based on board configuration
  esp32_emac_config.smi_mdc_gpio_num = CONFIG_SNAPCLIENT_ETH_MDC_GPIO;
  esp32_emac_config.smi_mdio_gpio_num = CONFIG_SNAPCLIENT_ETH_MDIO_GPIO;

  // Set clock mode and GPIO
#if CONFIG_ETH_RMII_CLK_INPUT
  esp32_emac_config.clock_config.rmii.clock_mode = EMAC_CLK_EXT_IN;
  esp32_emac_config.clock_config.rmii.clock_gpio = CONFIG_ETH_RMII_CLK_IN_GPIO;
#elif CONFIG_ETH_RMII_CLK_OUTPUT
  esp32_emac_config.clock_config.rmii.clock_mode = EMAC_CLK_OUT;
  esp32_emac_config.clock_config.rmii.clock_gpio = CONFIG_ETH_RMII_CLK_OUT_GPIO;
#else
  esp32_emac_config.clock_config.rmii.clock_mode = EMAC_CLK_DEFAULT;
  esp32_emac_config.clock_config.rmii.clock_gpio = EMAC_CLK_OUT_GPIO;
#endif

  // Create new ESP32 Ethernet MAC instance
  esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&esp32_emac_config, &mac_config);

  // Create new PHY instance based on board configuration
#if CONFIG_SNAPCLIENT_ETH_PHY_IP101
  esp_eth_phy_t *phy = esp_eth_phy_new_ip101(&phy_config);
#elif CONFIG_SNAPCLIENT_ETH_PHY_RTL8201
  esp_eth_phy_t *phy = esp_eth_phy_new_rtl8201(&phy_config);
#elif CONFIG_SNAPCLIENT_ETH_PHY_LAN87XX
  esp_eth_phy_t *phy = esp_eth_phy_new_lan87xx(&phy_config);
#elif CONFIG_SNAPCLIENT_ETH_PHY_DP83848
  esp_eth_phy_t *phy = esp_eth_phy_new_dp83848(&phy_config);
#elif CONFIG_SNAPCLIENT_ETH_PHY_KSZ80XX
  esp_eth_phy_t *phy = esp_eth_phy_new_ksz80xx(&phy_config);
#endif

  // Init Ethernet driver to default and install it
  esp_eth_handle_t eth_handle = NULL;
  esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy);
  ESP_GOTO_ON_FALSE(esp_eth_driver_install(&config, &eth_handle) == ESP_OK,
                    NULL, err, TAG, "Ethernet driver install failed");

  if (mac_out != NULL) {
    *mac_out = mac;
  }
  if (phy_out != NULL) {
    *phy_out = phy;
  }
  return eth_handle;
err:
  if (eth_handle != NULL) {
    esp_eth_driver_uninstall(eth_handle);
  }
  if (mac != NULL) {
    mac->del(mac);
  }
  if (phy != NULL) {
    phy->del(phy);
  }
  return ret;
}
#endif  // CONFIG_SNAPCLIENT_USE_INTERNAL_ETHERNET

#if CONFIG_SNAPCLIENT_USE_SPI_ETHERNET
/**
 * @brief SPI bus initialization (to be used by Ethernet SPI modules)
 *
 * @return
 *          - ESP_OK on success
 */
static esp_err_t spi_bus_init(void) {
  esp_err_t ret = ESP_OK;

  // Install GPIO ISR handler to be able to service SPI Eth modules interrupts
  ret = gpio_install_isr_service(0);
  if (ret != ESP_OK) {
    if (ret == ESP_ERR_INVALID_STATE) {
      ESP_LOGW(TAG, "GPIO ISR handler has been already installed");
      ret = ESP_OK;  // ISR handler has been already installed so no issues
    } else {
      ESP_LOGE(TAG, "GPIO ISR handler install failed");
      goto err;
    }
  }

  // Init SPI bus
  spi_bus_config_t buscfg = {
      .miso_io_num = CONFIG_SNAPCLIENT_ETH_SPI_MISO_GPIO,
      .mosi_io_num = CONFIG_SNAPCLIENT_ETH_SPI_MOSI_GPIO,
      .sclk_io_num = CONFIG_SNAPCLIENT_ETH_SPI_SCLK_GPIO,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
  };
  ESP_GOTO_ON_ERROR(spi_bus_initialize(CONFIG_SNAPCLIENT_ETH_SPI_HOST, &buscfg,
                                       SPI_DMA_CH_AUTO),
                    err, TAG, "SPI host #%d init failed",
                    CONFIG_SNAPCLIENT_ETH_SPI_HOST);

err:
  return ret;
}

/**
 * @brief Ethernet SPI modules initialization
 *
 * @param[in] spi_eth_module_config specific SPI Ethernet module configuration
 * @param[out] mac_out optionally returns Ethernet MAC object
 * @param[out] phy_out optionally returns Ethernet PHY object
 * @return
 *          - esp_eth_handle_t if init succeeded
 *          - NULL if init failed
 */
static esp_eth_handle_t eth_init_spi(
    spi_eth_module_config_t *spi_eth_module_config, esp_eth_mac_t **mac_out,
    esp_eth_phy_t **phy_out) {
  esp_eth_handle_t ret = NULL;

  // Init common MAC and PHY configs to default
  eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
  eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();

  // Update PHY config based on board specific configuration
  phy_config.phy_addr = spi_eth_module_config->phy_addr;
  phy_config.reset_gpio_num = spi_eth_module_config->phy_reset_gpio;

  // Configure SPI interface for specific SPI module
  spi_device_interface_config_t spi_devcfg = {
      .mode = 0,
      .clock_speed_hz = CONFIG_SNAPCLIENT_ETH_SPI_CLOCK_MHZ * 1000 * 1000,
      .queue_size = 20,
      .spics_io_num = spi_eth_module_config->spi_cs_gpio};
  // Init vendor specific MAC config to default, and create new SPI Ethernet MAC
  // instance and new PHY instance based on board configuration
#if CONFIG_SNAPCLIENT_USE_KSZ8851SNL
  eth_ksz8851snl_config_t ksz8851snl_config = ETH_KSZ8851SNL_DEFAULT_CONFIG(
      CONFIG_SNAPCLIENT_ETH_SPI_HOST, &spi_devcfg);
  ksz8851snl_config.int_gpio_num = spi_eth_module_config->int_gpio;
  esp_eth_mac_t *mac =
      esp_eth_mac_new_ksz8851snl(&ksz8851snl_config, &mac_config);
  esp_eth_phy_t *phy = esp_eth_phy_new_ksz8851snl(&phy_config);
#elif CONFIG_SNAPCLIENT_USE_DM9051
  eth_dm9051_config_t dm9051_config =
      ETH_DM9051_DEFAULT_CONFIG(CONFIG_SNAPCLIENT_ETH_SPI_HOST, &spi_devcfg);
  dm9051_config.int_gpio_num = spi_eth_module_config->int_gpio;
  esp_eth_mac_t *mac = esp_eth_mac_new_dm9051(&dm9051_config, &mac_config);
  esp_eth_phy_t *phy = esp_eth_phy_new_dm9051(&phy_config);
#elif CONFIG_SNAPCLIENT_USE_W5500
  eth_w5500_config_t w5500_config =
      ETH_W5500_DEFAULT_CONFIG(CONFIG_SNAPCLIENT_ETH_SPI_HOST, &spi_devcfg);
  w5500_config.int_gpio_num = spi_eth_module_config->int_gpio;
  esp_eth_mac_t *mac = esp_eth_mac_new_w5500(&w5500_config, &mac_config);
  esp_eth_phy_t *phy = esp_eth_phy_new_w5500(&phy_config);
#endif  // CONFIG_SNAPCLIENT_USE_W5500
  // Init Ethernet driver to default and install it
  esp_eth_handle_t eth_handle = NULL;
  esp_eth_config_t eth_config_spi = ETH_DEFAULT_CONFIG(mac, phy);
  ESP_GOTO_ON_FALSE(
      esp_eth_driver_install(&eth_config_spi, &eth_handle) == ESP_OK, NULL, err,
      TAG, "SPI Ethernet driver install failed");

  // The SPI Ethernet module might not have a burned factory MAC address, we can
  // set it manually.
  if (spi_eth_module_config->mac_addr != NULL) {
    ESP_GOTO_ON_FALSE(esp_eth_ioctl(eth_handle, ETH_CMD_S_MAC_ADDR,
                                    spi_eth_module_config->mac_addr) == ESP_OK,
                      NULL, err, TAG, "SPI Ethernet MAC address config failed");
  }

  if (mac_out != NULL) {
    *mac_out = mac;
  }
  if (phy_out != NULL) {
    *phy_out = phy;
  }
  return eth_handle;
err:
  if (eth_handle != NULL) {
    esp_eth_driver_uninstall(eth_handle);
  }
  if (mac != NULL) {
    mac->del(mac);
  }
  if (phy != NULL) {
    phy->del(phy);
  }
  return ret;
}
#endif  // CONFIG_SNAPCLIENT_USE_SPI_ETHERNET

/**
 */
static esp_err_t eth_init(esp_eth_handle_t *eth_handles_out[],
                          uint8_t *eth_cnt_out) {
  esp_err_t ret = ESP_OK;
  esp_eth_handle_t *eth_handles = NULL;
  uint8_t eth_cnt = 0;

#if CONFIG_SNAPCLIENT_USE_INTERNAL_ETHERNET || \
    CONFIG_SNAPCLIENT_USE_SPI_ETHERNET
  ESP_GOTO_ON_FALSE(
      eth_handles_out != NULL && eth_cnt_out != NULL, ESP_ERR_INVALID_ARG, err,
      TAG,
      "invalid arguments: initialized handles array or number of interfaces");
  eth_handles = calloc(SPI_ETHERNETS_NUM + INTERNAL_ETHERNETS_NUM,
                       sizeof(esp_eth_handle_t));
  ESP_GOTO_ON_FALSE(eth_handles != NULL, ESP_ERR_NO_MEM, err, TAG, "no memory");

#if CONFIG_SNAPCLIENT_USE_INTERNAL_ETHERNET
  eth_handles[eth_cnt] = eth_init_internal(NULL, NULL);
  ESP_GOTO_ON_FALSE(eth_handles[eth_cnt], ESP_FAIL, err, TAG,
                    "internal Ethernet init failed");
  eth_cnt++;
#endif  // CONFIG_SNAPCLIENT_USE_INTERNAL_ETHERNET

#if CONFIG_SNAPCLIENT_USE_SPI_ETHERNET
  ESP_GOTO_ON_ERROR(spi_bus_init(), err, TAG, "SPI bus init failed");
  // Init specific SPI Ethernet module configuration from Kconfig (CS GPIO,
  // Interrupt GPIO, etc.)
  spi_eth_module_config_t
      spi_eth_module_config[CONFIG_SNAPCLIENT_SPI_ETHERNETS_NUM] = {0};
  INIT_SPI_ETH_MODULE_CONFIG(spi_eth_module_config, 0);
  // The SPI Ethernet module(s) might not have a burned factory MAC address,
  // hence use manually configured address(es). In this example, Locally
  // Administered MAC address derived from ESP32x base MAC address is used. Note
  // that Locally Administered OUI range should be used only when testing on a
  // LAN under your control!
  uint8_t base_mac_addr[ETH_ADDR_LEN];
  ESP_GOTO_ON_ERROR(esp_efuse_mac_get_default(base_mac_addr), err, TAG,
                    "get EFUSE MAC failed");
  uint8_t local_mac_1[ETH_ADDR_LEN];
  esp_derive_local_mac(local_mac_1, base_mac_addr);
  spi_eth_module_config[0].mac_addr = local_mac_1;
#if CONFIG_SNAPCLIENT_SPI_ETHERNETS_NUM > 1
  INIT_SPI_ETH_MODULE_CONFIG(spi_eth_module_config, 1);
  uint8_t local_mac_2[ETH_ADDR_LEN];
  base_mac_addr[ETH_ADDR_LEN - 1] += 1;
  esp_derive_local_mac(local_mac_2, base_mac_addr);
  spi_eth_module_config[1].mac_addr = local_mac_2;
#endif
#if CONFIG_SNAPCLIENT_SPI_ETHERNETS_NUM > 2
#error Maximum number of supported SPI Ethernet devices is currently limited to 2 by this example.
#endif
  for (int i = 0; i < CONFIG_SNAPCLIENT_SPI_ETHERNETS_NUM; i++) {
    eth_handles[eth_cnt] = eth_init_spi(&spi_eth_module_config[i], NULL, NULL);
    ESP_GOTO_ON_FALSE(eth_handles[eth_cnt], ESP_FAIL, err, TAG,
                      "SPI Ethernet init failed");
    eth_cnt++;
  }
#endif  // CONFIG_ETH_USE_SPI_ETHERNET
#else
  ESP_LOGD(TAG, "no Ethernet device selected to init");
#endif  // CONFIG_SNAPCLIENT_USE_INTERNAL_ETHERNET ||
        // CONFIG_SNAPCLIENT_USE_SPI_ETHERNET
  *eth_handles_out = eth_handles;
  *eth_cnt_out = eth_cnt;

  return ret;
#if CONFIG_SNAPCLIENT_USE_INTERNAL_ETHERNET || \
    CONFIG_SNAPCLIENT_USE_SPI_ETHERNET
err:
  // Clean up any successfully created drivers before freeing handles
  if (eth_handles) {
    for (int i = 0; i < eth_cnt; i++) {
      if (eth_handles[i]) {
        esp_eth_stop(eth_handles[i]);
        esp_eth_driver_uninstall(eth_handles[i]);
      }
    }
    free(eth_handles);
  }
  return ret;
#endif
}

/* ============ Gateway Ping Check ============ */

static void ping_on_success(esp_ping_handle_t hdl, void *args) {
  ping_success = true;
  xSemaphoreGive(ping_done_sem);
}

static void ping_on_timeout(esp_ping_handle_t hdl, void *args) {
  // Don't signal yet - let it try all attempts
}

static void ping_on_end(esp_ping_handle_t hdl, void *args) {
  if (!ping_success) {
    xSemaphoreGive(ping_done_sem);  // Signal failure after all retries
  }
}

/**
 * @brief Check if gateway is reachable via ICMP ping
 * @param netif The network interface to check
 * @return true if gateway responds to ping, false otherwise
 */
static bool eth_check_gateway_reachable(esp_netif_t *netif) {
  esp_netif_ip_info_t ip;
  if (esp_netif_get_ip_info(netif, &ip) != ESP_OK || ip.gw.addr == 0) {
    ESP_LOGW(TAG, "No gateway configured, skipping reachability check");
    return true;  // No gateway to check - assume OK
  }

  // Semaphore should be created in eth_start(), but check defensively
  if (!ping_done_sem) {
    ESP_LOGE(TAG, "Ping semaphore not initialized");
    return false;
  }
  ping_success = false;

  esp_ping_config_t ping_config = ESP_PING_DEFAULT_CONFIG();
  ping_config.target_addr.u_addr.ip4.addr = ip.gw.addr;
  ping_config.target_addr.type = ESP_IPADDR_TYPE_V4;
  ping_config.count = 3;           // 3 attempts
  ping_config.timeout_ms = 1000;   // 1 second per attempt
  ping_config.interface = esp_netif_get_netif_impl_index(netif);

  esp_ping_callbacks_t cbs = {
      .on_ping_success = ping_on_success,
      .on_ping_timeout = ping_on_timeout,
      .on_ping_end = ping_on_end,
  };

  esp_ping_handle_t ping;
  if (esp_ping_new_session(&ping_config, &cbs, &ping) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create ping session");
    return false;
  }

  esp_ping_start(ping);

  // Wait for ping to complete (max 5 seconds)
  if (xSemaphoreTake(ping_done_sem, pdMS_TO_TICKS(5000)) != pdTRUE) {
    ESP_LOGW(TAG, "Ping timed out, forcing stop");
    ping_success = false;
  }

  // Stop ping and wait for callbacks to complete before deleting session
  // This prevents use-after-free if callbacks fire after session deletion
  esp_ping_stop(ping);
  vTaskDelay(pdMS_TO_TICKS(100));  // Allow pending callbacks to complete
  esp_ping_delete_session(ping);

  if (ping_success) {
    ESP_LOGI(TAG, "Gateway " IPSTR " is reachable", IP2STR(&ip.gw));
  } else {
    ESP_LOGW(TAG, "Gateway " IPSTR " not reachable", IP2STR(&ip.gw));
  }

  return ping_success;
}

/**
 * @brief Apply static IP configuration from settings
 *
 * LOCKING CONTRACT: This function acquires connIpSemaphoreHandle internally
 * at the end to update connection state. Caller MUST NOT hold the semaphore
 * when calling this function to avoid deadlock.
 *
 * @param netif The network interface to configure
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if config invalid
 */
static esp_err_t eth_apply_static_ip(esp_netif_t *netif) {
  char ip_str[16] = {0};
  char netmask_str[16] = {0};
  char gw_str[16] = {0};
  char dns_str[16] = {0};

  settings_get_eth_static_ip(ip_str, sizeof(ip_str));
  settings_get_eth_netmask(netmask_str, sizeof(netmask_str));
  settings_get_eth_gateway(gw_str, sizeof(gw_str));
  settings_get_eth_dns(dns_str, sizeof(dns_str));

  // Validate required fields
  if (ip_str[0] == '\0') {
    ESP_LOGW(TAG, "Static IP not configured, falling back to DHCP");
    return ESP_ERR_INVALID_ARG;
  }

  esp_netif_ip_info_t static_ip_info = {0};

  // Parse IP addresses
  if (inet_pton(AF_INET, ip_str, &static_ip_info.ip) != 1) {
    ESP_LOGE(TAG, "Invalid static IP: %s", ip_str);
    return ESP_ERR_INVALID_ARG;
  }

  if (netmask_str[0] != '\0') {
    if (inet_pton(AF_INET, netmask_str, &static_ip_info.netmask) != 1) {
      ESP_LOGE(TAG, "Invalid netmask: %s", netmask_str);
      return ESP_ERR_INVALID_ARG;
    }
  } else {
    // Default netmask
    inet_pton(AF_INET, "255.255.255.0", &static_ip_info.netmask);
  }

  if (gw_str[0] != '\0') {
    if (inet_pton(AF_INET, gw_str, &static_ip_info.gw) != 1) {
      ESP_LOGE(TAG, "Invalid gateway: %s", gw_str);
      return ESP_ERR_INVALID_ARG;
    }
  }

  // Stop DHCP client before setting static IP
  esp_err_t dhcp_err = esp_netif_dhcpc_stop(netif);
  if (dhcp_err != ESP_OK && dhcp_err != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED) {
    ESP_LOGD(TAG, "DHCP stop returned: %s (continuing)", esp_err_to_name(dhcp_err));
  }

  // Apply static IP configuration
  esp_err_t err = esp_netif_set_ip_info(netif, &static_ip_info);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set static IP: %s", esp_err_to_name(err));
    // Re-enable DHCP on failure
    esp_netif_dhcpc_start(netif);
    return err;
  }

  ESP_LOGI(TAG, "Static IP configured: " IPSTR, IP2STR(&static_ip_info.ip));
  ESP_LOGI(TAG, "Netmask: " IPSTR, IP2STR(&static_ip_info.netmask));
  ESP_LOGI(TAG, "Gateway: " IPSTR, IP2STR(&static_ip_info.gw));

  // Set DNS if configured
  if (dns_str[0] != '\0') {
    esp_netif_dns_info_t dns_info = {0};
    if (inet_pton(AF_INET, dns_str, &dns_info.ip.u_addr.ip4) == 1) {
      dns_info.ip.type = ESP_IPADDR_TYPE_V4;
      esp_netif_set_dns_info(netif, ESP_NETIF_DNS_MAIN, &dns_info);
      ESP_LOGI(TAG, "DNS: %s", dns_str);
    }
  }

  // Manually update the connection state since esp_netif_set_ip_info()
  // does NOT trigger IP_EVENT_ETH_GOT_IP
  xSemaphoreTake(connIpSemaphoreHandle, portMAX_DELAY);
  memcpy(&ip_info, &static_ip_info, sizeof(esp_netif_ip_info_t));
  connected = true;
  xSemaphoreGive(connIpSemaphoreHandle);

  return ESP_OK;
}

/**
 * @brief Unified takeover checkpoint - called from ALL IP acquisition paths (Fix 1)
 *
 * Checks if conditions are met for Ethernet takeover and performs it atomically.
 * This ensures consistent behavior whether IP was acquired via DHCP or static config.
 *
 * @param netif The Ethernet network interface that now has an IP
 */
static void eth_check_and_apply_takeover(esp_netif_t *netif) {
  bool do_takeover = false;

  xSemaphoreTake(connIpSemaphoreHandle, portMAX_DELAY);
  if (want_eth_takeover && !we_changed_default_netif && !playerstarted) {
    do_takeover = true;
    want_eth_takeover = false;
    // Don't set we_changed_default_netif until after successful netif change
  }
  xSemaphoreGive(connIpSemaphoreHandle);

  if (do_takeover) {
    ESP_LOGI(TAG, "Ethernet takeover: setting default netif to ETH");
    esp_err_t err = esp_netif_set_default_netif(netif);
    if (err == ESP_OK) {
      xSemaphoreTake(connIpSemaphoreHandle, portMAX_DELAY);
      we_changed_default_netif = true;
      xSemaphoreGive(connIpSemaphoreHandle);
      app_request_reconnect();
    } else {
      ESP_LOGE(TAG, "Failed to set default netif: %s", esp_err_to_name(err));
      // Restore takeover intent so it can be retried
      xSemaphoreTake(connIpSemaphoreHandle, portMAX_DELAY);
      want_eth_takeover = true;
      xSemaphoreGive(connIpSemaphoreHandle);
    }
  } else if (want_eth_takeover && playerstarted) {
    ESP_LOGI(TAG, "Playback active; deferring Ethernet takeover until playback stops");
  }
}

/**
 * @brief Background task for static IP configuration
 *
 * Moves blocking static IP operations out of the event handler to prevent
 * blocking other Ethernet events. The task handles:
 * - Link stabilization delay
 * - Static IP application
 * - Gateway reachability check
 * - Takeover coordination
 *
 * CRITICAL: Uses static_ip_netif (protected by semaphore) instead of task
 * parameter to avoid use-after-free if netif is invalidated during delays.
 *
 * @param pvParameters Unused (netif obtained from protected static variable)
 */
static void static_ip_task(void *pvParameters) {
  (void)pvParameters;  // Unused - we use protected static_ip_netif instead
  esp_netif_t *netif = NULL;

  ESP_LOGI(TAG, "Static IP task started");

  // Get netif from protected variable and check if we should abort
  xSemaphoreTake(connIpSemaphoreHandle, portMAX_DELAY);
  if (!static_ip_in_progress || !static_ip_netif) {
    ESP_LOGW(TAG, "Static IP task: aborted (flag cleared or no netif)");
    static_ip_task_handle = NULL;
    xSemaphoreGive(connIpSemaphoreHandle);
    vTaskDelete(NULL);
    return;
  }
  netif = static_ip_netif;
  xSemaphoreGive(connIpSemaphoreHandle);

  // Wait for link to stabilize
  vTaskDelay(pdMS_TO_TICKS(500));

  // Check again if we should continue (cable might have been unplugged)
  xSemaphoreTake(connIpSemaphoreHandle, portMAX_DELAY);
  if (!static_ip_in_progress || static_ip_netif != netif) {
    ESP_LOGW(TAG, "Static IP task: aborted after link delay");
    static_ip_task_handle = NULL;
    xSemaphoreGive(connIpSemaphoreHandle);
    vTaskDelete(NULL);
    return;
  }
  xSemaphoreGive(connIpSemaphoreHandle);

  // Apply static IP configuration
  esp_err_t result = eth_apply_static_ip(netif);

  if (result == ESP_OK) {
    // Give time for IP to be applied before checking gateway
    vTaskDelay(pdMS_TO_TICKS(500));

    // Check if still valid
    xSemaphoreTake(connIpSemaphoreHandle, portMAX_DELAY);
    bool still_valid = static_ip_in_progress && connected && (static_ip_netif == netif);
    xSemaphoreGive(connIpSemaphoreHandle);

    if (!still_valid) {
      ESP_LOGW(TAG, "Static IP task: aborted after IP apply");
      xSemaphoreTake(connIpSemaphoreHandle, portMAX_DELAY);
      static_ip_in_progress = false;
      static_ip_task_handle = NULL;
      xSemaphoreGive(connIpSemaphoreHandle);
      vTaskDelete(NULL);
      return;
    }

    // Check gateway reachability
    if (!eth_check_gateway_reachable(netif)) {
      ESP_LOGW(TAG, "Static IP failed gateway check, falling back to DHCP");

      // Check if still connected before starting DHCP (prevents race with disconnect)
      xSemaphoreTake(connIpSemaphoreHandle, portMAX_DELAY);
      bool still_connected = (static_ip_netif == netif);  // netif still valid
      connected = false;
      static_ip_in_progress = false;
      static_ip_task_handle = NULL;
      xSemaphoreGive(connIpSemaphoreHandle);

      if (still_connected) {
        // Start DHCP - GOT_IP event will handle takeover
        esp_netif_dhcpc_start(netif);
      } else {
        ESP_LOGW(TAG, "Ethernet disconnected, skipping DHCP fallback");
      }
    } else {
      // Static IP succeeded - apply takeover using unified checkpoint
      xSemaphoreTake(connIpSemaphoreHandle, portMAX_DELAY);
      static_ip_in_progress = false;
      static_ip_task_handle = NULL;
      xSemaphoreGive(connIpSemaphoreHandle);

      eth_check_and_apply_takeover(netif);
      ESP_LOGI(TAG, "Static IP configuration complete");
    }
  } else {
    // Static IP configuration failed, DHCP should already be running
    ESP_LOGW(TAG, "Static IP configuration failed, using DHCP");
    xSemaphoreTake(connIpSemaphoreHandle, portMAX_DELAY);
    static_ip_in_progress = false;
    static_ip_task_handle = NULL;
    xSemaphoreGive(connIpSemaphoreHandle);
  }

  vTaskDelete(NULL);
}

/** Event handler for Ethernet events */
static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data) {
  uint8_t mac_addr[6] = {0};
  /* we can get the ethernet driver handle from event data */
  esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;
  esp_netif_t *netif = (esp_netif_t *)arg;

  switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
      esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
      ESP_LOGI(TAG, "Ethernet Link Up");
      ESP_LOGI(TAG, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
               mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4],
               mac_addr[5]);

      esp_err_t ipv6_err = esp_netif_create_ip6_linklocal(netif);
      if (ipv6_err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to create IPv6 link-local: %s (continuing)", esp_err_to_name(ipv6_err));
      }

      // Check if WiFi is currently up - if so, plan to prefer Ethernet once
      // Ethernet has acquired an IP (after DHCP or static IP is applied).
      esp_netif_t *sta_netif = network_get_netif_from_desc(NETWORK_INTERFACE_DESC_STA);
      if (sta_netif && network_has_ip(sta_netif)) {
        xSemaphoreTake(connIpSemaphoreHandle, portMAX_DELAY);
        want_eth_takeover = true;
        xSemaphoreGive(connIpSemaphoreHandle);
        ESP_LOGI(TAG, "Ethernet present and WiFi active; will prefer Ethernet after IP acquired");
      }

      // Handle static IP mode (spawn task instead of blocking)
      if (current_eth_mode == 2) {  // Static
        xSemaphoreTake(connIpSemaphoreHandle, portMAX_DELAY);

        // Kill any existing static IP task before starting a new one
        if (static_ip_task_handle != NULL) {
          ESP_LOGW(TAG, "Aborting previous static IP task");
          vTaskDelete(static_ip_task_handle);
          static_ip_task_handle = NULL;
        }

        // Check if playback is active - defer if so
        if (playerstarted) {
          ESP_LOGI(TAG, "Playback active; deferring static IP until playback stops");
          static_ip_pending = true;
          static_ip_netif = netif;
          static_ip_in_progress = false;
          xSemaphoreGive(connIpSemaphoreHandle);
          break;
        }

        // Store netif in protected variable BEFORE creating task
        static_ip_netif = netif;
        static_ip_in_progress = true;
        static_ip_pending = false;
        xSemaphoreGive(connIpSemaphoreHandle);

        // Spawn task to handle static IP in background (doesn't block event handler)
        BaseType_t task_created = xTaskCreate(
            static_ip_task,
            "eth_static_ip",
            4096,
            NULL,  // Task uses protected static_ip_netif instead
            5,
            &static_ip_task_handle
        );

        if (task_created != pdPASS) {
          ESP_LOGE(TAG, "Failed to create static IP task, falling back to DHCP");
          xSemaphoreTake(connIpSemaphoreHandle, portMAX_DELAY);
          static_ip_in_progress = false;
          static_ip_netif = NULL;
          static_ip_task_handle = NULL;
          xSemaphoreGive(connIpSemaphoreHandle);
          // Explicitly start DHCP as fallback
          esp_err_t dhcp_err = esp_netif_dhcpc_start(netif);
          if (dhcp_err != ESP_OK && dhcp_err != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STARTED) {
            ESP_LOGE(TAG, "Failed to start DHCP fallback: %s", esp_err_to_name(dhcp_err));
          }
        }
      }
      // DHCP mode (current_eth_mode == 1): takeover will be handled in got_ip_event_handler

      break;
    case ETHERNET_EVENT_DISCONNECTED:
      // Defensive check - semaphore should be created in eth_start()
      if (!connIpSemaphoreHandle) {
        ESP_LOGE(TAG, "Semaphore not initialized in disconnect handler");
        break;
      }
      xSemaphoreTake(connIpSemaphoreHandle, portMAX_DELAY);
      connected = false;

      // Kill any running static IP task immediately
      if (static_ip_task_handle != NULL) {
        ESP_LOGI(TAG, "Killing static IP task on disconnect");
        vTaskDelete(static_ip_task_handle);
        static_ip_task_handle = NULL;
      }

      // Reset static IP state guards on disconnect
      static_ip_in_progress = false;
      static_ip_pending = false;
      static_ip_netif = NULL;

      // Stop any running DHCP client to avoid confusion
      esp_err_t dhcp_err = esp_netif_dhcpc_stop(netif);
      if (dhcp_err != ESP_OK && dhcp_err != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED) {
        ESP_LOGD(TAG, "DHCP stop returned: %s", esp_err_to_name(dhcp_err));
      }

      /* If we previously changed the default netif to prefer Ethernet, reset
       * the flag and trigger a reconnect so the system falls back to WiFi.
       */
      if (we_changed_default_netif) {
        ESP_LOGI(TAG, "Ethernet disconnected; triggering WiFi fallback");
        we_changed_default_netif = false;
        want_eth_takeover = false;
        xSemaphoreGive(connIpSemaphoreHandle);
        /* Request reconnect so main re-evaluates network and uses WiFi */
        app_request_reconnect();
      } else {
        want_eth_takeover = false;  // Clear any pending takeover intent
        xSemaphoreGive(connIpSemaphoreHandle);
      }

      ESP_LOGI(TAG, "Ethernet Link Down");
      break;
    case ETHERNET_EVENT_START:
      ESP_LOGI(TAG, "Ethernet Started");
      break;
    case ETHERNET_EVENT_STOP:
      ESP_LOGI(TAG, "Ethernet Stopped");
      break;
    default:
      break;
  }
}

/** Event handler for IP_EVENT_ETH_LOST_IP */
static void lost_ip_event_handler(void *arg, esp_event_base_t event_base,
                                  int32_t event_id, void *event_data) {
  ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;

  for (int i = 0; i < eth_port_cnt; i++) {
    char if_desc_str[32];  // Larger buffer to prevent overflow
    snprintf(if_desc_str, sizeof(if_desc_str), "%s%d", NETWORK_INTERFACE_DESC_ETH, i);

    if (network_is_our_netif(if_desc_str, event->esp_netif)) {
      ESP_LOGI(TAG, "Ethernet Lost IP Address");

      xSemaphoreTake(connIpSemaphoreHandle, portMAX_DELAY);
      memcpy((void *)&ip_info, (const void *)&event->ip_info,
             sizeof(esp_netif_ip_info_t));
      connected = false;
      xSemaphoreGive(connIpSemaphoreHandle);

      break;
    }
  }
}

/** Event handler for IP_EVENT_ETH_GOT_IP */
static void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data) {
  ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;

  for (int i = 0; i < eth_port_cnt; i++) {
    char if_desc_str[32];  // Larger buffer to prevent overflow
    snprintf(if_desc_str, sizeof(if_desc_str), "%s%d", NETWORK_INTERFACE_DESC_ETH, i);

    if (network_is_our_netif(if_desc_str, event->esp_netif)) {
      xSemaphoreTake(connIpSemaphoreHandle, portMAX_DELAY);

      memcpy((void *)&ip_info, (const void *)&event->ip_info,
             sizeof(esp_netif_ip_info_t));
      connected = true;

      ESP_LOGI(TAG, "Ethernet Got IP Address");
      ESP_LOGI(TAG, "~~~~~~~~~~~");
      ESP_LOGI(TAG, "ETHIP:" IPSTR, IP2STR(&ip_info.ip));
      ESP_LOGI(TAG, "ETHMASK:" IPSTR, IP2STR(&ip_info.netmask));
      ESP_LOGI(TAG, "ETHGW:" IPSTR, IP2STR(&ip_info.gw));
      ESP_LOGI(TAG, "~~~~~~~~~~~");

      xSemaphoreGive(connIpSemaphoreHandle);

      /* Use unified takeover checkpoint (Fix 1) - handles playback check internally */
      eth_check_and_apply_takeover(event->esp_netif);

      break;
    }
  }
}

/**
 */
bool eth_get_ip(esp_netif_ip_info_t *ip) {
  xSemaphoreTake(connIpSemaphoreHandle, portMAX_DELAY);

  if (ip) {
    memcpy((void *)ip, (const void *)&ip_info, sizeof(esp_netif_ip_info_t));
  }
  bool _connected = connected;

  xSemaphoreGive(connIpSemaphoreHandle);

  return _connected;
}

/* Called by player code when playback stops so we can complete a pending
 * Ethernet takeover that was delayed during active playback.
 */
void eth_on_playback_stopped(void) {
  // Defensive check - semaphore should be created in eth_start()
  if (!connIpSemaphoreHandle) {
    ESP_LOGD(TAG, "eth_on_playback_stopped: semaphore not initialized (Ethernet disabled?)");
    return;
  }

  bool do_takeover = false;
  bool do_static_ip = false;
  esp_netif_t *pending_netif = NULL;

  xSemaphoreTake(connIpSemaphoreHandle, portMAX_DELAY);

  // Fix 4: Check for pending static IP configuration first
  if (static_ip_pending && static_ip_netif && !static_ip_in_progress) {
    do_static_ip = true;
    pending_netif = static_ip_netif;
    static_ip_pending = false;
    static_ip_in_progress = true;
  }
  // Check for pending takeover (DHCP path or already-configured static IP)
  else if (want_eth_takeover && connected && !we_changed_default_netif) {
    do_takeover = true;
    want_eth_takeover = false;
  }

  xSemaphoreGive(connIpSemaphoreHandle);

  // Handle pending static IP configuration
  if (do_static_ip) {
    ESP_LOGI(TAG, "Playback stopped: starting deferred static IP configuration");
    BaseType_t task_created = xTaskCreate(
        static_ip_task,
        "eth_static_ip",
        4096,
        NULL,  // Task uses protected static_ip_netif instead
        5,
        &static_ip_task_handle
    );

    if (task_created != pdPASS) {
      ESP_LOGE(TAG, "Failed to create deferred static IP task, falling back to DHCP");
      xSemaphoreTake(connIpSemaphoreHandle, portMAX_DELAY);
      static_ip_in_progress = false;
      static_ip_task_handle = NULL;
      xSemaphoreGive(connIpSemaphoreHandle);
      // Explicitly start DHCP as fallback
      if (pending_netif) {
        esp_err_t dhcp_err = esp_netif_dhcpc_start(pending_netif);
        if (dhcp_err != ESP_OK && dhcp_err != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STARTED) {
          ESP_LOGE(TAG, "Failed to start DHCP fallback: %s", esp_err_to_name(dhcp_err));
        }
      }
    }
    return;
  }

  // Handle pending takeover
  if (do_takeover) {
    ESP_LOGI(TAG, "Playback stopped: performing pending Ethernet takeover");
    esp_netif_t *eth_netif = network_get_netif_from_desc(NETWORK_INTERFACE_DESC_ETH);
    if (eth_netif) {
      esp_err_t err = esp_netif_set_default_netif(eth_netif);
      if (err == ESP_OK) {
        xSemaphoreTake(connIpSemaphoreHandle, portMAX_DELAY);
        we_changed_default_netif = true;
        xSemaphoreGive(connIpSemaphoreHandle);
        app_request_reconnect();
      } else {
        ESP_LOGE(TAG, "Failed to set default netif: %s", esp_err_to_name(err));
      }
    } else {
      ESP_LOGW(TAG, "Playback-stopped takeover: ETH netif not found");
    }
  }
}

static void eth_on_got_ipv6(void *arg, esp_event_base_t event_base,
                            int32_t event_id, void *event_data) {
  ip_event_got_ip6_t *event = (ip_event_got_ip6_t *)event_data;
  if (!network_is_our_netif(NETWORK_INTERFACE_DESC_ETH, event->esp_netif)) {
    return;
  }
  esp_ip6_addr_type_t ipv6_type =
      esp_netif_ip6_get_addr_type(&event->ip6_info.ip);
  ESP_LOGI(TAG,
           "Got IPv6 event: Interface \"%s\" address: " IPV6STR ", type: %s",
           esp_netif_get_desc(event->esp_netif), IPV62STR(event->ip6_info.ip),
           ipv6_addr_types_to_str[ipv6_type]);
}

/** Init function that exposes to the main application */
void eth_start(void) {
  // Initialize semaphores first (needed even if Ethernet is disabled)
  if (!connIpSemaphoreHandle) {
    connIpSemaphoreHandle = xSemaphoreCreateMutex();
  }
  // Create ping semaphore once here to avoid leak from repeated creation
  if (!ping_done_sem) {
    ping_done_sem = xSemaphoreCreateBinary();
  }

  // Check Ethernet mode from settings
  settings_get_eth_mode(&current_eth_mode);
  ESP_LOGI(TAG, "Ethernet mode: %ld (%s)", (long)current_eth_mode,
           current_eth_mode == 0 ? "Disabled" :
           current_eth_mode == 1 ? "DHCP" : "Static");

  // If Ethernet is disabled, skip all initialization
  if (current_eth_mode == 0) {
    ESP_LOGI(TAG, "Ethernet disabled by configuration");
    return;
  }

  // Initialize Ethernet driver
  esp_eth_handle_t *eth_handles;
  esp_err_t ret = eth_init(&eth_handles, &eth_port_cnt);
  if (ret != ESP_OK || eth_port_cnt == 0) {
    ESP_LOGE(TAG, "Ethernet driver init failed: %s", esp_err_to_name(ret));
    eth_auto_disable_and_persist();
    return;
  }

#if CONFIG_SNAPCLIENT_USE_INTERNAL_ETHERNET || CONFIG_SNAPCLIENT_USE_SPI_ETHERNET
  esp_netif_t *eth_netif = NULL;

  // Create instance(s) of esp-netif for Ethernet(s)
  if (eth_port_cnt == 1) {
    // Use ESP_NETIF_DEFAULT_ETH when just one Ethernet interface is used and
    // you don't need to modify default esp-netif configuration parameters.
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    eth_netif = esp_netif_new(&cfg);
    if (!eth_netif) {
      ESP_LOGE(TAG, "Failed to create Ethernet netif");
      eth_cleanup_drivers(eth_handles, eth_port_cnt);
      eth_auto_disable_and_persist();
      return;
    }

    esp_eth_netif_glue_handle_t glue = esp_eth_new_netif_glue(eth_handles[0]);
    if (!glue) {
      ESP_LOGE(TAG, "Failed to create netif glue");
      esp_netif_destroy(eth_netif);
      eth_cleanup_drivers(eth_handles, eth_port_cnt);
      eth_auto_disable_and_persist();
      return;
    }

    ret = esp_netif_attach(eth_netif, glue);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to attach Ethernet to TCP/IP stack: %s", esp_err_to_name(ret));
      esp_eth_del_netif_glue(glue);
      esp_netif_destroy(eth_netif);
      eth_cleanup_drivers(eth_handles, eth_port_cnt);
      eth_auto_disable_and_persist();
      return;
    }
  } else {
    // Use ESP_NETIF_INHERENT_DEFAULT_ETH when multiple Ethernet interfaces are
    // used and so you need to modify esp-netif configuration parameters for
    // each interface (name, priority, etc.).
    esp_netif_inherent_config_t esp_netif_config =
        ESP_NETIF_INHERENT_DEFAULT_ETH();
    esp_netif_config_t cfg_spi = {.base = &esp_netif_config,
                                  .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH};
    char if_key_str[32];   // Larger buffer to prevent overflow
    char if_desc_str[32];  // Larger buffer to prevent overflow

    // Track created netifs for cleanup on partial failure
    esp_netif_t *created_netifs[SPI_ETHERNETS_NUM + INTERNAL_ETHERNETS_NUM];
    memset(created_netifs, 0, sizeof(created_netifs));

    for (int i = 0; i < eth_port_cnt; i++) {
      snprintf(if_key_str, sizeof(if_key_str), "ETH_%d", i);
      snprintf(if_desc_str, sizeof(if_desc_str), "%s%d", NETWORK_INTERFACE_DESC_ETH, i);
      esp_netif_config.if_key = if_key_str;
      esp_netif_config.if_desc = if_desc_str;
      esp_netif_config.route_prio -= i * 5;
      eth_netif = esp_netif_new(&cfg_spi);

      if (!eth_netif) {
        ESP_LOGE(TAG, "Failed to create Ethernet netif %d", i);
        // Cleanup previously created netifs
        for (int j = 0; j < i; j++) {
          if (created_netifs[j]) esp_netif_destroy(created_netifs[j]);
        }
        eth_cleanup_drivers(eth_handles, eth_port_cnt);
        eth_auto_disable_and_persist();
        return;
      }
      created_netifs[i] = eth_netif;

      esp_eth_netif_glue_handle_t glue = esp_eth_new_netif_glue(eth_handles[i]);
      if (!glue) {
        ESP_LOGE(TAG, "Failed to create netif glue %d", i);
        // Cleanup all created netifs including current
        for (int j = 0; j <= i; j++) {
          if (created_netifs[j]) esp_netif_destroy(created_netifs[j]);
        }
        eth_cleanup_drivers(eth_handles, eth_port_cnt);
        eth_auto_disable_and_persist();
        return;
      }

      ret = esp_netif_attach(eth_netif, glue);
      if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to attach Ethernet %d: %s", i, esp_err_to_name(ret));
        esp_eth_del_netif_glue(glue);
        // Cleanup all created netifs including current
        for (int j = 0; j <= i; j++) {
          if (created_netifs[j]) esp_netif_destroy(created_netifs[j]);
        }
        eth_cleanup_drivers(eth_handles, eth_port_cnt);
        eth_auto_disable_and_persist();
        return;
      }
    }
  }

  // Register event handlers - non-fatal if these fail
  ret = esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID,
                                   &eth_event_handler, eth_netif);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to register ETH event handler: %s (continuing)", esp_err_to_name(ret));
  }

  ret = esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP,
                                   &got_ip_event_handler, NULL);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to register got_ip handler: %s (continuing)", esp_err_to_name(ret));
  }

  ret = esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_LOST_IP,
                                   &lost_ip_event_handler, NULL);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to register lost_ip handler: %s (continuing)", esp_err_to_name(ret));
  }

  ret = esp_event_handler_register(IP_EVENT, IP_EVENT_GOT_IP6,
                                   &eth_on_got_ipv6, NULL);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to register IPv6 handler: %s (continuing)", esp_err_to_name(ret));
  }

  // Start Ethernet driver state machine - non-fatal, may recover when cable plugged in
  for (int i = 0; i < eth_port_cnt; i++) {
    ret = esp_eth_start(eth_handles[i]);
    if (ret != ESP_OK) {
      ESP_LOGW(TAG, "Failed to start Ethernet %d: %s (may recover on cable connect)",
               i, esp_err_to_name(ret));
    }
  }

  ESP_LOGI(TAG, "Ethernet initialization complete");
#endif
}
