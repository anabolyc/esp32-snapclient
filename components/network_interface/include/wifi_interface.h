#ifndef _WIFI_INTERFACE_H_
#define _WIFI_INTERFACE_H_

#include <stdbool.h>

#include "esp_netif.h"

// use wifi provisioning
#define ENABLE_WIFI_PROVISIONING CONFIG_ENABLE_WIFI_PROVISIONING

/* Hardcoded WiFi configuration that you can set via
   'make menuconfig'.
*/
#if !ENABLE_WIFI_PROVISIONING
#define WIFI_SSID CONFIG_WIFI_SSID
#define WIFI_PASSWORD CONFIG_WIFI_PASSWORD
#endif

#define WIFI_MAXIMUM_RETRY CONFIG_WIFI_MAXIMUM_RETRY

bool wifi_get_ip(esp_netif_ip_info_t *ip);
void wifi_start(void);

/**
 * Dynamically enable/disable WiFi power save.
 * Call with enable=false during audio playback for better throughput.
 * Call with enable=true when idle to save power.
 * Only effective if CONFIG_WIFI_DYNAMIC_POWER_SAVE is enabled.
 */
void wifi_set_power_save(bool enable);

#endif /* _WIFI_INTERFACE_H_ */
