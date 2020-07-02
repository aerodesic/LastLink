#include <mdns.h>
#include "mdns_config.h"

void start_mdns_service()
{
    //initialize mDNS service
    esp_err_t err = mdns_init();
    if (err) {
        printf("MDNS Init failed: %d\n", err);
        return;
    }

    //set hostname
    mdns_hostname_set("LastLink");

    //set default instance
    mdns_instance_name_set("Lastlink access node");

#ifdef CONFIG_LASTLINK_HTTPS_SERVER_ENABLED
    mdns_service_add(NULL, "_https", "_tcp", 443, NULL, 0);
    mdns_service_instance_name_set("_https", "_tcp", "LastLink web server");
#else
    mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
    mdns_service_instance_name_set("_http", "_tcp", "LastLink web server");
#endif

#ifdef NOTUSED
    mdns_txt_item_t serviceTxtData[3] = {
        {"board","{esp32}"},
        {"u","user"},
        {"p","password"}
    };

    //set txt data for service (will free and replace current data)
#ifdef CONFIG_LASTLINK_HTTPS_SERVER_ENABLED
    mdns_service_txt_set("_https", "_tcp", serviceTxtData, 3);
#else
    mdns_service_txt_set("_http", "_tcp", serviceTxtData, 3);
#endif
#endif
}

void stop_mdns_service(void)
{
    mdns_free();
}

