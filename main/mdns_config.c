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

    mdns_service_add(NULL, "_https", "_tcp", 443, NULL, 0);
    mdns_service_instance_name_set("_https", "_tcp", "LastLink web server");

    mdns_txt_item_t serviceTxtData[3] = {
        {"board","{esp32}"},
        {"u","user"},
        {"p","password"}
    };

    //set txt data for service (will free and replace current data)
    mdns_service_txt_set("_https", "_tcp", serviceTxtData, 3);
}
