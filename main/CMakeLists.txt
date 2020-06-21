idf_component_register(SRCS "commands.c" "configdata.c" "lastlink_main.c" "linklayer.c" "linklayer_io.c"
                            "lsocket.c" "nvs_support.c" "os_freertos.c" "packets.c" "routes.c" "spiffs.c"
                            "sx126x_driver.c" "sx127x_driver.c" "vfs_lastlink.c" "crc16.c" "service_names.c" "packet_window.c" 
                            "httpd_server.c" "wifi_conenct.c"
                       INCLUDE_DIRS "include"
                       EMBED_TXTFILES "certs/cacert.pem" "certs/prvtkey.pem")
