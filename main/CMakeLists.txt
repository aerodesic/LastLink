idf_component_register(SRCS "commands.c" "configdata.c" "crc16.c" "duplicate_sequence.c" "expressions.c" "gps.c" "httpd_server.c"
                      "http_templates.c" "lastlink_main.c" "linklayer.c" "linklayer_io.c" "lsocket.c" "mdns_config.c" "network_connect.c"
                      "nmea_parser.c" "nvs_support.c" "os_specific.c" "packets.c" "packet_window.c" "power_manager.c" "routes.c" "sensors.c"
                      "service_names.c" "spiffs.c" "sx126x_driver.c" "sx127x_driver.c" "tokenize.c" "uuid.c" "varlist.c"
                      "vfs_lastlink.c"
                       INCLUDE_DIRS "include"
                       EMBED_TXTFILES "certs/cacert.pem" "certs/prvtkey.pem")
