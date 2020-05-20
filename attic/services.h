/*
 * services.h
 *
 * Advertised services.
 */
#ifndef __services_h_included
#define __serivces_h_included

#include "sdkconfig.h"

#include <stdbool.h>
#include "lsocket.h"

#if CONFIG_LASTLINK_SERVICE_NAME_ENABLE
bool register_service(const char* name, ls_socket_type_t socket_type, ls_port_t port);
bool deregister_service(const char* name);

bool find_service_by_name(const char* name, int *address, ls_socket_type_t* socket_type, ls_port_t *port);

bool init_services(void);
bool deinit_services(void);

#endif /* CONFIG_LASTLINK_SERVICE_NAME_ENABLE */

#endif /* __services_h_included */

