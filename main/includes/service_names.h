/*
 * service_namess.h
 *
 * Advertised services.
 */
#ifndef __service_names_h_included
#define __serivce_names_h_included

#include "sdkconfig.h"

#include <stdbool.h>
#include "lsocket.h"

#if CONFIG_LASTLINK_SERVICE_NAMES_ENABLE

#define SERVICE_NAMES_WAIT_TIMEOUT     CONFIG_LASTLINK_SERVICE_NAMES_WAIT_TIMEOUT   /* Time to wait for new service request in scanner */
#define SERVICE_NAMES_REPLY_TIMEOUT    CONFIG_LASTLINK_SERVICE_NAMES_REPLY_TIMEOUT  /* Timeout waiting for a reply */
#define SERVICE_NAMES_REPLY_RETRIES    CONFIG_LASTLINK_SERVICE_NAMES_REPLY_RETRIES  /* Number of attempts for service lookup */
#define SERVICE_NAMES_DEFAULT_LIFETIME CONFIG_LASTLINK_SERVICE_NAMES_LIFETIME       /* Time before discard in ESTABLISHED state */
#define SERVICE_NAMES_MAX_LEN          CONFIG_LASTLINK_SERVICE_NAMES_MAX_LEN        /* Max length of a name */
#define SERVICE_NAMES_PORT             CONFIG_LASTLINK_SERVICE_NAMES_PORT           /* Port for service name request */

bool register_service(const char* name, int s, uint16_t lifetime);
bool deregister_service(const char* name);


ls_error_t find_service(const char* name, ls_address_t src_addr, ls_socket_type_t *socket_type, ls_port_t* src_port, ls_address_t* dest_addr, ls_port_t* dest_port);
bool init_service_names(void);
bool deinit_service_names(void);
int find_all_services(const char* name, ls_address_t *addresses, int num_addresses);

#endif /* CONFIG_LASTLINK_SERVICE_NAMES_ENABLE */

#endif /* __service_names_h_included */

