/*
 * gps.h
 *
 */

#include "sdkconfig.h"

#ifdef CONFIG_LASTLINK_SENSORS_GPS_ENABLE

bool start_gps(void);
bool stop_gps(void);

#endif /* CONFIG_LASTLINK_SENSORS_GPS_ENABLE */
