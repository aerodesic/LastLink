/*
 * sensors.h
 */
#ifndef __sensors_h_included
#define __sensors_h_included

typedef enum {
   SENSOR_TYPE_INPUT = 0,            // Just a raw input device
   SENSOR_TYPE_INPUT_ACCUMULATOR,    // Each reading is added to previous to form value
   SENSOR_TYPE_OUTPUT,               // Just an outut.  Value is set and send to device
   SENSOR_TYPE_OUTPUT_TIMED,         // Output set but is counted down as seconds from it's initial value
   SENSOR_TYPE_VALUE,                // Value not connected to hardware.
} sensor_type_t;

typedef enum {
   SENSOR_READ=0,
   SENSOR_WRITE,
} sensor_transaction_t;

typedef struct sensor_cache sensor_cache_t;

/*
 * Called with:
 *    transaction     SENSOR_READ / SENSOR_WRITE / etc.
 *    name            name of sensor
 *    param           optional param
 *    reply_buffer    buffer to receive value / error message
 *    reply_len       length of reply buffer
 *    value           buffer with value (for a write) [ varargs, only present on write ]
 *
 * Returns TRUE if successful or FALSE with reply_buffer containing error message.
 */
typedef bool (sensor_function_t)(sensor_transaction_t transaction, const char* name, void* param, char* reply_buffer, size_t reply_len, ...);

/*
 *  register a new sensor
 *
 *  Entry:
 *      name            Name of sensor as reported and manipulated
 *      type            Type (input/output/timed/value, etc.)
 *      units           Units of sensor (if not NULL) [e.g. "seconds", "cc", "degC", etc.]
 *      notify_time     If > 0 then automatically broadcast sensor value every <notify_time> seconds.
 *                      If < 0, then automatically broadcast sensor when value changes
 *                      If == 0, then only send value when requested.
 *      function        Function to call to read/write sensor value.
 *      param           An additional parameter, if needed, passed to the read/write function.
 */
bool register_sensor(const char* name, sensor_type_t type, const char* units, int notify_time, sensor_function_t function, void* param);

/*
 * Deregister a sensor.
 */
bool deregister_sensor(const char* name);

bool init_sensors(void);

bool deinit_sensors(void);

#endif /* __sensors_h_included */

