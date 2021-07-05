//
// dht driver
//

#ifndef __dht_h_included
#define __dht_h_included

// Error codes
typedef enum {
    DHT_OK = 0,    // Message received, checksum ok
    DHT_CHECKSUM,  // Checksum error
    DHT_TIMEOUT,   // Receive timed out
    DHT_ERROR,     // Responsded but not enough bits
} dht_ret_t;

typedef double dht_value_t;

dht_ret_t dht_read(dht_value_t* rh, dht_value_t* temperature);

#endif // __dht_included
