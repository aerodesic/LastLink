/*
 * lastlink.c
 *
 * This module contains the top-level LastLink packetizer and distributor.
 *
 * It draws upon a lower-level packet serializer that can be one of several
 * devices (currently SX127x) interfaced via SPI.
 */

/*
 * Define the GPIO pins used for connection to the SX127x
 */
#define SX127x_DIO0                    CONFIG_LASTLINK_GPIO_DIO0   /* DIO0 interrupt pin */
#define SX127x_DIO1                    CONFIG_LASTLINK_GPIO_DIO1   /* DIO1 interrupt pin */
#define SX127x_DIO2                    CONFIG_LASTLINK_GPIO_DIO2   /* DIO2 interrupt pin */
#define SX127x_SCK                     CONFIG_LASTLINK_GPIO_SCK    /* SPI Clock */
#define SX127x_MOSI                    CONFIG_LASTLINK_GPIO_MOSI   /* SPI Master Out Slave In */
#define SX127x_MISO                    CONFIG_LASTLINK_GPIO_MISO   /* SPI Master In Slave Out */
#define SX127x_SS                      CONFIG_LASTLINK_GPIO_SS     /* SPI Slave Select */
#define SX127x_RESET                   CONFIG_LASTLINK_GPIO_RESET  /* Device reset */
#define SX127x_WANTED_VERSION          0x12

# Default values for some entries
#define ROUTE_LIFETIME                 CONFIG_LASTLINK_ROUTE_LIFETIME
#define MAX_ROUTES                     CONFIG_LASTLINK_MAX_ROUTES
#define ANNOUNCE_INTERVAL_DEFAULT      CONFIG_LASTLINK_ANNOUNCE_INTERVAL
#define REPLY_TIMEOUT                  CONFIG_LASTLINK_REPLY_TIMEOUT
#define ROUTEREQUEST_TIMEOUT           CONFIG_LASTLINK_ROUTEREQUEST_TIMEOUT
#define ROUTEREQUEST_RETRIES           CONFIG_LASTLINK_ROUTEREQUEST_RETRIES

#define PROTOCOL_LEN                   CONFIG_LASTLINK_PROTOCOL_LENGTH
#define FLAGS_LEN                      CONFIG_LASTLINK_FLAGS_LENGTH
#define ADDRESS_LEN                    CONFIG_LASTLINK_ADDRESS_LENGTH
#define BROADCAST_ADDRESS              CONFIG_LASTLINK_BROADCAST_ADDRESS
#define NULL_ADDRESS                   CONFIG_LASTLINK_NULL_ADDRESS
#define SEQUENCE_NUMBER_LEN            CONFIG_LASTLINK_SEQUENCE_NUMBER_LENGTH
#define INTERVAL_LEN                   CONFIG_LASTLINK_INTERVAL_LENGTH
#define MAX_METRIC                     (TTL_DEFAULT+1)
#define TTL_LEN                        CONFIG_LASTLINK_TTL_LENGTH
#define TTL_DEFAULT                    CONFIG_LASTLINK_TTL_DEFAULT
#define METRIC_LEN                     CONFIG_LASTLINK_METRIC_LENGTH
#define BEACON_NAME_LEN                CONFIG_LASTLINK_BEACON_LENGTH
#define REASON_LEN                     CONFIG_LASTLINK_REASON_LENGTH

# Helper functions used to build packet field items
def create_field(len, origin=0):
    if type(origin) == int:
        return (origin, len)
    elif type(origin) == tuple:
        return (origin[0] + origin[1], len)
    else:
        raise MeshNetException("Invalid type for 'origin'")

def end_field(field):
    return field[0] + field[1]


#########################################################################
# Supporting classes
#########################################################################
# A group of data with field referencing
class FieldRef(object):
    def __init__(self, **kwargs):
        # If data preload is given, load the data
        if 'load' in kwargs:
            self._data = bytearray(kwargs['load'])
        elif 'len' in kwargs:
            # Else if a length was given, preset with 0 bytes
            self._data = bytearray((0,) * kwargs['len'])
        else:
            # Otherwise just an empty array
            self._data = bytearray()

    def data(self):
        return self._data

    def __len__(self):
        return len(self._data)

    def __str__(self):
        return "FieldRef(%d)" % len(self)

    # Set or get a field of bytes, bigendian
    # <field> is ( <origin>, <length> )
    def _field(self, field, value=None, return_type=int):
        if type(value) == int:
            v = value
            for item in range(field[1] - 1, -1, -1):
                self._data[field[0] + item] = v % 256
                v >>= 8

        elif type(value) == str or type(value) == bytearray:
            # Convert string to bytes
            v = bytearray(value)

            # Extend to width of destination field
            if len(v) < field[1]:
                v.extend((0,) * (field[1] - len(value)))

            # Move string to limited field
            self._data[field[0]:field[0]+field[1]] = v[:field[1]]

        # Value is None, so fetch value from table
        elif value == None:
            if return_type == int:
                value = 0
                for item in range(field[1]):
                    value = (value << 8) + self._data[field[0] + item]
            else:
                return self._data[field[0]:field[0]+field[1]]

        return value

    # Used to set/clear/test bits in a field
    def _field_bit(self, field, bitnum=None, value=None):
        # print("_field_bit field %s bitnum %d value %s on packet %s" % (field, bitnum, value, str(self)))
        current = self._field(field)

        if value == None:
            # Testing a value
            current &= (1 << bitnum)

        else:
            if value:
                current |= (1 << bitnum)
            else:
                current &= ~(1 << bitnum)

            self._field(field, current)

        return current

    # Add 'len' bytes to record
    def _extend(self, size):
        self._data.extend(bytearray(size))


_HEADER_NEXTHOP             = create_field(_ADDRESS_LEN)
_HEADER_TARGET              = create_field(_ADDRESS_LEN, _HEADER_NEXTHOP)
_HEADER_PREVIOUS            = create_field(_ADDRESS_LEN, _HEADER_TARGET)
_HEADER_SOURCE              = create_field(_ADDRESS_LEN, _HEADER_PREVIOUS)
_HEADER_PROTOCOL            = create_field(_PROTOCOL_LEN, _HEADER_SOURCE)
_HEADER_TTL                 = create_field(_TTL_LEN, _HEADER_PROTOCOL)
_HEADER_LENGTH              = end_field(_HEADER_TTL)
_HEADER_PAYLOAD             = _HEADER_LENGTH

def ADDR_OF(addr):
    if addr == 0:
        return "NULL"

    elif addr == BROADCAST_ADDRESS:
        return "BCAST"

    else:
        return "%d" % addr

class Packet(FieldRef):

    def __init__(self, **kwargs):

        # Allocate enough for the header; more will be allocated
        if 'len' not in kwargs:
            kwargs['len'] = _HEADER_LENGTH
        super(Packet, self).__init__(**kwargs)

        self._rssi = None
        self._promiscuous = False

        # Set defaults if no origin data
        if 'load' not in kwargs:
            self.nexthop    (kwargs['nexthop']  if 'nexthop'  in kwargs else NULL_ADDRESS)   # To is usually a specific node or broadcast for things like beacons; if None, filled in by route
            self.previous   (kwargs['previous'] if 'previous' in kwargs else NULL_ADDRESS)   # From is filled in by send_packet if not defined at creation
            self.target     (kwargs['target']   if 'target'   in kwargs else NULL_ADDRESS)   # Destination is usually a specific node or broadcast.  Usually the final destination
            self.source     (kwargs['source']   if 'source'   in kwargs else NULL_ADDRESS)   # Source is usually from this host
            self.ttl        (kwargs['ttl']      if 'ttl'      in kwargs else _TTL_DEFAULT)   # TTL is default for most packets; set to 1 for beacons or one-time notices
            self.protocol   (kwargs['protocol'] if 'protocol' in kwargs else 0)              # Protocol is packet type - defined by Packet inheriter

            if 'data' in kwargs:
                self._data[_HEADER_LENGTH:] = bytearray(kwargs['data'])

        # RSSI of receiver if present
        self.rssi(kwargs['rssi'] if 'rssi' in kwargs else None)


    def __str__(self):
        return "Packet N=%s P=%s T=%s S=%s TTL=%d Proto=%d Len=%d" % (ADDR_OF(self.nexthop()), ADDR_OF(self.previous()), ADDR_OF(self.target()), ADDR_OF(self.source()), self.ttl(), self.protocol(), len(self.data()))

    def promiscuous(self, value=None):
        if value != None:
            self._promiscuous = value
        return self._promiscuous

    def rssi(self, value=None):
        if value != None:
            self._rssi = value
        return self._rssi

    def nexthop(self, value=None):
        return self._field(_HEADER_NEXTHOP, value) 

    def previous(self, value=None):
        return self._field(_HEADER_PREVIOUS, value)

    def target(self, value=None):
        return self._field(_HEADER_TARGET, value) 

    def source(self, value=None):
        return self._field(_HEADER_SOURCE, value)

    def ttl(self, value=None):
        return self._field(_HEADER_TTL, value)

    def protocol(self, value=None):
        return self._field(_HEADER_PROTOCOL, value)

    def process(self, parent):
        raise MeshNetException("Packet.process is not callable")

#########################################################################
# Beacon packet
#########################################################################
_BEACON_NAME                = create_field(_BEACON_NAME_LEN, _HEADER_PAYLOAD)
_BEACON_LENGTH              = end_field(_BEACON_NAME)

class Beacon(Packet):
    PROTOCOL_ID = 0

    def __init__(self, **kwargs):
        kwargs['len'] = _BEACON_LENGTH
        kwargs['protocol'] = self.PROTOCOL_ID
        kwargs['ttl'] = 1
        kwargs['nexthop'] = BROADCAST_ADDRESS
        super(Beacon, self).__init__(**kwargs)

        self.name(kwargs['name'] if 'name' in kwargs else "Beacon")

    def __str__(self):
        return "Beacon: [%s] N='%s'" % (super().__str__(), self.name())

    def name(self, value=None):
        return self._fields(_BEACON_NAME, value, return_type=str)

    def process(self, parent):
        with parent._packet_lock:
            # For now we just print them out
            if parent._debug:
                print(str(self))

#########################################################################
# Route announce
#########################################################################
#
# RouteAnnounce is used to advertise a route to a node, usually a gateway node.
#
# When sent to a specific node, it announces an established route.
# When sent as broadcast, it helps establish routes (and gateway status)
# and is rebroadcast so adjacent nodes will see it.
#
_RANN_FLAGS                 = create_field(_FLAGS_LEN, _HEADER_PAYLOAD)
_RANN_FLAGS_GATEWAY             = const(0)
_RANN_SEQUENCE              = create_field(_SEQUENCE_NUMBER_LEN, _RANN_FLAGS)
_RANN_METRIC                = create_field(_METRIC_LEN, _RANN_SEQUENCE)
_RANN_LENGTH                = end_field(_RANN_METRIC)

class RouteAnnounce(Packet):
    PROTOCOL_ID = 1

    def __init__(self, **kwargs):
        kwargs['len'] = _RANN_LENGTH
        kwargs['protocol'] = self.PROTOCOL_ID
        super(RouteAnnounce, self).__init__(**kwargs)

        if 'load' not in kwargs:
            self.gateway_flag(kwargs['gateway'] if 'gateway' in kwargs else False)
            self.metric(kwargs['metric'] if 'metric' in kwargs else 1)
            self.sequence(kwargs['sequence'] if 'sequence' in kwargs else 0)

    def __str__(self):
        return "RouteAnnounce: [%s] Seq=%d M=%d F=%02x" % (super().__str__(), self.sequence(), self.metric(), self.flags())

    def flags(self, value=None):
        return self._field(_RANN_FLAGS, value)

    def gateway_flag(self, value=None):
        return self._field_bit(_RANN_FLAGS, _RANN_FLAGS_GATEWAY, value)

    def sequence(self, value=None):
        return self._field(_RANN_SEQUENCE, value)

    def metric(self, value=None):
        return self._field(_RANN_METRIC, value)

    #
    # Capture the route to the <source> and rebroadcast if TTL is non-zero
    # If we already have a route to this node, only capture updated metric if it gets better
    #
    def process(self, parent):
        with parent._packet_lock:
            route = parent.update_route(target=self.source(), nexthop=self.previous(), sequence=self.sequence(), metric=self.metric(), gateway_flag=self.gateway_flag())
            if route != None:
                if parent._debug:
                    print("RouteAnnounce: better route to %s" % str(route))
                # We have a new/better route.  If not for us, announce it.
                if self.target() == parent.address:
                    # We have a route to target.  Rebroadcast any pending packets.
                    with parent._route_lock:
                        # Release the request and all waiting packets
                        route.release_pending_routerequest(parent)

                else:
                    # Mark as NULL so the route gets recomputed
                    self.nexthop(NULL_ADDRESS)
                    self.metric(self.metric() + 1)
                    parent.send_packet(self, ttl=True)

#########################################################################
# Route Request
#########################################################################
#
# A route request is sent by a node when it needs to know a route to a specific target
# node.  This packet is always braodcast rather than single address.
#
# This packet will be rebroadcast until it reaches all nodes.  This will
# partially build routing tables through the N-1 nodes handling the packets
# but will result in a RouteAnnounce from the destination node indicating
# to the caller the node that returned it plus the metric to that node.
#
_RREQ_FLAGS                 = create_field(_FLAGS_LEN, _HEADER_PAYLOAD)
_RREQ_FLAGS_GATEWAY             = const(0)
_RREQ_SEQUENCE              = create_field(_SEQUENCE_NUMBER_LEN, _RREQ_FLAGS)
_RREQ_METRIC                = create_field(_METRIC_LEN, _RREQ_SEQUENCE)
_RREQ_LENGTH                = end_field(_RREQ_METRIC)

class RouteRequest(Packet):
    PROTOCOL_ID = 2

    def __init__(self, **kwargs):
        kwargs['len'] = _RREQ_LENGTH
        kwargs['protocol'] = self.PROTOCOL_ID
        kwargs['nexthop'] = BROADCAST_ADDRESS
        super(RouteRequest, self).__init__(**kwargs)

        # Set defaults if no origin data
        if 'load' not in kwargs:
            self.gateway_flag(kwargs['gateway'] if 'gateway' in kwargs else False)
            self.sequence(kwargs['sequence'] if 'sequence' in kwargs else None)
            self.metric(kwargs['metric'] if 'metric' in kwargs else 1)

    def __str__(self):
        return "RouteRequest: [%s] Seq=%d M=%d F=%02x" % (super().__str__(), self.sequence(), self.metric(), self.flags())

    def flags(self, value=None):
        return self._field(_RREQ_FLAGS, value)

    def gateway_flag(self, value=None):
        return self._field_bit(_RREQ_FLAGS, _RREQ_FLAGS_GATEWAY, value)

    def sequence(self, value=None):
        return self._field(_RREQ_SEQUENCE, value)

    def metric(self, value=None):
        return self._field(_RREQ_METRIC, value)

    # Process in incoming RouteRequest
    # TODO: Need brakes to avoid transmitting too many at once !! (Maybe ok for testing)
    def process(self, parent):
        with parent._packet_lock:
            # Update route to the source to reflect a possibe path to the source
            route = parent.update_route(target=self.source(), nexthop=self.previous(), sequence=self.sequence(), metric=self.metric(), gateway_flag=self.gateway_flag())

            # If packet is asking us, create the RouteAnnounce
            if self.target() == parent.address:
                parent.send_packet(RouteAnnounce(target=self.previous(), sequence=self.sequence(), metric=self.metric(), gateway_flag=parent._gateway))

            # Otherwise send the packet on if the route is better than the last time (ignoring duplicate paths through this node)
            elif self.nexthop() == BROADCAST_ADDRESS and route != None:
                self.metric(self.metric() + 1)
                parent.send_packet(self, ttl=True)


#########################################################################
# Route Error
#########################################################################
#
# A RouteError is returned to the source for any packet that cannot be
# delivered.
#
_RERR_ADDRESS               = create_field(_ADDRESS_LEN, _HEADER_PAYLOAD)
_RERR_SEQUENCE              = create_field(_SEQUENCE_NUMBER_LEN, _RERR_ADDRESS)
_RERR_REASON                = create_field(_REASON_LEN, _RERR_SEQUENCE)
_RERR_LENGTH                = end_field(_RERR_REASON)

class RouteError(Packet):
    PROTOCOL_ID = 4

    def __init__(self, **kwargs):
        kwargs['len'] = _RERR_LENGTH
        kwargs['protocol'] = self.PROTOCOL_ID
        super(RouteError, self).__init__(**kwargs)

        self.address(kwargs['address'] if 'address' in kwargs else 0)
        self.sequence(kwargs['sequence'] if 'sequence' in kwargs else 0)
        self.reason(kwargs['reason'] if 'reason' in kwargs else 0)

    def __str__(self):
        return "RouteError: [%s] A=%d Seq=%d R=%d" % (super().__str__(), self.address(), self.sequence(), self.readon())

    def address(self, value=None):
        return self._field(_RERR_ADDRESS, value)

    def sequence(self, value=None):
        return self._field(_RERR_SEQUENCE, value)

    def reason(self, value=None):
        return self._field(_RERR_REASON, value)

    # Don't call packet.process since we do not care routing info
    def process(self, parent):
        with parent._packet_lock:
            if parent._debug:
                print(str(self))


#########################################################################
# Data packet.
#########################################################################
#
# A Data packet is used to convey any data between nodes.  This packet
# will be overloaded for any purpose that is needed.
#
_DATA_LENGTH                = _HEADER_LENGTH

class DataPacket(Packet):
    def __init__(self, **kwargs):
        payload = kwargs['payload'] if 'payload' in kwargs else bytearray()
        if type(payload) == str:
            payload = bytearray(payload)

        kwargs['len'] = _DATA_LENGTH + len(payload)
        # Create base items in packet
        super(DataPacket, self).__init__(**kwargs)

        # Apply payload if we have one
        if len(payload) != 0:
            self.payload(payload)

    def __str__(self):
        try:
            return "Data: [%s] '%s'" % (super().__str__(), self.payload().decode())
        except:
            return "Data: [%s] '%s'" % (super().__str__(), self.payload())

    # Set or read the payload portion of the data
    def payload(self, value=None, start=0, end=None):
        start += _DATA_LENGTH
        end = None if end == None else end + _DATA_LENGTH

        if value == None:
            return self._data[start:end]
        else:
            self._data[start:end] = value
        return value

    # We don't call packet.process() since we are not getting any routing info from
    # this packet.
    def process(self, parent):
        # if parent._debug:
        #    print("DataPacket.process called")

        queued = False

        with parent._packet_lock:
            # If this packet has found it's recipient, put in queue
            if self.target() == parent.address:
                parent.put_receive_packet(self)

            else:
                # Reset nexthop so route is recomputed
                self.nexthop(NULL_ADDRESS)

                # Route the packet onward if ttl not expired
                parent.send_packet(self, ttl=True)

class Route():
    def __init__(self, **kwargs):
        lifetime = kwargs['lifetime'] if 'lifetime' in kwargs else _ROUTE_LIFETIME
        self.update_lifetime(lifetime)
        self._pending_queue = queue()
        self._sequence = kwargs['sequence'] if 'sequence' in kwargs else 0
        self._metric = kwargs['metric'] if 'metric' in kwargs else 0
        self._target = kwargs['target'] if 'target' in kwargs else NULL_ADDRESS
        self._nexthop = kwargs['nexthop'] if 'nexthop' in kwargs else NULL_ADDRESS
        self._gateway = kwargs['gateway'] if 'gateway' in kwargs else False
        self._pending_routerequest = None
        self._pending_routerequest_retry_timer = 0
        self._pending_routerequest_retry_timeout = 0
        self._pending_routerequest_retries = 0

    def __str__(self):
        return "Route T=%d N=%d M=%d Seq=%d F=%02x Life=%.1f Q=%d" % (self._target, self._nexthop, self._metric, self._sequence, self._gateway, self._lifetime - time(), len(self._pending_queue))

    def update_lifetime(self, lifetime=_ROUTE_LIFETIME):
        self._lifetime = time() + lifetime

    def put_pending_packet(self, packet):
        self._pending_queue.put(packet)

    def set_pending_routerequest(self, request, retries=_ROUTEREQUEST_RETRIES, timeout=_ROUTEREQUEST_TIMEOUT):
        # The request to make
        self._pending_routerequest = request

        # Number of tries to make
        self._pending_routerequest_retries = retries

        # When to start the retry
        self._pending_routerequest_retry_timeout = timeout
        self._pending_routerequest_retry_timer = time() + timeout

    # Get the pending routerequest.  If expired, remove it.  Return the request if found.
    def get_pending_routerequest(self):
        packet = None

        if time() >= self._pending_routerequest_retry_timer:
            # Decrease retry count
            self._pending_routerequest_retries -= 1
            if self._pending_routerequest_retries >= 0:
                # Restart timer
                self._pending_routerequest_retry_timer = time() + self._pending_routerequest_retry_timeout
                packet = self._pending_routerequest

            else:
                self._pending_routerequest = None

        return packet

    def release_pending_routerequest(self, parent):
        self._pending_routerequest = None

        packet = True
        # print("Release Pending for %s" % (str(self)))
        while packet:
            packet = self._pending_queue.get(wait=0)
            if packet:
                # print("Release sending %s" % (str(packet)))
                parent.send_packet(packet)
        # print("Release done")

    def is_expired(self):
        return time() >= self._lifetime

    def sequence(self, value=None):
        if value == None:
            return self._sequence
        else:
            self._sequence = value

    def metric(self, value=None):
        if value == None:
            return self._metric
        else:
            self._metric = value

    def nexthop(self, value=None):
        if value == None:
            return self._nexthop
        else:
            self._nexthop = value

    def gateway_flag(self, value=None):
        if value == None:
            return self._gateway
        else:
            self._gateway = value

#########################################################################
# This level maintains handles the routing protocol
# and will deliver non-routing messages to the inheriter.
#########################################################################
class MeshNet(RadioDriver):

    def __init__(self, domain, address, **kwargs):
        super(MeshNet, self).__init__(domain, **kwargs)

        self.address = address
        self._meshlock = rlock()
        self._transmit_queue = queue()
        self._receive_queue = queue()
        self._hwmp_sequence_number = 0
        self._hwmp_sequence_lock = lock()
        self._route_lock = rlock()
        self._promiscuous = False
        self._debug = False

        self._announce_thread = None

        # Defines routes to nodes
        self._routes = {}
        self._packet_errors_crc = 0
        self._packet_received = 0
        self._packet_transmitted = 0
        self._packet_ignored = 0
        self._packet_lock = rlock()

        self._gateway = kwargs['gateway'] if 'gateway' in kwargs else False
        if self._gateway:
            self._announce_interval = float(kwargs['interval']) if 'interval' in kwargs else _ANOUNCE_DEFAULT_INTERVAL
        else:
            self._announce_interval = 0

        self._PROTOCOLS = {
                RouteAnnounce.PROTOCOL_ID: RouteAnnounce,
                RouteRequest.PROTOCOL_ID:  RouteRequest,
                RouteError.PROTOCOL_ID:    RouteError,
                None:                      DataPacket,   # Data packet protocol id is a wildcard
        }

    # In promiscuous mode, all received packets are dropped into the receive queue, as well
    # as being processed.
    def set_promiscuous(self, mode=True):
        self._promiscuous = mode

    def set_debug(self, mode = True):
        self._debug = mode

    def start(self):
        self._spi = SPI(baudrate=10000000, polarity=0, phase=0, bits=8, firstbit = SPI.MSB,
                        sck = Pin(_SX127x_SCK, Pin.OUT, Pin.PULL_DOWN),
                        mosi = Pin(_SX127x_MOSI, Pin.OUT, Pin.PULL_UP),
                        miso = Pin(_SX127x_MISO, Pin.IN, Pin.PULL_UP))

        self._ss = Pin(_SX127x_SS, Pin.OUT)
        self._reset = Pin(_SX127x_RESET, Pin.OUT)
        self._dio_table = [ Pin(_SX127x_DIO0, Pin.IN), Pin(_SX127x_DIO1, Pin.IN), Pin(_SX127x_DIO2, Pin.IN) ]
        self._ping_count = 0
        self._power = None # not True nor False

        # Perform base class init
        # super(MeshNet, self).start(_SX127x_WANTED_VERSION)
        super().start(_SX127x_WANTED_VERSION, activate=False)

        # Set power state
        self.set_power()

        # A timer than can be started to do retries; not started until needed
        self._retry_routerequests_thread = thread(run=self._retry_routerequests, stack=8192)
        self._retry_routerequests_thread.start(timeout=0.5)

        # Start announce if requested
        if self._gateway:
            self.announce_start(self._announce_interval / 1000.0)

    def announce_start(self, interval):
        print("Announce gateway every %.1f seconds" % interval)
        self._announce_thread = thread(run=self._announce, stack=8192)
        self._announce_thread.start(interval=interval)

    def _announce(self, t, interval):
        countdown = interval

        # Only sleep a second at a time so we can be killed fairly quickly
        while t.running:
            sleep(1)
            countdown -= 1
            if countdown <= 0:
                countdown += interval
                packet = RouteAnnounce(target=BROADCAST_ADDRESS, nexthop=BROADCAST_ADDRESS, sequence=self._create_sequence_number(), gateway_flag=self._gateway)
                self.send_packet(packet)

        return 0

    # Return the protocol wrapper or Data is not otherwise defined
    def get_protocol_wrapper(self, protocol):
        return self._PROTOCOLS[protocol] if protocol in self._PROTOCOLS else self._PROTOCOLS[None]

    # Remove a root
    def remove_route(self, address):
        with self._route_lock:
            if address in self._routes:
                del(self_routes[address])

    # Update a route.  If route is not defined, create it.  If defined but metric is better or sequence is different, update it.
    # Return True if new or updated route
    def update_route(self, target, nexthop, sequence, metric=_MAX_METRIC, gateway_flag=False, force=False):
        with self._route_lock:
            if force or target not in self._routes or self._routes[target].is_expired():
                # Create new route
                route = Route(target=target, nexthop=nexthop, sequence=sequence, metric=metric, gateway_flag=gateway_flag)
                self._routes[target] = route
                if self._debug:
                    print("Created %s" % str(route))

            # Else if we are forcing creation, or the sequence number is different or the metric is better, create a new route
            elif sequence != self._routes[target].sequence() or metric < self._routes[target].metric():
                # Update route
                route = self._routes[target]
                route.nexthop(nexthop)
                route.metric(metric)
                route.sequence(sequence)
                route.update_lifetime()
                if self._debug:
                    print("Updated %s" % str(route))

            else:
                # No route to host
                route = None

        return route

    # Default to search routes table
    def find_route(self, address):
        with self._route_lock:
            return self._routes[address] if address in self._routes and not self._routes[address].is_expired() else None

    # Reset device
    def reset(self):
        self._reset.value(0)
        sleep(0.1)
        self._reset.value(1)

    # Read register from SPI port
    def read_register(self, address):
        value = int.from_bytes(self._spi_transfer(address & 0x7F), 'big')
        # print("%02x from %02x" % (value, address))
        return value

    # Write register to SPI port
    def write_register(self, address, value):
        # print("write %02x to %02x" % (value, address))
        self._spi_transfer(address | 0x80, value)

    def _spi_transfer(self, address, value=0):
        response = bytearray(1)
        self._ss.value(0)
        self._spi.write(bytes([address]))
        self._spi.write_readinto(bytes([value]), response)
        self._ss.value(1)
        return response

    # Read block of data from SPI port
    def read_buffer(self, address, length):
        try:
            response = bytearray(length)
            self._ss.value(0)
            self._spi.write(bytes([address & 0x7F]))
            self._spi.readinto(response)
            self._ss.value(1)

        except:
            # No room.  gc now
            gc.collect()
            response = None

        return response

    # Write block of data to SPI port
    def write_buffer(self, address, buffer, size):
        self._ss.value(0)
        self._spi.write(bytes([address | 0x80]))
        self._spi.write(memoryview(buffer)[0:size])
        self._ss.value(1)

    def attach_interrupt(self, dio, edge, callback):
        # if self._debug:
        #    print("attach_interrupt dio %d rising %s with callback %s" % (dio, edge, callback))

        if dio < 0 or dio >= len(self._dio_table):
            raise Exception("DIO %d out of range (0..%d)" % (dio, len(self._dio_table) - 1))

        edge = Pin.IRQ_RISING if edge else Pin.IRQ_FALLING
        self._dio_table[dio].irq(handler=callback, trigger=edge if callback else 0)

    # Enwrap the packet with a class object for the particular message type
    def wrap_packet(self, data, rssi=None):
        return self.get_protocol_wrapper(data[_HEADER_PROTOCOL[0]])(load=data, rssi=rssi)
        
    # Duplicate packet with new private data
    def dup_packet(self, packet):
        return self.wrap_packet(bytearray(packet.data()), rssi=packet.rssi())

    def onReceive(self, data, crc_ok, rssi):
        if crc_ok:
            packet = self.wrap_packet(data, rssi)

            nexthop = packet.nexthop()

            if self._debug:
                print("Received: %s" % (str(packet)))

            # In promiscuous, deliver to receiver so it can handle it (but not process it)
            if self._promiscuous:
                packet_copy = self.dup_packet(packet)
                packet_copy.promiscuous(True)
                self.put_receive_packet(packet_copy)

            if nexthop == BROADCAST_ADDRESS or nexthop == self.address:
                self._packet_received += 1
                # To us or broadcasted
                packet.process(self)

            else:
                # It is non processed
                self._packet_ignored += 1

        else:
            self._packet_errors_crc += 1


    def receive_packet(self):
        gc.collect()
        return self._receive_queue.get()

    def put_receive_packet(self, packet):
        self._receive_queue.put(packet)
        gc.collect()

    # Finished transmitting - see if we can transmit another
    # If we have another packet, return it to caller.
    def onTransmit(self):
        # if self._debug:
        #    print("onTransmit complete")

        self._packet_transmitted += 1

        # Delete top packet in queue
        packet = self._transmit_queue.get(wait=0)
        del packet

        # Return head of queue if one exists
        packet = self._transmit_queue.head()

        gc.collect()

        return packet.data() if packet else None

    def _create_sequence_number(self):
        with self._hwmp_sequence_lock:
            self._hwmp_sequence_number += 1
            return self._hwmp_sequence_number

    # A packet with a source and destination is ready to transmit.
    # Label the from address and if no to address, attempt to route
    # If ttl is true, decrease ttl and discard packet if 0
    def send_packet(self, packet, ttl=False):
        if ttl and packet.ttl(packet.ttl() - 1) == 0:
            # Packet has expired
            if self._debug:
                print("Expired: %s" % str(packet))
        else:
            # Label packets as coming from us
            packet.previous(self.address)
            # print("%s: set previous to %d" % (str(packet), self.address))

            # Label as originating here if no previous assigned source address
            if packet.source() == NULL_ADDRESS:
                packet.source(self.address)
                # print("%s: set source to %d" % (str(packet), self.address))

            # If the nexthop is NULL, then we compute next hop based on route table.
            # If no route table, create pending NULL route and cache packet for later retransmission.
            if packet.nexthop() == NULL_ADDRESS:
                with self._route_lock:
                    # Look up the route to the destination
                    route = self.find_route(packet.target())

                    # If no route, create a dummy route and queue the results
                    if route == None:
                        # Unknown route.  Create a NULL route awaiting RouteAnnounce
                        route = self.update_route(target=packet.target(), nexthop=NULL_ADDRESS, sequence=self._create_sequence_number(), force=True)

                        # Save packet in route for later delivery
                        route.put_pending_packet(packet)

                        if self._debug:
                            print("Routing %s" % str(packet))
                        request = RouteRequest(target=packet.target(), previous=self.address, source=self.address, sequence=route.sequence(), metric=1, gateway_flag=self._gateway)

                        # This will queue repeats of this request until cancelled
                        route.set_pending_routerequest(request)

                    elif route.nexthop() == NULL_ADDRESS:
                        # We still have a pending route, so append packet to queue only.
                        request = None
                        route.put_pending_packet(packet)

                    else:
                        # Label the destination for the packet
                        packet.nexthop(route.nexthop())
                        request = packet

                # Transmit the request if we created one or else the actual packet
                packet = request

            if packet:
                #
                # TODO: we may need another method to restart transmit queue other than looking
                # and transmit queue length.  A 'transmitting' flag (protected by meshlock) that
                # is True if transmitting of packet is in progress.  Cleared on onTransmit when
                # queue has become empty.
                #
                # This may need to be implemented to allow stalling transmission for windows of
                # reception.  A timer will then restart the queue if items remain within it.
                #
                # if self._debug:
                #     print("sending: %s" % str(packet))

                with self._meshlock:
                    # print("Appending to queue: %s" % packet.decode())
                    self._transmit_queue.put(packet)
                    if len(self._transmit_queue) == 1:
                        self.transmit_packet(packet.data())
                        if self._debug:
                            print("Transmitted: %s" % str(packet))

    # A thread to check all routes and those with resend the packets for those with retry requests
    def _retry_routerequests(self, thread, timeout):
        while thread.running:
            sleep(timeout)
            with self._route_lock:
                # Go through all routes looking for those with pending requests.
                for target in list(self._routes):
                    route = self._routes[target]
                    # If route is expired, remove it
                    if route.is_expired():
                        # Clean up route
                        del(self._routes[target])

                    # Otherwise if it has a pending request, resend it
                    else:
                        packet = route.get_pending_routerequest()
                        if packet:
                            if self._debug:
                                print("Retry route request %s" % str(packet))
                            self.send_packet(packet)

    def stop(self):
        # Stop announce if running
        if self._announce_thread:
            self._announce_thread.stop()
            self._announce_thread.wait()
            self._announce_thread = None

        if self._retry_routerequest_thread != None:
            self._retry_routerequest_thread.stop()
            self._retry_routerequest_thread.wait()
            self._retry_routerequest_thread = None

        super(MeshNet, self).stop()

        # Shut down power
        self.set_power(False)

        print("MeshNet handler close called")
        # Close DIO interrupts
        for dio in self._dio_table:
            dio.irq(handler=None, trigger=0)

        # Close SPI channel if opened
        if self._spi:
            self._spi.deinit()
            self._spi = None

    def set_power(self, power=True):
        # print("set_power %s" % power)

        if power != self._power:
            self._power = power

            # Call base class
            super(MeshNet, self).set_power(power)

    def __del__(self):
        self.stop()

    def dump(self):
        item = 0
        for reg in range(0x43):
            print("%02x: %02x" % (reg, self.read_register(reg)), end="    " if item != 7 else "\n")
            item = (item + 1) % 8
        print("")

