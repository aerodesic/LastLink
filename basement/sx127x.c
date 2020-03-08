/*
 * Driver for sx127x chip.
 *
 * Initialized with a control table of worker functions.
 */
#include <stdconfig.h>

/* Disable driver if it is not needed */
#ifdef CONFIG_LASTLINK_RADIO_SX127x_ENABLED

#include <stdlib.h>
#include <ctype.h>
#include <math.h>

#include "sx127x.h"
#include "radio.h"

/*
 * Create an instance of a SX127x device.
 *
 * Entry:
 * 	radio		Pointer to value to receive pointer to SX127x control block
 * 	actions		Actions provided by caller to do the actual I/O
 *
 * Returns:
 * 	if >= 0, control-block index for other device calls
 *
 */
int
sx127x_init(RadioActions** RadioInterfaceIo* actions, int xtal, int channel, int delay) {

	SX127x_actions = actions;

	return 0;
}



void calc_freq(double freq, int *msb, int* mid, int *lsb) {
	double frf = round(freq / SX127x_PLL_STEP);
	*msb = (int) (frf / 65536);
	*mid = (int) (frf / 256) % 256;
	*lsb = (int) frf % 256;
}

    def __init__(self, domain, **kwargs):
        self._domain  = domain
        self._xtal    = kwargs['xtal']    if 'xtal'    in kwargs else 32e6
        self._channel = kwargs['channel'] if 'channel' in kwargs else None
        self._delay   = kwargs['delay']   if 'delay'   in kwargs else _DEFAULT_PACKET_DELAY

        self._packets_memory_errors = 0

        self._pll_step = self._xtal / 2**19
        # print("PLL step %f" % self._pll_step)

        # Define channel table for this frequency
        if 'channels' in self._domain:
            self._channels = {}
            for channel in self._domain['channels']:
                freq = channel['freq'][0]
                step = channel['freq'][1]
                for c in range(channel['chan'][0], channel['chan'][1] + 1):
                    # self._channels[chantype][c] = { 'dr': channel['dr'], 'freq': self._calc_freq(freq) }
                    # Add hz to table for debugging purposes - not really used
                    self._channels[c] = { 'dr': channel['dr'], 'freq': self._calc_freq(freq), 'hz': freq }
                    freq += step

            # # Dump for inspection
            # for c in self._channels:
            #     print("%d: %s" % (c, self._channels[c]))

        else:
            raise LorDeviceException("'channels' not found in domain")

        if 'data_rates' in self._domain:
            self._data_rates = self._domain['data_rates']

        else:
            raise SX127xDeviceException("'data_rates' not found in domain")

        self._sync_word        = kwargs['sync_word']        if 'sync_word'        in kwargs else 0x34
        self._preamble_length  = kwargs['preamble_length']  if 'preamble_length'  in kwargs else 8
        self._coding_rate      = kwargs['coding_rate']      if 'coding_rate'      in kwargs else 5
        self._implicit_header  = kwargs['implicit_header']  if 'implicit_header'  in kwargs else False
        # self._hop_period     = kwargs['hop_period']       if 'hop_period'       in kwargs else 0
        self._enable_crc       = kwargs['enable_crc']       if 'enable_crc'       in kwargs else True

        # Set default in case not set by caller
        self._bandwidth        = kwargs['bandwidth']        if 'bandwidth'        in kwargs else 125e3
        self._spreading_factor = kwargs['spreading_factor'] if 'spreading_factor' in kwargs else 7
        self._tx_power         = kwargs['tx_power']         if 'tx_power'         in kwargs else 2

        self._tx_interrupts = 0
        self._rx_interrupts = 0
        # self._fhss_interrupts = 0

        self._current_implicit_header = None

        self._lock = rlock()


    def start(self, wanted_version=0x12, activate=True):
        self.reset()

        # Read version
        version = None
        max_tries = 5
        while version != wanted_version and max_tries != 0:
            version = self.read_register(_SX127x_REG_VERSION)
            max_tries = max_tries - 1

        if version != wanted_version:
            raise Exception("Wrong version detected: %02x wanted %02x" % (version, wanted_version))

        # Put receiver in sleep
        self.set_sleep_mode()

        # Set initial default  parameters parameters
        self.set_bandwidth(self._bandwidth)
        self.set_spreading_factor(self._spreading_factor)
        self.set_tx_power(self._tx_power)
        self.set_implicit_header(self._implicit_header)
        self.set_coding_rate(self._coding_rate)
        self.set_preamble_length(self._preamble_length)
        self.set_sync_word(self._sync_word)
        self.set_enable_crc(self._enable_crc)
        # self.set_hop_period(self._hop_period)

        # Configure the unit for receive (may override several of above)
        if self._channel != None:
            self.set_channel(self._channel)
        else:
            self.set_channel((0, -1))

        # LNA Boost
        self.write_register(_SX127x_REG_LNA, self.read_register(_SX127x_REG_LNA) | 0x03)  # MANIFEST CONST?

        # auto AGC enable
        self.write_register(_SX127x_REG_MODEM_CONFIG_3, 0x04)  # MANIFEST??

        self.write_register(_SX127x_REG_TX_FIFO_BASE, _TX_FIFO_BASE) 
        self.write_register(_SX127x_REG_RX_FIFO_BASE, _RX_FIFO_BASE) 

        # Mask all but Tx and Rx
        self.write_register(_SX127x_REG_IRQ_FLAGS_MASK, 0xFF & ~(_SX127x_IRQ_TX_DONE | _SX127x_IRQ_RX_DONE))

        # Clear all interrupts
        self.write_register(_SX127x_REG_IRQ_FLAGS, 0xFF)

        # if self._hop_period != 0:
        #     # Catch the FSHH step
        #     self.attach_interrupt(1, True, self._fhss_interrupt)

        if activate:
            # Place in standby mode
            self.set_receive_mode()
        else:
            self.set_standby_mode()

        # print("SX127x started")


    # If we cannot do a block write, write byte at a time
    # Can be overwritten by base class to achieve better throughput
    def write_buffer(self, address, buffer, size):
        # print("write_buffer: '%s'" % buffer.decode())
        for i in range(size):
            self.write_register(address, buffer[i])

    # If user does not define a block write, do it the hard way
    def read_buffer(self, address, length):
        buffer = bytearray()
        for l in range(length):
            buffer.append(self.read_register(address))
        self._garbage_collect()
        return buffer

    # Must be overriden by base class
    def write_register(self, reg, value):
        raise Exception("write_register not defined.")

    def read_register(self, reg):
        raise Exception("read_register not defined.")

    def attach_interrupt(self, dio, edge, callback):
        raise Exception("enable_interrupt not defined.")

    def set_power(self, power=True):
        if power:
            # Bring things up
            self.set_receive_mode()
        else:
            self.set_sleep_mode()

    def get_packet_rssi(self):
        rssi = self.read_register(_SX127x_REG_PACKET_RSSI) - 157
        if self._domain['freq_range'][0] < 868E6:
            rssi = rssi + 7
        return rssi

    def get_packet_snr(self):
        return self.read_register(_SX127x_REG_PACKET_SNR) / 4.0

    def set_standby_mode(self):
        # print("standby mode")
        self.write_register(_SX127x_REG_OP_MODE, _SX127x_MODE_LONG_RANGE | _SX127x_MODE_STANDBY)

    def set_sleep_mode(self):
        # print("sleep mode")
        self.write_register(_SX127x_REG_OP_MODE, _SX127x_MODE_LONG_RANGE | _SX127x_MODE_SLEEP)

    def set_receive_mode(self):
        # print("receive mode")
        # self.set_channel(self._receive_channel)
        self.attach_interrupt(0, True, self._rxhandle_interrupt)
        # self.write_register(_SX127x_REG_OP_MODE, _SX127x_MODE_LONG_RANGE | _SX127x_MODE_RX_SINGLE)
        self.write_register(_SX127x_REG_OP_MODE, _SX127x_MODE_LONG_RANGE | _SX127x_MODE_RX_CONTINUOUS)
        self.write_register(_SX127x_REG_DIO_MAPPING_1, 0b00000000)

    def set_transmit_mode(self):
        # print("transmit mode")
        # Reset SEED
        # self.set_channel(self._transmit_channel)
        self.attach_interrupt(0, True, self._txhandle_interrupt)
        self.write_register(_SX127x_REG_OP_MODE, _SX127x_MODE_LONG_RANGE | _SX127x_MODE_TX)
        self.write_register(_SX127x_REG_DIO_MAPPING_1, 0b01000000)

    # Level in dBm
    def set_tx_power(self, tx_power):
        if type(tx_power) == int:
            tx_power = (tx_power, "PA")

        self._tx_power = tx_power
        if tx_power[1] == "PA":
            # PA Boost mode
            level = min(max(int(round(tx_power[0]) - 2), 0), 15)
            self.write_register(_SX127x_REG_PA_CONFIG, _SX127x_PA_BOOST | tx_power[0])
        else:
            self.write_register(_SX127x_REG_PA_CONFIG, 0x70 | (min(max(tx_power[0], 0), 15)))

    def get_tx_power(self):
        return self._tx_power

    #
    # set channel and optional data_rate
    #
    # Parameters:
    #    channel              Channel, which defines frequency and bandwidth
    #    datarate             Datarate for channel; None doesn't change datarate; -1 sets to default.
    #                         Note: if datarate is invalid for new channel, it will be set to the default.
    # Options:
    #    set_channel(<channel>, <datarate>)     # Switch to specific channel and datarate
    #    set_channel((<channel>, <datarate>))   # Switch to specific channel and datarate from tuple
    #    set_channel(<channel>)                 # Set channel with default datarate
    #    set_channel(datarate=<datarate>)       # Only change datarate
    #
    def set_channel(self, channel=None, datarate=None):
        # Normalize to get channel and datarate
        if type(channel) == tuple:
            new_channel = channel[0]
            # New datarate is from tuple if defined else from current datarate
            new_datarate = channel[1] if len(channel) == 2 else self._channel[1]

        else:
            # Fetch defaults from current channel values
            new_channel = self._channel[0] if channel == None else channel
            new_datarate = self._channel[1] if datarate == None else datarate

        if new_channel in self._channels:
            current_channel = self._channels[new_channel]

            info = current_channel['freq']
            self.write_register(_SX127x_REG_FREQ_MSB, info[0])
            self.write_register(_SX127x_REG_FREQ_MID, info[1])
            self.write_register(_SX127x_REG_FREQ_LSB, info[2])

            # If forcing default or new datarate is invalid, set to default (lowest) for channel
            if new_datarate not in current_channel['dr']:
                new_datarate = current_channel['dr'][0]

            self.set_bandwidth(self._data_rates[new_datarate]['bw'])
            self.set_spreading_factor(self._data_rates[new_datarate]['sf'])
            self.set_tx_power(self._data_rates[new_datarate]['tx'])
    
            self._channel = (new_channel, new_datarate)

            print("set channel to %d with dr %d" % (new_channel, new_datarate))

        else:
            raise Exception("Invalid channel: %s" % channel)

    def get_channel(self):
        return self._channel

    # Set bandwidth (limited by table specification)
    def set_bandwidth(self, bandwidth):
        bw = len(_BANDWIDTH_BINS)
        for i in range(len(_BANDWIDTH_BINS)):
            if bandwidth <= _BANDWIDTH_BINS[i]:
                bw = i
                break
    
        self.write_register(_SX127x_REG_MODEM_CONFIG_1,
                             (self.read_register(_SX127x_REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4))

        self._bandwidth = bandwidth

    def get_bandwidth(self):
        return self._bandwidth
    
    def set_spreading_factor(self, spreading_factor):
        self._spreading_factor = min(max(spreading_factor, 6), 12)
    
        # Set 'low data rate' flag if long symbol time otherwise clear it
        config3 = self.read_register(_SX127x_REG_MODEM_CONFIG_3)
        if 1000 / (self._bandwidth / 2**self._spreading_factor) > 16:
            config3 |= 0x08
        else:
            config3 &= ~0x08
        
        self.write_register(_SX127x_REG_MODEM_CONFIG_3, config3)
    
        self.write_register(_SX127x_REG_DETECTION_OPTIMIZE, 0xc5 if self._spreading_factor == 6 else 0xc3)
        self.write_register(_SX127x_REG_DETECTION_THRESHOLD, 0x0c if self._spreading_factor == 6 else 0x0a)
        self.write_register(_SX127x_REG_MODEM_CONFIG_2,
                            (self.read_register(_SX127x_REG_MODEM_CONFIG_2) & 0x0f) | ((self._spreading_factor << 4) & 0xf0))
    
    def get_spreading_factor(self):
        return self._spreading_factor

    def set_coding_rate(self, rate):
        # Limit it
        rate = min(max(rate, 5), 8)

        self.write_register(_SX127x_REG_MODEM_CONFIG_1, (self.read_register(_SX127x_REG_MODEM_CONFIG_1) & 0xF1) | (rate - 4) << 1)

    def set_preamble_length(self, length):
        self.write_register(_SX127x_REG_PREAMBLE_MSB, (length >> 8))
        self.write_register(_SX127x_REG_PREAMBLE_LSB, length)

    def set_enable_crc(self, enable=True):
        config = self.read_register(_SX127x_REG_MODEM_CONFIG_2)
        if enable:
            config |= 0x04
        else:
            config &= ~0x04
        self.write_register(_SX127x_REG_MODEM_CONFIG_2, config)

    # def set_hop_period(self, hop_period):
    #    self.write_register(_SX127x_REG_HOP_PERIOD, hop_period)

    def set_sync_word(self, sync):
        self.write_register(_SX127x_REG_SYNC_WORD, sync)

    def set_implicit_header(self, implicit_header = True):
        if implicit_header != self._current_implicit_header:
            self._current_implicit_header = implicit_header
            config = self.read_register(_SX127x_REG_MODEM_CONFIG_1)
            if implicit_header:
                config |= 0x01
            else:
                config &= ~0x01
            self.write_register(_SX127x_REG_MODEM_CONFIG_1, config)

    # Enable receive mode
    def enable_receive(self, length=0):
        self.set_implicit_header(length != 0)

        if length != 0:
            self.write_register(_SX127x_REG_PAYLOAD_LENGTH, length)

    # Receive interrupt comes here
    def _rxhandle_interrupt(self, event):
        # print("_rxhandle_interrupt fired on %s" % str(event))
        flags = self.read_register(_SX127x_REG_IRQ_FLAGS)
        self.write_register(_SX127x_REG_IRQ_FLAGS, flags)

        self._rx_interrupts += 1

        if flags & _SX127x_IRQ_RX_DONE:
            with self._lock:
                self.write_register(_SX127x_REG_FIFO_PTR, self.read_register(_SX127x_REG_RX_FIFO_CURRENT))
                if self._implicit_header:
                    length = self.read_register(_SX127x_REG_PAYLOAD_LENGTH)
                else:
                    length = self.read_register(_SX127x_REG_RX_NUM_BYTES)

                packet = self.read_buffer(_SX127x_REG_FIFO, length)

                if packet:
                    crc_ok = (flags & _SX127x_IRQ_PAYLOAD_CRC_ERROR) == 0
                    self.onReceive(packet, crc_ok, self.get_packet_rssi())
                else:
                    self._packets_memory_failed += 1

        else:
            print("_rxhandle_interrupt: not for us %02x" % flags)
  
    # FHSS interrupt - change channel
    # def _fhss_interrupt(self, event):
    #    self._write_register(_SX127x_FHSS_CHANNEL, next_channel)
    #    self._fhss_interrupts += 1


    def _txhandle_interrupt(self, event):
        flags = self.read_register(_SX127x_REG_IRQ_FLAGS)
        self.write_register(_SX127x_REG_IRQ_FLAGS, flags)

        self._tx_interrupts += 1

        # print("_txhandle_interrupt fired on %s %02x" % (str(event), flags))
        if flags & _SX127x_IRQ_TX_DONE:

            # Transmit interrupt
            with self._lock:
                # Discard current queue entry and get next packet to send
                packet = self.onTransmit()
                if packet:
                    # If a packet exists, send it.
                    if self._delay != 0:
                        timer(self._delay, self._transmit_packet_delay).start(packet)
                    else:
                        self.transmit_packet(packet)
                else:
                    # Other return to receive mode.
                    self.set_receive_mode()
        else:
            print("_txhandle_interrupt: not for us %02x" % flags)

    # Transmit packet from delay of timer
    def _transmit_packet_delay(self, timer, packet):
        self.transmit_packet(packet)
        gc.collect()

    def _start_packet(self, implicit_header = False):
        self.set_standby_mode()
        self.set_implicit_header(implicit_header)
        self.write_register(_SX127x_REG_FIFO_PTR, _TX_FIFO_BASE)
        self.write_register(_SX127x_REG_PAYLOAD_LENGTH, 0)

    def _write_packet(self, buffer):
        current = self.read_register(_SX127x_REG_PAYLOAD_LENGTH)
        size = min(len(buffer), (_SX127x_MAX_PACKET_LENGTH - _TX_FIFO_BASE - current))

        # print("_write_packet: writing %d: '%s'" % (size, buffer))

        self.write_buffer(_SX127x_REG_FIFO, buffer, size)

        # print("_write_packet: writing current %d + size %d = %d" % (current, size, current+size))
        self.write_register(_SX127x_REG_PAYLOAD_LENGTH, current + size)

        return size

    def transmit_packet(self, packet, implicit_header = False):
        # print("transmit_packet len %d lock %s" % (len(packet), self._lock.locked()))
        with self._lock:
            # print("Starting packet")
            self._start_packet(implicit_header)
            self._write_packet(packet)
            self.set_transmit_mode()
            # print("Unlocked")

    def _garbage_collect(self):
        gc.collect()

    def stop(self):
        # Disbable interrupts 
        self.write_register(_SX127x_REG_IRQ_FLAGS_MASK, 0xFF)


#endif /* CONFIG_LASTLINK_RADIO_SX127x_ENABLED */

