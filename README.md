# LastLink

LastLink is an implementation of a LoRa-based mesh network (though in no way resembling LoRa or LoRaWAN
transport protocols.)  An ad-hoc system of devices forms a network of small, low powered devices, with 'work'
nodes scattered as appropriate.  The initial version supports the Heltec (and others) SX127x modules.

Curently, only USA emission is supported.  Though frequencies in other domains are programmable, the drivers
do not yet support (for example) EU regulations on duty-cycle or dwell time.

LastLink is designed to be a battery powered and solar-rechargable, with WiFi accessable nodes, capable of
providing limited community communication among users with portable web-capable wifi-equipped devices.  The
basic interface is through a web page accessable on each WiFi-equipped node, with one or more 'work' nodes used
to store and distribute community-wide messages. The 'work' nodes in the initial release will support a
public/private messaging system.  Other types, such as limit Internet gateways, could later provide messaging
access to the Internet at large.  The API is 'REST' based with a javascript client for doing the 'heavy lifting.'  

The user web interface provides a mailing list format with public and private messages and distribution lists.

The user web browser interface supports privately-signed SSL keys (there may be no Internet, in the usual sense,
so verifying through public 'authority' servers may be unavailabl in times of disaster.)  End-to-end encryption
is provided by a layer of public or private keys.  The low-level data transport is byte-agnostic and can
transmit streams of octets between a source and destination without interpretation, be they encrypted or
otherwise.
