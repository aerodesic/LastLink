# LastLink

Update: The current work is moving toward integration of the SX126x (both 1 and 2) radios into the mix.
Routing and packet delivery are working adequatly (I'l pubish more on the routing protocol when a 'release' is
is about to happen.  Routing is based on a widely-used directed probe with redunant path trimming.  In a small
network, it's reasonably packet effiencent without demanding too much on local caching (though there is some of
that involved.)

Nodes can support up to N radios with routing between the networks provided by each radio (currently limited to
eight, but can be easily changed.)  Current hardware limits N to 1 as we are using various Heltec, TTGO/LilyGO,
etc. platforms.  We have visions of a master head-ends with 8 simultaneous radio networks running in order to bridge
outlying subnets.  Software is written with this in mind, including routing between/among subnets, but no testing
has been applied.  For now, N=1.

Nodes are intended to be either 'links' in the network with HTTPS access to message services (possibe BT later) or
'worker' units (which also function as 'links'.  'Workers' contain local functionality such as message storage,
retrieval and display as well as potential 'chat' services.  These will be ad-hoc.

Message security is currenty non-existant but will be based on one of severa light-weight packet-level (not streaming)
protocols.  Ultimatey, 'stronger' encryption will probably require support on user terminal equipment (cell/table)
in order to avoid disclosure of private keys.  This API is under development but not included herein.

============================================================================================================
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

The user web interface provides a mailing list format with public and private messages and distribution lists and
supports privately-signed SSL key.  There may be no Internet, in the usual sense, so verifying through public 
'authority' servers may be unavailable in times of disaster.

End-to-end encryption is provided by a layer of public or private keys. The low-level data transport is byte-agnostic
and can transmit streams of octets between a source and destination without interpretation, be they encrypted or
otherwise.
