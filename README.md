# LastLink

Implementation of a LoRa-based mesh radio network for extremely small and low powered devices.
The initial version is designed to support the Heltec (and others) SX127x modules.  Curently support
for only USA emission is supported.  Though frequencies in other domains are programmable, the drivers
do not yet support (for example) EU regulations on duty-cycle or dwell time.

LastLink is designed to be a battery powered network of WiFi accessable nodes, capably of providing limited
community communication among users with portable web-capable wifi-equipped devices.  The basic interface
is through a web page accessable on each WiFi-equipped node, with one or more 'root' nodes used to storing
and distributing community-wide messages.  The messages can be private or public, much like email and mailing
lists of yore.

