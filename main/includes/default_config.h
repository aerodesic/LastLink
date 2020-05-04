/*
 * default_config.h
 *
 * Default configuration for LastLink node.
 */
#ifndef __default_config_h_included
#define __default_config_h_included

#define DEFAULT_CONFIG {        \
    "configversion=4",          \
    "[lastlink]",               \
    "    address=1",            \
    "    flags=0x00",           \
    "    announce=0",           \
    "    receive_only_from=",   \
    "    listen_only=0",        \
    "[end]",                    \
    "[display]",                \
    "    contrast=128",         \
    "[end]",                    \
    NULL,                       \
}

#endif /* __default_config_h_included */

