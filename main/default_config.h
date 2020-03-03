/*
 * default_config.h
 *
 * Default configuration for LastLink node.
 */
#ifndef __default_config_h_included
#define __default_config_h_included

#define DEFAULT_CONFIG {        \
    "configversion=2",		\
    "zip=1",			\
    "zap=2",			\
    "[section1]",		\
    "    this=that",		\
    "    that=this",		\
    "    [section2]",		\
    "       blah=1",		\
    "       blot=2",		\
    "       [section3]",        \
    "          only=garbage",   \
    "       [end]",             \
    "    [end]",                \
    "[end]",                    \
    "zorch=3",                  \
    NULL,                       \
}

#endif /* __default_config_h_included */

