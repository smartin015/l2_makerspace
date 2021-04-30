#ifndef __LOGGER_H__
#define __LOGGER_H__

#include <stdio.h>
#include <string.h>

#define ERROR_LEVEL     0x01
#define INFO_LEVEL      0x02
#define DEBUG_LEVEL     0x03

#ifndef LOG_LEVEL
#define LOG_LEVEL   DEBUG_LEVEL
#endif

#define PRINTFUNCTION(...)      printf(__VA_ARGS__)
#define NEWLINE     "\n"

#if LOG_LEVEL >= DEBUG_LEVEL
#define LOG_DEBUG(message, args...)     PRINTFUNCTION("D:" message NEWLINE, ## args)
#else
#define LOG_DEBUG(message, args...)
#endif

#if LOG_LEVEL >= INFO_LEVEL
#define LOG_INFO(message, args...)      PRINTFUNCTION("I:" message NEWLINE, ## args)
#else
#define LOG_INFO(message, args...)
#endif

#if LOG_LEVEL >= ERROR_LEVEL
#define LOG_ERROR(message, args...)     PRINTFUNCTION("E:" message NEWLINE, ## args)
#else
#define LOG_ERROR(message, args...)
#endif

#endif // __LOGGER_H__
