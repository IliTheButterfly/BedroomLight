#ifndef COMMON_H
#define COMMON_H

#include <Arduino.h>

#if defined(DEBUG) && DEBUG > 0

#define dbg(...) Serial.print(__VA_ARGS__)
#define dbgln(...) Serial.println(__VA_ARGS__)
#define dbginit(baud) Serial.begin(baud)
#define dbgstream (Serial)
#define dbgflush() Serial.flush()

#else // defined(DEBUG) && DEBUG > 0

#define dbg(...)
#define dbgln(...)
#define dbginit(baud)
#define dbgstream (false)
#define dbgflush()

#endif // defined(DEBUG) && DEBUG > 0

#endif // COMMON_H