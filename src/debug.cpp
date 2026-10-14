#include "config.h"
#include <stdarg.h>

void debugf(const char *fmt, ...) {
    if (!debug)
        return;
    char buf[384];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    Serial.print(buf);
}

void debugln(const char *s) {
    if (debug)
        Serial.println(s);
}
