#ifndef WATCHDOG_H
#define WATCHDOG_H
#include <Arduino.h>

class Watchdog
{
private:
    unsigned long timeout;
    unsigned long lastResetTime;

public:
    Watchdog(unsigned long timeoutMillis) : timeout(timeoutMillis), lastResetTime(millis()) {}

    void reset()
    {
        lastResetTime = millis();
    }

    bool isExpired()
    {
        return (millis() - lastResetTime) >= timeout;
    }
};

#endif // WATCHDOG_H