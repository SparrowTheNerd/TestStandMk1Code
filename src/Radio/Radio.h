#include <Arduino.h>
#ifndef RADIO_H
#define RADIO_H

class Radio {
public:
    void init();
    uint8_t tx();
    // #pragma pack(push,1)
    // #pragma pack(pop)

    struct packet {
    float timeStamp, tankPrs, combnPrs, force;
    uint8_t status;
    //2^5 can store 0-31. Use (0,25) as raw percentage, then (26,31) represent subsequent 12.5% increments to 100%
    //the remaining 3 bits can be used for arming status, QD status, and igniter continuity
    } dataPacket;

private:
    // Private members
    void pinModeAF(int ulPin, uint32_t Alternate);
};

#endif // RADIO_H
