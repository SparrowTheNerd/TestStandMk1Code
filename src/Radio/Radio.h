#include <Arduino.h>
#ifndef RADIO_H
#define RADIO_H

class Radio {
public:
    void init();
    uint8_t tx(uint8_t* dataPacket);
    // #pragma pack(push,1)
    // #pragma pack(pop)

private:
    // Private members
    void pinModeAF(int ulPin, uint32_t Alternate);
};

#endif // RADIO_H
