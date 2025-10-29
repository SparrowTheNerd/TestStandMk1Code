#include "Radio.h"
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>
#include "pindefs.h"

#define CS PC2
#define INT PC12

#define CLIENT_ADDRESS 17
#define SERVER_ADDRESS 41

RHHardwareSPI spi;
RH_RF95 rf95(CS_LoRa, INT_LoRa, spi);
RHReliableDatagram manager(rf95, CLIENT_ADDRESS);

uint32_t startTime;

void Radio::init() {
   pinMode(CS_LoRa, OUTPUT);
   digitalWrite(CS_LoRa, HIGH); //pull other chip selects high
   pinModeAF(PA6,GPIO_AF5_SPI1); SPI.setMISO(PA6);
   pinModeAF(PA7,GPIO_AF5_SPI1); SPI.setMOSI(PA7);
   pinModeAF(PA5,GPIO_AF5_SPI1); SPI.setSCLK(PA5);
   // spi.setFrequency(RHGenericSPI::Frequency8MHz);
   SPI.setClockDivider(SPI_CLOCK_DIV16);
   SPI.begin();
   

   // rf95.init();
   manager.init();
   rf95.setFrequency(924); //set frequency to 915MHz
   rf95.setTxPower(20,false); //set the transmit power to 20dBm using PA_BOOST
   rf95.setModemConfig(RH_RF95::Bw500Cr45Sf128);
   Serial.println("Radio initialized");
}

uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
uint32_t nackFreq = 0;

uint8_t Radio::tx(uint8_t* dataPacket) {
   uint8_t len = sizeof(buf);
   uint8_t from;
   if (!manager.sendtoWait((uint8_t *)&dataPacket, sizeof(dataPacket), SERVER_ADDRESS))
   {
      Serial.println("NACK");
      Serial.println(nackFreq);
      nackFreq = 0;
      return 0;
   }
   else {
      if (manager.recvfromAckTimeout(buf, &len, 1000, &from)) {
         return buf[0];
      }
      else {
         return 1;
      }
   }
}

void Radio::pinModeAF(int ulPin, uint32_t Alternate) {
   int pn = digitalPinToPinName(ulPin);

   if (STM_PIN(pn) < 8) {
      LL_GPIO_SetAFPin_0_7( get_GPIO_Port(STM_PORT(pn)), STM_LL_GPIO_PIN(pn), Alternate);
   } else {
      LL_GPIO_SetAFPin_8_15(get_GPIO_Port(STM_PORT(pn)), STM_LL_GPIO_PIN(pn), Alternate);
   }

   LL_GPIO_SetPinMode(get_GPIO_Port(STM_PORT(pn)), STM_LL_GPIO_PIN(pn), LL_GPIO_MODE_ALTERNATE);
}