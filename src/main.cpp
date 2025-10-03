#include <Arduino.h>
#include <pindefs.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>
#include <HX711.h>

#define CLIENT_ADDR 17
#define SERVER_ADDR 41

RHHardwareSPI spi;
RH_RF95 rf95(CS_LoRa, INT_LoRa, spi);
RHReliableDatagram manager(rf95, CLIENT_ADDR);

HX711 loadcell;

bool armStatus = false;
bool qdStatus = false;
bool igniterStatus = false;

struct packet {
  float timeStamp, tankPrs, combnPrs, force;
  uint8_t status;
  //2^5 can store 0-31. Use (0,25) as raw percentage, then (26,31) represent subsequent 12.5% increments to 100%
  //the remaining 3 bits can be used for arming status, QD status, and igniter continuity
} dataPacket;

float randFloat(float LO, float HI) {
    return LO + (float)(rand()) /( (float)(RAND_MAX/(HI-LO)));
}

uint32_t txCounter;
uint32_t time;

void setup() {
  pinMode(CS_SD,OUTPUT); digitalWrite(CS_SD,HIGH);
  pinMode(CS_LoRa, OUTPUT); digitalWrite(CS_LoRa,HIGH);
  pinMode(PY1, OUTPUT); digitalWrite(PY1,LOW);

  Serial.begin(115200);
  // while (!Serial); // Wait for Serial Console (comment out line if no computer)
  spi.setDataMode(spi.DataMode0);
  spi.setBitOrder(spi.BitOrderMSBFirst);
  spi.setFrequency(spi.Frequency1MHz);
  spi.begin();

  if (!manager.init())
    Serial.println("init failed");
  else Serial.println("init success");

  rf95.setFrequency(924); //set frequency to 915MHz
  rf95.setTxPower(0,false); //set the transmit power to 20dBm using PA_BOOST

  // You can change the modulation parameters with eg
  rf95.setModemConfig(RH_RF95::Bw500Cr45Sf128);
  txCounter = time = millis();

  // loadcell.begin(25,24);
  // loadcell.tare(20);


}

const float voltsPerBit = 0.000000002384185791015625;

uint8_t status, valvePos, valveBits;

uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];

uint32_t nackFreq = 0;

void loop() {
    float timeElapsed = (float)(millis()-time)/1000.f;

  valvePos = 0;
  if(valvePos > 25 && valvePos < 95) {
      valveBits = 25 + (uint8_t)((valvePos - 25)/12.5); //2^5 can store 0-31. Use (0,25) as raw percentage, then (26,31) represent 6 12.5% increments to 100%
  } else if (valvePos > 95) {
    valveBits = 31;
  } else { valveBits = valvePos; }

  status = valveBits << 3;
  status |= (igniterStatus | (armStatus << 1) | (qdStatus << 2));
  // dataPacket = (packet){timeElapsed,randFloat(750,850),randFloat(300,500),randFloat(1,800),status};
  // dataPacket = (packet){100.f,800.f,400.f,300.f,1};
  dataPacket = (packet){timeElapsed,0.f,0.f,0.f,status};
  nackFreq++;
  uint8_t len = sizeof(buf);
  uint8_t from;

  if (!manager.sendtoWait((uint8_t *)&dataPacket, sizeof(dataPacket), SERVER_ADDR))
  {
    Serial.println("NACK");
    Serial.println(nackFreq);
    nackFreq = 0;
  }
  else {
    if (manager.recvfromAckTimeout(buf, &len, 1000, &from)) {
      if(buf[0] == 'F' && armStatus) {
        Serial.println("FIRING COMMAND");
        digitalWrite(PY1,HIGH);
        delay(50);
        digitalWrite(PY1,LOW);
        armStatus = false;
      }
      if(buf[0] == 'A') {
        armStatus = !armStatus;
        Serial.println("ARM COMMAND");
      }
    }
  }
  delay(50);
  
  // if (loadcell.wait_ready_timeout(1000)) {
  //   long reading = loadcell.read();
  //   Serial.println(reading);
  // } else {
  //   Serial.println("HX711 not found.");
  // }
  // long reading = loadcell.read();
  // // Serial.println(reading);
  // float volts = (float)reading*voltsPerBit;
  // float kgs = 1000.f * volts / 0.02;
  // Serial.println(kgs-2,3);
  // delay(5);
}