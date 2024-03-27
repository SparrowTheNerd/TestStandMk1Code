#include <Arduino.h>
#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS 21
#define RFM95_INT digitalPinToInterrupt(2)
#define RFM95_RST 1

RH_RF95 rf95(RFM95_CS, RFM95_INT);

struct packet {
  float tankPrs, combnPrs, force;
} dataPacket;

float randFloat(float LO, float HI) {
    return LO + (float)(rand()) /( (float)(RAND_MAX/(HI-LO)));
}

void setup() {
  pinMode(23,OUTPUT); digitalWrite(23,HIGH);
  pinMode(22,OUTPUT); digitalWrite(22,HIGH);

  Serial.begin(115200);
  // while (!Serial); // Wait for Serial Console (comment out line if no computer)

  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  SPI.begin();
  if (!rf95.init())
    Serial.println("init failed");  
  else {Serial.println("init success");}
  rf95.setFrequency(915.0); //set frequency to 915MHz
  rf95.setTxPower(20,false); //set the transmit power to 20dBm using PA_BOOST
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // You can change the modulation parameters with eg
  rf95.setModemConfig(RH_RF95::Bw500Cr45Sf128);
}

void loop() {


  if(rf95.mode() != rf95.RHModeTx) {
    dataPacket = (packet){randFloat(750,850),randFloat(300,500),randFloat(0,800)};
    
    char radiopacket[sizeof(dataPacket)];
    memcpy(radiopacket,&dataPacket,sizeof(dataPacket));
    rf95.send((uint8_t *)radiopacket, strlen(radiopacket));
    Serial.print("Sent: "); Serial.print(dataPacket.tankPrs,5); Serial.print(" "); Serial.print(dataPacket.combnPrs,5); Serial.print(" "); Serial.println(dataPacket.force,5);
  }
}