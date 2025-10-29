#include <Arduino.h>
#include <pindefs.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>
#include <HX711.h>
#include "Radio/Radio.h"



HX711 loadcell;
Radio radio;

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

uint32_t Time;

void setup() {
  pinMode(CS_SD,OUTPUT); digitalWrite(CS_SD,HIGH);
  pinMode(CS_LoRa, OUTPUT); digitalWrite(CS_LoRa,HIGH);
  pinMode(PY1, OUTPUT); digitalWrite(PY1,LOW);

  Serial.begin(115200);
  // while (!Serial); // Wait for Serial Console (comment out line if no computer)
  Time = millis();
  radio.init();

  // loadcell.begin(25,24);
  // loadcell.tare(20);
}

const float voltsPerBit = 0.000000002384185791015625;

uint8_t status, valvePos, valveBits;

void loop() {
  float timeElapsed = (float)(millis()-Time)/1000.f;

  valvePos = 0;
  if(valvePos > 25 && valvePos < 95) {
      valveBits = 25 + (uint8_t)((valvePos - 25)/12.5); //2^5 can store 0-31. Use (0,25) as raw percentage, then (26,31) represent 6 12.5% increments to 100%
  } else if (valvePos > 95) {
    valveBits = 31;
  } else { valveBits = valvePos; }

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

  status = valveBits << 3;
  status |= (igniterStatus | (armStatus << 1) | (qdStatus << 2));
  // dataPacket = (packet){timeElapsed,randFloat(750,850),randFloat(300,500),randFloat(1,800),status};
  // dataPacket = (packet){100.f,800.f,400.f,300.f,1};
  dataPacket = (packet){timeElapsed,0.f,0.f,0.f,status};

  uint8_t cmd = radio.tx((uint8_t *)&dataPacket);

  if(cmd == 'F' && armStatus) {
    Serial.println("FIRING COMMAND");
    digitalWrite(PY1,HIGH);
    delay(50);
    digitalWrite(PY1,LOW);
    armStatus = false;
  }
  else if(cmd == 'A') {
    armStatus = !armStatus;
    Serial.println("ARM COMMAND");
  }

  delay(50);
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 256;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
}