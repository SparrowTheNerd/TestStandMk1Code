#include <Arduino.h>
#include <pindefs.h>
#include <STM32FreeRTOS.h>
#include <SPI.h>
#include <HX711.h>
#include "Radio/Radio.h"
#include <Servo.h>

HX711 loadcell;
Radio radio;
Servo servo1;

bool armStatus = false;
bool qdStatus = false;
bool igniterStatus = false;

float randFloat(float LO, float HI) {
    return LO + (float)(rand()) /( (float)(RAND_MAX/(HI-LO)));
}

SemaphoreHandle_t xSerialSemaphore;

uint32_t startTime, servoTimer, servoStart = 0;


const float voltsPerBit = 0.000000002384185791015625;

uint8_t status, valvePos, valveBits;

bool servoOn = false;

void TaskRadio(void *pvParameters);
void TaskData(void *pvParameters);
void TaskServo(void *pvParameters);

void setup() {
  // pinMode(PY1, OUTPUT); digitalWrite(PY1,LOW);
  pinMode(CONT, INPUT);
  SerialUSB.begin();
  // while (!Serial); // Wait for Serial Console (comment out line if no computer)
  startTime = millis();

  xTaskCreate(
    TaskRadio
    ,  "Radio Task"   // A name just for humans
    ,  512  // This stack size can be checked & adjusted by reading Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );
  xTaskCreate(
    TaskData
    ,  "Data Task"   // A name just for humans
    ,  512  // This stack size can be checked & adjusted by reading Highwater
    ,  NULL
    ,  3  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );
  xTaskCreate(
    TaskServo
    ,  "Servo Task"   // A name just for humans
    ,  512  // This stack size can be checked & adjusted by reading Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );
  vTaskStartScheduler();
  Serial.println("Insufficient RAM");
  while(1);
}

float timeElapsed;

/* ===== TASKS ===== */

void TaskRadio(void *pvParameters __attribute__((unused))) {
  radio.init();
  for(;;) {
    timeElapsed = (float)(millis()-startTime)/1000.f;
    // dataPacket = (packet){timeElapsed,randFloat(750,850),randFloat(300,500),randFloat(1,800),status};
    // dataPacket = (packet){100.f,800.f,400.f,300.f,1};
    radio.dataPacket = (Radio::packet){timeElapsed,randFloat(750,850),randFloat(300,500),randFloat(1,800),status};
    uint8_t cmd = radio.tx();

    switch (cmd) {
      case 'F':
        if (armStatus) {
          Serial.println("FIRING COMMAND");
          digitalWrite(PY1,HIGH);
          delay(50);
          digitalWrite(PY1,LOW);
          armStatus = false;
        }
        break;
      case 'A':
        armStatus = !armStatus;
        Serial.println("ARM COMMAND");
        break;
      case '1':
        Serial.println("Fill state command");
        break;
      case '2':
        Serial.println("Hold state command");
        break;
      case '3':
        Serial.println("Firing/safe command");
        break;
      default:
        break;
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}
void TaskData(void *pvParameters __attribute__((unused))) {
  loadcell.begin(25,24);
  loadcell.tare(20);
  
  for(;;) {
    valvePos = 0;
    if(valvePos > 25 && valvePos < 95) {
        valveBits = 25 + (uint8_t)((valvePos - 25)/12.5); //2^5 can store 0-31. Use (0,25) as raw percentage, then (26,31) represent 6 12.5% increments to 100%
    } else if (valvePos > 95) {
      valveBits = 31;
    } else { valveBits = valvePos; }
    igniterStatus = digitalRead(CONT);
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
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void TaskServo(void *pvParameters __attribute__((unused))) {
  servo1.attach(SERVO1); servo1.write(90); //stationary

  for(;;) {
    if(servoOn) {
      servo1.write(0); //move CCW
      servoOn = false;
    } else {
      servo1.write(180); //move CW
      servoOn = true;
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void loop() {
  // Empty. Things are done in Tasks.
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