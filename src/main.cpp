/*Biblioteca do Arduino*/
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <HX711_ADC.h>
#if defined(ESP8266) || defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif

/*Bibliotecas FreeRTOS */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"

/*Mapeamento de pinos*/
#define HX711Dout 4
#define HX711Sck 5
#define btnStart 15
#define btnOrganic 2
#define btnGlass 16
#define btnMetal 17
#define btnPaper 18
#define btnPlastic 19
#define btnTare 23
#define btnCalibrate 23

/*Construtores*/
HX711_ADC LoadCell(HX711Dout, HX711Sck);
LiquidCrystal_I2C lcd(0x27, 16, 2);

/*Variáveis Globais*/
const int calVal_eepromAdress = 0;
unsigned long t = 0;

/*Variáveis para armazenamento do handle das tasks*/
TaskHandle_t taskReadWeightHandle = NULL;

// QueueHandle_t xFilaRGBRed;

// SemaphoreHandle_t xSemaphoreMasterMode;

// TimerHandle_t xTimerClose;

/*Protótipos das Tasks*/
void vTaskReadWeight(void *pvParameters);

// void callBackTimerClose(TimerHandle_t xTimer);

/*Funções*/
void calibrate(void);
void initButtons(void);

// void btnMasterISRCallBack();

void setup()
{
  Serial.begin(115200);
  vTaskDelay(pdMS_TO_TICKS(10));
  initButtons();

  Serial.println();
  Serial.println("Starting...");

  /*Criação Queues*/
  // xFilaRGBRed = xQueueCreate(1, sizeof(int));

  /*Criação Semaphores*/
  // xSemaphoreMasterMode = xSemaphoreCreateBinary();

  /*if(xSemaphoreMasterMode == NULL){
   Serial.println("Não foi possível criar o semaforo!");
   ESP.restart();
  }*/

  /*Criação Timers*/
  // xTimerClose = xTimerCreate("TIMER CLOSE",pdMS_TO_TICKS(5000),pdTRUE,0,callBackTimerClose);
  // xTimerStart(xTimerClose, 0);

  /*Criação Interrupções*/
  // attachInterrupt(digitalPinToInterrupt(btnStart), btnMasterISRCallBack, FALLING);

  /*criação Tasks*/
  if (xTaskCreatePinnedToCore(vTaskReadWeight, "TASK READ WEIGHT", configMINIMAL_STACK_SIZE + 4096, NULL, 1, &taskReadWeightHandle, APP_CPU_NUM) == pdFAIL)
  {
    Serial.println("Não foi possível criar a Task Read Weight");
    ESP.restart();
  }
  vTaskSuspend(taskReadWeightHandle);

  LoadCell.begin();
  // LoadCell.setReverseOutput(); //uncomment to turn a negative output value to positive
  unsigned long stabilizingtime = 2000; // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
  boolean _tare = true;                 // set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag())
  {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1)
      ;
  }
  else
  {
    LoadCell.setCalFactor(1.0); // user set calibration value (float), initial value 1.0 may be used for this sketch
    Serial.println("Startup is complete");
  }
  while (!LoadCell.update())
    ;
  calibrate(); // start calibration procedure

  lcd.init();
  lcd.backlight();

  vTaskResume(taskReadWeightHandle);
}

void loop()
{
  vTaskDelay(pdMS_TO_TICKS(1000));
}

//.......................Tasks.............................
void vTaskReadWeight(void *pvParameters)
{
  while (1)
  {
    static boolean newDataReady = 0;
    const int serialPrintInterval = 500; // increase value to slow down serial print activity

    // check for new data/start next conversion:
    if (LoadCell.update())
      newDataReady = true;

    // get smoothed value from the dataset:
    if (newDataReady)
    {
      if (millis() > t + serialPrintInterval)
      {
        float i = LoadCell.getData();
        Serial.print("Load_cell output val: ");
        Serial.println(i);

        lcd.clear();
        lcd.print("Peso:");
        lcd.setCursor(0, 1);
        lcd.print(i / 1000);
        lcd.print(" kg");

        newDataReady = 0;
        t = millis();
      }
    }

    // receive command from serial terminal
    if (Serial.available() > 0)
    {
      char inByte = Serial.read();
      if (inByte == 't')
        LoadCell.tareNoDelay(); // tare
      else if (inByte == 'r')
        calibrate(); // calibrate
    }

    // check if last tare operation is complete
    if (LoadCell.getTareStatus() == true)
    {
      Serial.println("Tare complete");
    }
  }
}

//.......................Timers.............................
/*void callBackTimerClose(TimerHandle_t xTimer){

}*/

//......................ISR.................................
/*void btnMasterISRCallBack(){

}*/

//......................Funções.............................
void calibrate(void)
{
  Serial.println("***");
  Serial.println("Start calibration:");
  Serial.println("Place the load cell an a level stable surface.");
  Serial.println("Remove any load applied to the load cell.");
  Serial.println("Send 't' from serial monitor to set the tare offset.");

  boolean _resume = false;
  while (_resume == false)
  {
    LoadCell.update();
    if (Serial.available() > 0)
    {
      if (Serial.available() > 0)
      {
        char inByte = Serial.read();
        if (inByte == 't')
          LoadCell.tareNoDelay();
      }
    }
    if (LoadCell.getTareStatus() == true)
    {
      Serial.println("Tare complete");
      _resume = true;
    }
  }

  Serial.println("Now, place your known mass on the loadcell.");
  Serial.println("Then send the weight of this mass (i.e. 100.0) from serial monitor.");

  float known_mass = 0;
  _resume = false;
  while (_resume == false)
  {
    LoadCell.update();
    if (Serial.available() > 0)
    {
      known_mass = Serial.parseFloat();
      if (known_mass != 0)
      {
        Serial.print("Known mass is: ");
        Serial.println(known_mass);
        _resume = true;
      }
    }
  }

  LoadCell.refreshDataSet();                                          // refresh the dataset to be sure that the known mass is measured correct
  float newCalibrationValue = LoadCell.getNewCalibration(known_mass); // get the new calibration value

  Serial.print("New calibration value has been set to: ");
  Serial.print(newCalibrationValue);
  Serial.println(", use this as calibration value (calFactor) in your project sketch.");
  Serial.print("Save this value to EEPROM adress ");
  Serial.print(calVal_eepromAdress);
  Serial.println("? y/n");

  _resume = false;
  while (_resume == false)
  {
    if (Serial.available() > 0)
    {
      char inByte = Serial.read();
      if (inByte == 'y')
      {
#if defined(ESP8266) || defined(ESP32)
        EEPROM.begin(512);
#endif
        EEPROM.put(calVal_eepromAdress, newCalibrationValue);
#if defined(ESP8266) || defined(ESP32)
        EEPROM.commit();
#endif
        EEPROM.get(calVal_eepromAdress, newCalibrationValue);
        Serial.print("Value ");
        Serial.print(newCalibrationValue);
        Serial.print(" saved to EEPROM address: ");
        Serial.println(calVal_eepromAdress);
        _resume = true;
      }
      else if (inByte == 'n')
      {
        Serial.println("Value not saved to EEPROM");
        _resume = true;
      }
    }
  }

  Serial.println("End calibration");
  Serial.println("***");
  Serial.println("To re-calibrate, send 'r' from serial monitor.");
  Serial.println("For manual edit of the calibration value, send 'c' from serial monitor.");
  Serial.println("***");
}

void initButtons(void)
{
  pinMode(btnStart, INPUT_PULLUP);
  pinMode(btnOrganic, INPUT_PULLUP);
  pinMode(btnGlass, INPUT_PULLUP);
  pinMode(btnMetal, INPUT_PULLUP);
  pinMode(btnPaper, INPUT_PULLUP);
  pinMode(btnPlastic, INPUT_PULLUP);
  pinMode(btnTare, INPUT_PULLUP);
  pinMode(btnCalibrate, INPUT_PULLUP);
}