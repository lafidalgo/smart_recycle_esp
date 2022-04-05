/*Biblioteca do Arduino*/
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ArduinoJson.h>
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
#define btnCalibrate 25

/*Constantes*/
#define weightReference 2000 // 2 kg

/*Construtores*/
HX711_ADC LoadCell(HX711Dout, HX711Sck);
LiquidCrystal_I2C lcd(0x27, 16, 2);

/*Variáveis Globais*/
const int calVal_eepromAdress = 0;
unsigned long t = 0;

bool btnDebounce = 0;
bool readWeightStarted = 0;
bool tareStarted = 0;
bool calibrationStarted = 0;

/*Variáveis para armazenamento do handle das tasks*/
TaskHandle_t taskReadWeightHandle = NULL;
TaskHandle_t taskSendWeightHandle = NULL;
TaskHandle_t taskTareHandle = NULL;
TaskHandle_t taskCalibrateHandle = NULL;
TaskHandle_t taskReadTimeoutHandle = NULL;

QueueHandle_t xFilaTrashType;
QueueHandle_t xFilaTrashWeight;

// SemaphoreHandle_t xSemaphoreSendWeight;

TimerHandle_t xTimerBtnDebounce;
TimerHandle_t xTimerReadWeightTimeout;

/*Protótipos das Tasks*/
void vTaskReadWeight(void *pvParameters);
void vTaskSendWeight(void *pvParameters);
void vTaskTare(void *pvParameters);
void vTaskCalibrate(void *pvParameters);
void vTaskReadTimeout(void *pvParameters);

/*Timer Callbacks*/
void callBackTimerBtnDebounce(TimerHandle_t xTimer);
void callBackTimerReadWeightTimeout(TimerHandle_t xTimer);

/*Funções*/
void calibrate(void);
void initButtons(void);
void publishMessage(int trashType, float trashWeight);

void btnStartISRCallBack();
void btnOrganicISRCallBack();
void btnGlassISRCallBack();
void btnMetalISRCallBack();
void btnPaperISRCallBack();
void btnPlasticISRCallBack();
void btnTareISRCallBack();
void btnCalibrateISRCallBack();

void setup()
{
  Serial.begin(115200);
  vTaskDelay(pdMS_TO_TICKS(10));
  initButtons();

  Serial.println();
  Serial.println("Starting...");

  /*Criação Queues*/
  xFilaTrashType = xQueueCreate(1, sizeof(int));
  xFilaTrashWeight = xQueueCreate(1, sizeof(float));

  /*Criação Semaphores*/
  // xSemaphoreSendWeight = xSemaphoreCreateBinary();

  /*if(xSemaphoreSendWeight == NULL){
   Serial.println("Não foi possível criar o semaforo!");
   ESP.restart();
  }*/

  /*Criação Timers*/
  xTimerBtnDebounce = xTimerCreate("TIMER BTN DEBOUNCE", pdMS_TO_TICKS(500), pdTRUE, 0, callBackTimerBtnDebounce);
  xTimerReadWeightTimeout = xTimerCreate("TIMER READ WEIGHT TIMEOUT", pdMS_TO_TICKS(10000), pdTRUE, 0, callBackTimerReadWeightTimeout);

  /*Criação Interrupções*/
  attachInterrupt(digitalPinToInterrupt(btnStart), btnStartISRCallBack, FALLING);
  attachInterrupt(digitalPinToInterrupt(btnOrganic), btnOrganicISRCallBack, FALLING);
  attachInterrupt(digitalPinToInterrupt(btnGlass), btnGlassISRCallBack, FALLING);
  attachInterrupt(digitalPinToInterrupt(btnMetal), btnMetalISRCallBack, FALLING);
  attachInterrupt(digitalPinToInterrupt(btnPaper), btnPaperISRCallBack, FALLING);
  attachInterrupt(digitalPinToInterrupt(btnPlastic), btnPlasticISRCallBack, FALLING);
  attachInterrupt(digitalPinToInterrupt(btnTare), btnTareISRCallBack, FALLING);
  attachInterrupt(digitalPinToInterrupt(btnCalibrate), btnCalibrateISRCallBack, FALLING);

  /*Criação Tasks*/
  if (xTaskCreatePinnedToCore(vTaskReadWeight, "TASK READ WEIGHT", configMINIMAL_STACK_SIZE + 4096, NULL, 1, &taskReadWeightHandle, APP_CPU_NUM) == pdFAIL)
  {
    Serial.println("Não foi possível criar a Task Read Weight");
    ESP.restart();
  }
  vTaskSuspend(taskReadWeightHandle);

  if (xTaskCreatePinnedToCore(vTaskSendWeight, "TASK SEND WEIGHT", configMINIMAL_STACK_SIZE + 4096, NULL, 1, &taskSendWeightHandle, APP_CPU_NUM) == pdFAIL)
  {
    Serial.println("Não foi possível criar a Task Send Weight");
    ESP.restart();
  }
  vTaskSuspend(taskSendWeightHandle);

  if (xTaskCreatePinnedToCore(vTaskTare, "TASK TARE", configMINIMAL_STACK_SIZE + 4096, NULL, 1, &taskTareHandle, APP_CPU_NUM) == pdFAIL)
  {
    Serial.println("Não foi possível criar a Task Tare");
    ESP.restart();
  }
  vTaskSuspend(taskTareHandle);

  if (xTaskCreatePinnedToCore(vTaskCalibrate, "TASK CALIBRATE", configMINIMAL_STACK_SIZE + 4096, NULL, 1, &taskCalibrateHandle, APP_CPU_NUM) == pdFAIL)
  {
    Serial.println("Não foi possível criar a Task Calibrate");
    ESP.restart();
  }
  vTaskSuspend(taskCalibrateHandle);

  if (xTaskCreatePinnedToCore(vTaskReadTimeout, "TASK READ TIMEOUT", configMINIMAL_STACK_SIZE + 4096, NULL, 1, &taskReadTimeoutHandle, APP_CPU_NUM) == pdFAIL)
  {
    Serial.println("Não foi possível criar a Task Read Timeout");
    ESP.restart();
  }
  vTaskSuspend(taskReadTimeoutHandle);

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
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Balanca");
  lcd.setCursor(0, 1);
  lcd.print("iniciada");
  vTaskDelay(pdMS_TO_TICKS(3000));
  lcd.clear();
  lcd.noBacklight();
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
    const int serialPrintInterval = 200; // increase value to slow down serial print activity

    // check for new data/start next conversion:
    if (LoadCell.update())
      newDataReady = true;

    // get smoothed value from the dataset:
    if (newDataReady)
    {
      float i = LoadCell.getData();
      Serial.print("Load_cell output val: ");
      Serial.println(i);
      xQueueOverwrite(xFilaTrashWeight, &i);
      lcd.backlight();
      lcd.clear();
      lcd.print("Peso:");
      lcd.setCursor(0, 1);
      lcd.print(i / 1000);
      lcd.print(" kg");

      newDataReady = 0;
    }

    vTaskDelay(pdMS_TO_TICKS(serialPrintInterval));
  }
}

void vTaskSendWeight(void *pvParameters)
{
  int trashType = 0; // 1 - Organic; 2 - Glass; 3 - Metal; 4 - Paper; 5 - Plastic
  float trashWeight = 0;
  while (1)
  {
    Serial.println("Task send weight");
    // xSemaphoreTake(xSemaphoreSendWeight, portMAX_DELAY);
    vTaskSuspend(taskReadWeightHandle);
    xTimerStop(xTimerReadWeightTimeout, 0);
    xQueueReceive(xFilaTrashType, &trashType, portMAX_DELAY);
    xQueueReceive(xFilaTrashWeight, &trashWeight, portMAX_DELAY);

    publishMessage(trashType, trashWeight);

    lcd.setCursor(0, 0);
    lcd.print("Peso enviado");
    lcd.setCursor(0, 1);
    lcd.print("com sucesso");

    vTaskDelay(pdMS_TO_TICKS(3000));
    lcd.clear();
    lcd.noBacklight();

    readWeightStarted = false;

    vTaskSuspend(taskSendWeightHandle);
  }
}

void vTaskTare(void *pvParameters)
{
  while (1)
  {
    Serial.println("Task tare");

    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("Balanca tarada");
    lcd.setCursor(0, 1);
    lcd.print("com sucesso");

    LoadCell.tareNoDelay();
    if (LoadCell.getTareStatus() == true)
    {
      Serial.println("Tare complete");
    }

    vTaskDelay(pdMS_TO_TICKS(3000));
    lcd.clear();
    lcd.noBacklight();

    tareStarted = false;

    vTaskSuspend(taskTareHandle);
  }
}

void vTaskCalibrate(void *pvParameters)
{
  while (1)
  {
    Serial.println("Task calibrate");

    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("Calibracao");
    lcd.setCursor(0, 1);
    lcd.print("com sucesso");

    calibrate();

    vTaskDelay(pdMS_TO_TICKS(3000));
    lcd.clear();
    lcd.noBacklight();

    calibrationStarted = false;

    vTaskSuspend(taskCalibrateHandle);
  }
}

void vTaskReadTimeout(void *pvParameters)
{
  while (1)
  {
    Serial.println("Task read timeout");
    vTaskSuspend(taskReadWeightHandle);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Tempo esgotado");
    vTaskDelay(pdMS_TO_TICKS(3000));
    lcd.clear();
    lcd.noBacklight();

    readWeightStarted = false;

    vTaskSuspend(taskReadTimeoutHandle);
  }
}

//.......................Timers.............................
void callBackTimerBtnDebounce(TimerHandle_t xTimer)
{
  btnDebounce = false;
  xTimerStop(xTimerBtnDebounce, 0);
}

void callBackTimerReadWeightTimeout(TimerHandle_t xTimer)
{
  vTaskResume(taskReadTimeoutHandle);
  xTimerStop(xTimerReadWeightTimeout, 0);
}

//......................ISR.................................
void btnStartISRCallBack()
{
  if (!btnDebounce && !readWeightStarted && !tareStarted && !calibrationStarted)
  {
    Serial.println("BTN START");
    vTaskResume(taskReadWeightHandle);
    // xSemaphoreGive(xSemaphoreSendWeight);
    xTimerStart(xTimerReadWeightTimeout, 0);
    btnDebounce = true;
    readWeightStarted = true;
    xTimerStart(xTimerBtnDebounce, 0);
  }
}

void btnOrganicISRCallBack()
{
  int trashType = 1;
  if (!btnDebounce && readWeightStarted)
  {
    Serial.println("BTN ORGANIC");
    xQueueOverwrite(xFilaTrashType, &trashType);
    vTaskResume(taskSendWeightHandle);
    btnDebounce = true;
    xTimerStart(xTimerBtnDebounce, 0);
  }
}

void btnGlassISRCallBack()
{
  int trashType = 2;
  if (!btnDebounce && readWeightStarted)
  {
    Serial.println("BTN GLASS");
    xQueueOverwrite(xFilaTrashType, &trashType);
    vTaskResume(taskSendWeightHandle);
    btnDebounce = true;
    xTimerStart(xTimerBtnDebounce, 0);
  }
}

void btnMetalISRCallBack()
{
  int trashType = 3;
  if (!btnDebounce && readWeightStarted)
  {
    Serial.println("BTN METAL");
    xQueueOverwrite(xFilaTrashType, &trashType);
    vTaskResume(taskSendWeightHandle);
    btnDebounce = true;
    xTimerStart(xTimerBtnDebounce, 0);
  }
}

void btnPaperISRCallBack()
{
  int trashType = 4;
  if (!btnDebounce && readWeightStarted)
  {
    Serial.println("BTN PAPER");
    xQueueOverwrite(xFilaTrashType, &trashType);
    vTaskResume(taskSendWeightHandle);
    btnDebounce = true;
    xTimerStart(xTimerBtnDebounce, 0);
  }
}

void btnPlasticISRCallBack()
{
  int trashType = 5;
  if (!btnDebounce && readWeightStarted)
  {
    Serial.println("BTN PLASTIC");
    xQueueOverwrite(xFilaTrashType, &trashType);
    vTaskResume(taskSendWeightHandle);
    btnDebounce = true;
    xTimerStart(xTimerBtnDebounce, 0);
  }
}

void btnTareISRCallBack()
{
  if (!btnDebounce && !readWeightStarted && !tareStarted && !calibrationStarted)
  {
    Serial.println("BTN TARE");
    vTaskResume(taskTareHandle);
    btnDebounce = true;
    tareStarted = true;
    xTimerStart(xTimerBtnDebounce, 0);
  }
}

void btnCalibrateISRCallBack()
{
  if (!btnDebounce && !readWeightStarted && !tareStarted && !calibrationStarted)
  {
    Serial.println("BTN CALIBRATE");
    vTaskResume(taskCalibrateHandle);
    btnDebounce = true;
    calibrationStarted = true;
    xTimerStart(xTimerBtnDebounce, 0);
  }
}

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

void publishMessage(int trashType, float trashWeight)
{
  StaticJsonDocument<200> doc;
  doc["trashType"] = trashType;
  doc["trashWeight"] = trashWeight;
  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer); // print to client

  Serial.println(jsonBuffer);

  // client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer);
}