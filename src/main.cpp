/*Biblioteca do Arduino*/
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ArduinoJson.h>
#include <HX711_ADC.h>
#include "secrets.h"
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include "WiFi.h"
#include "SPIFFS.h"
#include <NTPClient.h>
#include "time.h"
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
#define vibrationSensor 15
#define btnOrganic 2
#define btnGlass 16
#define btnMetal 17
#define btnPaper 18
#define btnPlastic 19
#define btnTare 23
#define btnCalibrate 25

/*Constantes*/
#define weightReference 2000 // 2 kg
#define weightRepeatMeasureReference 10
#define weightMeasureInterval 50      // Time in ms between weight measurements
#define measureTimeout 10000          // Timeout in ms to measure weight
#define checkConnectionInterval 10000 // Time in ms to check connection
#define initTimeout 40000             // Timeout in ms to init
#define checkTimeInterval 60000 // Time in ms to check time
#define AWS_IOT_PUBLISH_TOPIC "esp32/pub"
#define AWS_IOT_SUBSCRIBE_TOPIC "esp32/sub"
#define ntp_server "time.google.com"
#define utc_timezone -3

/*Files path*/
#define dailyWeightsData "/dailyWeights.json"

/*Construtores*/
HX711_ADC LoadCell(HX711Dout, HX711Sck);
LiquidCrystal_I2C lcd(0x27, 16, 2);
WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);
WiFiUDP udp;
NTPClient ntp(udp, ntp_server, utc_timezone * 3600, 60000);

/*Variáveis Globais*/
const int calVal_eepromAdress = 0;
float calibrationValue;
int dayLastUpdate = 0;
float dailyGlass = 0;
float dailyOrganic = 0;
float dailyMetal = 0;
float dailyPlastic = 0;
float dailyPaper = 0;

/*Tamanho do Objeto JSON*/
const size_t JSON_SIZE = 1200; // https://arduinojson.org/v6/assistant/

/*Variáveis Globais de Estado*/
volatile bool btnDebounce = 0;
volatile bool readWeightStarted = 0;
volatile bool tareStarted = 0;
volatile bool calibrationStarted = 0;
volatile bool btnAlreadyPressed = 0;

/*Variáveis para armazenamento do handle das tasks, queues, semaphores e timers*/
TaskHandle_t taskReadWeightHandle = NULL;
TaskHandle_t taskSendWeightHandle = NULL;
TaskHandle_t taskTareHandle = NULL;
TaskHandle_t taskCalibrateHandle = NULL;
TaskHandle_t taskReadTimeoutHandle = NULL;
TaskHandle_t taskCheckConnectionHandle = NULL;
TaskHandle_t taskCheckTimeHandle = NULL;

QueueHandle_t xFilaTrashType;
QueueHandle_t xFilaTrashWeight;

SemaphoreHandle_t xSemaphoreCalibrate;

TimerHandle_t xTimerBtnDebounce;
TimerHandle_t xTimerReadWeightTimeout;
TimerHandle_t xTimerCheckConnection;
TimerHandle_t xTimerInitTimeout;
TimerHandle_t xTimerCheckTime;

/*Protótipos das Tasks*/
void vTaskReadWeight(void *pvParameters);
void vTaskSendWeight(void *pvParameters);
void vTaskTare(void *pvParameters);
void vTaskCalibrate(void *pvParameters);
void vTaskReadTimeout(void *pvParameters);
void vTaskCheckConnection(void *pvParameters);
void vTaskCheckTime(void *pvParameters);

/*Timer Callbacks*/
void callBackTimerBtnDebounce(TimerHandle_t xTimer);
void callBackTimerReadWeightTimeout(TimerHandle_t xTimer);
void callBackTimerCheckConnection(TimerHandle_t xTimer);
void callBackTimerInitTimeout(TimerHandle_t xTimer);
void callBackTimerCheckTime(TimerHandle_t xTimer);

/*Funções*/
void initButtons(void);
void publishMessage(int trashType, float trashWeight);
void publishDaily(int trashType);
void connectAWS(void);
void messageHandler(char *topic, byte *payload, unsigned int length);
void dailyWeightsResetVar(void);
boolean dailyWeightsResetAll(void);
boolean dailyWeightsRead(bool serialPrint);
boolean dailyWeightsWrite(int trashType, float newTrashWeight);
void printTime(void);

/*ISRs*/
void vibrationSensorISRCallBack();
void btnOrganicISRCallBack();
void btnGlassISRCallBack();
void btnMetalISRCallBack();
void btnPaperISRCallBack();
void btnPlasticISRCallBack();
void btnTareISRCallBack();
void btnCalibrateISRCallBack();

//.......................Setup Function.............................
void setup()
{
  Serial.begin(115200);
  vTaskDelay(pdMS_TO_TICKS(10));
  initButtons();

  Serial.println();
  Serial.println("Starting...");

  if (!SPIFFS.begin(true))
  {
    Serial.println("An Error has occurred while mounting SPIFFS");
    ESP.restart();
  }

  dailyWeightsRead(true);

  /*Criação Queues*/
  xFilaTrashType = xQueueCreate(1, sizeof(int));
  xFilaTrashWeight = xQueueCreate(1, sizeof(float));

  /*Criação Semaphores*/
  xSemaphoreCalibrate = xSemaphoreCreateBinary();

  if (xSemaphoreCalibrate == NULL)
  {
    Serial.println("Não foi possível criar o semaforo!");
    ESP.restart();
  }

  /*Criação Timers*/
  xTimerBtnDebounce = xTimerCreate("TIMER BTN DEBOUNCE", pdMS_TO_TICKS(1000), pdTRUE, 0, callBackTimerBtnDebounce);
  xTimerReadWeightTimeout = xTimerCreate("TIMER READ WEIGHT TIMEOUT", pdMS_TO_TICKS(measureTimeout), pdTRUE, 0, callBackTimerReadWeightTimeout);
  xTimerCheckConnection = xTimerCreate("TIMER CHECK CONNECTION", pdMS_TO_TICKS(checkConnectionInterval), pdTRUE, 0, callBackTimerCheckConnection);
  xTimerInitTimeout = xTimerCreate("TIMER INIT TIMEOUT", pdMS_TO_TICKS(initTimeout), pdTRUE, 0, callBackTimerInitTimeout);
  xTimerStart(xTimerInitTimeout, 0);
  xTimerCheckTime = xTimerCreate("TIMER CHECK TIME", pdMS_TO_TICKS(checkTimeInterval), pdTRUE, 0, callBackTimerCheckTime);

  /*Criação Interrupções*/
  attachInterrupt(digitalPinToInterrupt(vibrationSensor), vibrationSensorISRCallBack, FALLING);
  attachInterrupt(digitalPinToInterrupt(btnOrganic), btnOrganicISRCallBack, FALLING);
  attachInterrupt(digitalPinToInterrupt(btnGlass), btnGlassISRCallBack, FALLING);
  attachInterrupt(digitalPinToInterrupt(btnMetal), btnMetalISRCallBack, FALLING);
  attachInterrupt(digitalPinToInterrupt(btnPaper), btnPaperISRCallBack, FALLING);
  attachInterrupt(digitalPinToInterrupt(btnPlastic), btnPlasticISRCallBack, FALLING);
  attachInterrupt(digitalPinToInterrupt(btnTare), btnTareISRCallBack, FALLING);
  attachInterrupt(digitalPinToInterrupt(btnCalibrate), btnCalibrateISRCallBack, FALLING);

  /*Criação Tasks*/
  if (xTaskCreatePinnedToCore(vTaskReadWeight, "TASK READ WEIGHT", configMINIMAL_STACK_SIZE + 1024, NULL, 1, &taskReadWeightHandle, APP_CPU_NUM) == pdFAIL)
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

  if (xTaskCreatePinnedToCore(vTaskTare, "TASK TARE", configMINIMAL_STACK_SIZE + 1024, NULL, 1, &taskTareHandle, APP_CPU_NUM) == pdFAIL)
  {
    Serial.println("Não foi possível criar a Task Tare");
    ESP.restart();
  }
  vTaskSuspend(taskTareHandle);

  if (xTaskCreatePinnedToCore(vTaskCalibrate, "TASK CALIBRATE", configMINIMAL_STACK_SIZE + 2048, NULL, 1, &taskCalibrateHandle, APP_CPU_NUM) == pdFAIL)
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

  if (xTaskCreatePinnedToCore(vTaskCheckConnection, "TASK CHECK CONNECTION", configMINIMAL_STACK_SIZE + 1024, NULL, 1, &taskCheckConnectionHandle, APP_CPU_NUM) == pdFAIL)
  {
    Serial.println("Não foi possível criar a Task Check Connection");
    ESP.restart();
  }
  vTaskSuspend(taskCheckConnectionHandle);

  if (xTaskCreatePinnedToCore(vTaskCheckTime, "TASK CHECK TIME", configMINIMAL_STACK_SIZE + 4096, NULL, 1, &taskCheckTimeHandle, APP_CPU_NUM) == pdFAIL)
  {
    Serial.println("Não foi possível criar a Task Check Time");
    ESP.restart();
  }
  vTaskSuspend(taskCheckTimeHandle);

  /*Initialize Load Cell*/
  LoadCell.begin();
  // LoadCell.setReverseOutput(); //uncomment to turn a negative output value to positive

#if defined(ESP8266) || defined(ESP32)
  EEPROM.begin(512); // uncomment this if you use ESP8266/ESP32 and want to fetch the calibration value from eeprom
#endif
  EEPROM.get(calVal_eepromAdress, calibrationValue); // uncomment this if you want to fetch the calibration value from eeprom

  unsigned long stabilizingtime = 2000; // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
  boolean _tare = true;                 // set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag())
  {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    ESP.restart();
  }
  else
  {
    LoadCell.setCalFactor(calibrationValue); // set calibration value (float)
    Serial.println("Startup is complete");
  }
  while (!LoadCell.update())
    ;

  /*Initialize LCD*/
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Balanca");
  lcd.setCursor(0, 1);
  lcd.print("ligada!");
  vTaskDelay(pdMS_TO_TICKS(3000));

  /*Initialize Connection*/
  connectAWS();
  ntp.begin();
  ntp.forceUpdate();
  printTime();

  lcd.clear();
  lcd.noBacklight();

  xTimerStop(xTimerInitTimeout, 0);
  xTimerStart(xTimerCheckConnection, 0);
  xTimerStart(xTimerCheckTime, 0);
}

//.......................Loop Function.............................
void loop()
{
  client.loop();
  vTaskDelay(pdMS_TO_TICKS(1000));
}

//.......................Tasks.............................
void vTaskReadWeight(void *pvParameters)
{
  int repeatMeasureCount = 0;
  float weightMeasured = 0;
  while (1)
  {
    // check for new data/start next conversion:
    if (LoadCell.update())
    {
      weightMeasured += LoadCell.getData() / 1000;
      repeatMeasureCount++;
    }

    if (repeatMeasureCount >= weightRepeatMeasureReference)
    {
      weightMeasured = weightMeasured / weightRepeatMeasureReference;
      if (weightMeasured < 0)
      {
        weightMeasured = 0;
      }
      Serial.print("Measured weight: ");
      Serial.println(weightMeasured);
      xQueueOverwrite(xFilaTrashWeight, &weightMeasured);
      lcd.backlight();
      lcd.clear();
      lcd.print("Peso:");
      lcd.setCursor(0, 1);
      lcd.print(weightMeasured);
      lcd.print(" kg");
      repeatMeasureCount = 0;
      weightMeasured = 0;
    }

    vTaskDelay(pdMS_TO_TICKS(weightMeasureInterval));
  }
}

void vTaskSendWeight(void *pvParameters)
{
  int trashType = 0; // 1 - Organic; 2 - Glass; 3 - Metal; 4 - Paper; 5 - Plastic
  float trashWeight = 0;
  while (1)
  {
    Serial.println("Task send weight");
    vTaskSuspend(taskReadWeightHandle);
    xTimerStop(xTimerReadWeightTimeout, 0);
    xQueueReceive(xFilaTrashType, &trashType, portMAX_DELAY);
    xQueueReceive(xFilaTrashWeight, &trashWeight, portMAX_DELAY);

    dailyWeightsWrite(trashType, trashWeight);
    dailyWeightsRead(false);
    publishMessage(trashType, trashWeight);

    lcd.setCursor(0, 0);
    lcd.print("Peso enviado");
    lcd.setCursor(0, 1);
    lcd.print("com sucesso");

    vTaskDelay(pdMS_TO_TICKS(3000));
    lcd.clear();
    lcd.noBacklight();

    readWeightStarted = false;
    btnAlreadyPressed = false;

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
    lcd.print("Tarando a");
    lcd.setCursor(0, 1);
    lcd.print("balanca...");

    LoadCell.tare();
    while (LoadCell.getTareStatus() == false)
    {
      Serial.print(".");
      vTaskDelay(pdMS_TO_TICKS(500));
    }
    Serial.println("Tare complete");

    lcd.setCursor(0, 0);
    lcd.print("Balanca tarada");
    lcd.setCursor(0, 1);
    lcd.print("com sucesso");

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
    lcd.print("Iniciando");
    lcd.setCursor(0, 1);
    lcd.print("calibracao");

    vTaskDelay(pdMS_TO_TICKS(1000));

    Serial.println("Place the load cell an a level stable surface.");
    Serial.println("Remove any load applied to the load cell.");
    Serial.println("Press Tare button to set the tare offset.");

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Remova o peso e");
    lcd.setCursor(0, 1);
    lcd.print("clique em 'Tare'");

    LoadCell.update();
    btnAlreadyPressed = false;
    xSemaphoreTake(xSemaphoreCalibrate, portMAX_DELAY);
    LoadCell.tare();
    while (LoadCell.getTareStatus() == false)
    {
      Serial.print(".");
      vTaskDelay(pdMS_TO_TICKS(500));
    }
    Serial.println("Tare complete");

    Serial.println("Now, place your known mass on the loadcell.");

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Adicione o peso e");
    lcd.setCursor(0, 1);
    lcd.print("clique em 'Tare'");

    float known_mass = weightReference;

    LoadCell.update();
    btnAlreadyPressed = false;
    xSemaphoreTake(xSemaphoreCalibrate, portMAX_DELAY);

    LoadCell.refreshDataSet();                                          // refresh the dataset to be sure that the known mass is measured correct
    float newCalibrationValue = LoadCell.getNewCalibration(known_mass); // get the new calibration value

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

    Serial.println("End calibration");

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Calibracao");
    lcd.setCursor(0, 1);
    lcd.print("com sucesso");

    vTaskDelay(pdMS_TO_TICKS(3000));
    lcd.clear();
    lcd.noBacklight();

    calibrationStarted = false;
    btnAlreadyPressed = false;

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

void vTaskCheckConnection(void *pvParameters)
{
  while (1)
  {
    // Serial.println("Task check connection");
    if ((WiFi.status() != WL_CONNECTED))
    {
      Serial.println("WiFi connection failed... Restarting ESP...");
      ESP.restart();
    }
    if (!client.connected())
    {
      Serial.println("MQTT connection failed... Restarting ESP...");
      ESP.restart();
    }

    xTimerStart(xTimerCheckConnection, 0);
    vTaskSuspend(taskCheckConnectionHandle);
  }
}

void vTaskCheckTime(void *pvParameters)
{
  const int publishInterval = 3000;
  while (1)
  {
    //Serial.println("Task check time");
    if(dayLastUpdate != ntp.getDay()){
      Serial.println("Reseting daily weights...");
      dailyWeightsResetAll();
      publishDaily(1);
      vTaskDelay(pdMS_TO_TICKS(publishInterval));
      publishDaily(2);
      vTaskDelay(pdMS_TO_TICKS(publishInterval));
      publishDaily(3);
      vTaskDelay(pdMS_TO_TICKS(publishInterval));
      publishDaily(4);
      vTaskDelay(pdMS_TO_TICKS(publishInterval));
      publishDaily(5);
    }

    xTimerStart(xTimerCheckTime, 0);
    vTaskSuspend(taskCheckTimeHandle);
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

void callBackTimerCheckConnection(TimerHandle_t xTimer)
{
  vTaskResume(taskCheckConnectionHandle);
  xTimerStop(xTimerCheckConnection, 0);
}

void callBackTimerInitTimeout(TimerHandle_t xTimer)
{
  Serial.println("Init timeout... Restarting ESP...");
  ESP.restart();
}

void callBackTimerCheckTime(TimerHandle_t xTimer)
{
  vTaskResume(taskCheckTimeHandle);
  xTimerStop(xTimerCheckTime, 0);
}

//......................ISRs.................................
void vibrationSensorISRCallBack()
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (!btnDebounce && !readWeightStarted && !tareStarted && !calibrationStarted)
  {
    Serial.println("BTN/SENSOR START");
    vTaskResume(taskReadWeightHandle);
    xTimerStartFromISR(xTimerReadWeightTimeout, &xHigherPriorityTaskWoken);
    btnDebounce = true;
    readWeightStarted = true;
    xTimerStartFromISR(xTimerBtnDebounce, &xHigherPriorityTaskWoken);
  }
}

void btnOrganicISRCallBack()
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  int trashType = 1;
  if (!btnDebounce && readWeightStarted && !btnAlreadyPressed)
  {
    Serial.println("BTN ORGANIC");
    xQueueOverwrite(xFilaTrashType, &trashType);
    vTaskResume(taskSendWeightHandle);
    btnDebounce = true;
    btnAlreadyPressed = true;
    xTimerStartFromISR(xTimerBtnDebounce, &xHigherPriorityTaskWoken);
  }
}

void btnGlassISRCallBack()
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  int trashType = 2;
  if (!btnDebounce && readWeightStarted && !btnAlreadyPressed)
  {
    Serial.println("BTN GLASS");
    xQueueOverwrite(xFilaTrashType, &trashType);
    vTaskResume(taskSendWeightHandle);
    btnDebounce = true;
    btnAlreadyPressed = true;
    xTimerStartFromISR(xTimerBtnDebounce, &xHigherPriorityTaskWoken);
  }
}

void btnMetalISRCallBack()
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  int trashType = 3;
  if (!btnDebounce && readWeightStarted && !btnAlreadyPressed)
  {
    Serial.println("BTN METAL");
    xQueueOverwrite(xFilaTrashType, &trashType);
    vTaskResume(taskSendWeightHandle);
    btnDebounce = true;
    btnAlreadyPressed = true;
    xTimerStartFromISR(xTimerBtnDebounce, &xHigherPriorityTaskWoken);
  }
}

void btnPaperISRCallBack()
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  int trashType = 4;
  if (!btnDebounce && readWeightStarted && !btnAlreadyPressed)
  {
    Serial.println("BTN PAPER");
    xQueueOverwrite(xFilaTrashType, &trashType);
    vTaskResume(taskSendWeightHandle);
    btnDebounce = true;
    btnAlreadyPressed = true;
    xTimerStartFromISR(xTimerBtnDebounce, &xHigherPriorityTaskWoken);
  }
}

void btnPlasticISRCallBack()
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  int trashType = 5;
  if (!btnDebounce && readWeightStarted && !btnAlreadyPressed)
  {
    Serial.println("BTN PLASTIC");
    xQueueOverwrite(xFilaTrashType, &trashType);
    vTaskResume(taskSendWeightHandle);
    btnDebounce = true;
    btnAlreadyPressed = true;
    xTimerStartFromISR(xTimerBtnDebounce, &xHigherPriorityTaskWoken);
  }
}

void btnTareISRCallBack()
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (!btnDebounce && !readWeightStarted && !tareStarted && !calibrationStarted)
  {
    Serial.println("BTN TARE");
    vTaskResume(taskTareHandle);
    btnDebounce = true;
    tareStarted = true;
    xTimerStartFromISR(xTimerBtnDebounce, &xHigherPriorityTaskWoken);
  }
  if (!btnDebounce && !readWeightStarted && !tareStarted && calibrationStarted && !btnAlreadyPressed)
  {
    xSemaphoreGiveFromISR(xSemaphoreCalibrate, &xHigherPriorityTaskWoken);
    btnAlreadyPressed = true;
  }
}

void btnCalibrateISRCallBack()
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (!btnDebounce && !readWeightStarted && !tareStarted && !calibrationStarted)
  {
    Serial.println("BTN CALIBRATE");
    vTaskResume(taskCalibrateHandle);
    btnDebounce = true;
    calibrationStarted = true;
    xTimerStartFromISR(xTimerBtnDebounce, &xHigherPriorityTaskWoken);
  }
}

//......................Funções.............................
void initButtons(void)
{
  pinMode(vibrationSensor, INPUT_PULLUP);
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
  StaticJsonDocument<300> doc;
  doc["macAddress"] = WiFi.macAddress();
  doc["trashType"] = trashType;
  doc["trashWeight"] = trashWeight;
  switch (trashType)
  {
  case (1):
    doc["trashDailyWeight"] = dailyOrganic;
    break;
  case (2):
    doc["trashDailyWeight"] = dailyGlass;
    break;
  case (3):
    doc["trashDailyWeight"] = dailyMetal;
    break;
  case (4):
    doc["trashDailyWeight"] = dailyPaper;
    break;
  case (5):
    doc["trashDailyWeight"] = dailyPlastic;
    break;
  }

  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer); // print to client

  Serial.println(jsonBuffer);

  client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer);
}

void publishDaily(int trashType){
  StaticJsonDocument<300> doc;
  doc["macAddress"] = WiFi.macAddress();
  doc["trashType"] = trashType;
  switch (trashType)
  {
  case (1):
    doc["trashDailyWeight"] = dailyOrganic;
    break;
  case (2):
    doc["trashDailyWeight"] = dailyGlass;
    break;
  case (3):
    doc["trashDailyWeight"] = dailyMetal;
    break;
  case (4):
    doc["trashDailyWeight"] = dailyPaper;
    break;
  case (5):
    doc["trashDailyWeight"] = dailyPlastic;
    break;
  }

  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer); // print to client

  Serial.println(jsonBuffer);

  client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer);
}

void connectAWS()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to Wi-Fi");

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Conectando ao");
  lcd.setCursor(0, 1);
  lcd.print("Wi-Fi...");

  while (WiFi.status() != WL_CONNECTED)
  {
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.print(".");
  }

  Serial.println(".Connected!");

  // Configure WiFiClientSecure to use the AWS IoT device credentials
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);

  // Connect to the MQTT broker on the AWS endpoint we defined earlier
  client.setServer(AWS_IOT_ENDPOINT, 8883);

  // Create a message handler
  client.setCallback(messageHandler);

  Serial.print("Connecting to AWS IOT");

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Conectando a");
  lcd.setCursor(0, 1);
  lcd.print("nuvem...");

  while (!client.connect(THINGNAME))
  {
    Serial.print(".");
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  if (!client.connected())
  {
    Serial.println(".AWS IoT Timeout!");
    return;
  }

  // Subscribe to a topic
  client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);

  Serial.println(".AWS IoT Connected!");

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Conectado!");

  vTaskDelay(pdMS_TO_TICKS(2000));
}

void messageHandler(char *topic, byte *payload, unsigned int length)
{
  Serial.print("incoming: ");
  Serial.println(topic);

  StaticJsonDocument<200> doc;
  deserializeJson(doc, payload);
  const char *message = doc["message"];
  Serial.println(message);
}

void dailyWeightsResetVar(void){
  dayLastUpdate = 0;
  dailyOrganic = 0;
  dailyGlass = 0;
  dailyMetal = 0;
  dailyPaper = 0;
  dailyPlastic = 0;
}

boolean dailyWeightsResetAll(void)
{
  StaticJsonDocument<JSON_SIZE> jsonDailyWeights;

  File fileRead = SPIFFS.open(F(dailyWeightsData), "r");
  if (deserializeJson(jsonDailyWeights, fileRead))
  {
    // Falha na leitura, assume valores padrão
    dailyWeightsResetVar();
    Serial.println("Fail to read dailyWeights, setting default values.");
    return false;
  }
  else
  {
    jsonDailyWeights["dayLastUpdate"] = ntp.getDay();
    jsonDailyWeights["dailyOrganic"] = 0;
    jsonDailyWeights["dailyGlass"] = 0;
    jsonDailyWeights["dailyMetal"] = 0;
    jsonDailyWeights["dailyPaper"] = 0;
    jsonDailyWeights["dailyPlastic"] = 0;

    dayLastUpdate = ntp.getDay();
    dailyOrganic = 0;
    dailyGlass = 0;
    dailyMetal = 0;
    dailyPaper = 0;
    dailyPlastic = 0;
    
    fileRead.close();

    File fileWrite = SPIFFS.open(F(dailyWeightsData), "w+");
    serializeJsonPretty(jsonDailyWeights, fileWrite);
    fileWrite.close();

    return true;
  }
}

boolean dailyWeightsRead(bool serialPrint)
{
  // Lê configuração
  StaticJsonDocument<JSON_SIZE> jsonDailyWeights;

  File fileRead = SPIFFS.open(F(dailyWeightsData), "r");
  if (deserializeJson(jsonDailyWeights, fileRead))
  {
    // Falha na leitura, assume valores padrão
    dailyWeightsResetVar();
    Serial.println("Fail to read dailyWeights, setting default values.");
    return false;
  }
  else
  {
    // Sucesso na leitura
    dayLastUpdate = jsonDailyWeights["dayLastUpdate"];
    dailyGlass = jsonDailyWeights["dailyGlass"];
    dailyOrganic = jsonDailyWeights["dailyOrganic"];
    dailyMetal = jsonDailyWeights["dailyMetal"];
    dailyPlastic = jsonDailyWeights["dailyPlastic"];
    dailyPaper = jsonDailyWeights["dailyPaper"];

    fileRead.close();

    if (serialPrint)
    {
      Serial.println(F("Reading dailyWeights: "));
      serializeJsonPretty(jsonDailyWeights, Serial);
      Serial.println("");
    }

    return true;
  }
}

boolean dailyWeightsWrite(int trashType, float newTrashWeight)
{
  StaticJsonDocument<JSON_SIZE> jsonDailyWeights;

  File fileRead = SPIFFS.open(F(dailyWeightsData), "r");
  if (deserializeJson(jsonDailyWeights, fileRead))
  {
    // Falha na leitura, assume valores padrão
    dailyWeightsResetVar();
    Serial.println("Fail to read dailyWeights, setting default values.");
    return false;
  }
  else
  {
    switch (trashType)
    {
    case (1):
      jsonDailyWeights["dailyOrganic"] = dailyOrganic + newTrashWeight;
      break;
    case (2):
      jsonDailyWeights["dailyGlass"] = dailyGlass + newTrashWeight;
      break;
    case (3):
      jsonDailyWeights["dailyMetal"] = dailyMetal + newTrashWeight;
      break;
    case (4):
      jsonDailyWeights["dailyPaper"] = dailyPaper + newTrashWeight;
      break;
    case (5):
      jsonDailyWeights["dailyPlastic"] = dailyPlastic + newTrashWeight;
      break;
    }
    jsonDailyWeights["dayLastUpdate"] = ntp.getDay();

    fileRead.close();

    File fileWrite = SPIFFS.open(F(dailyWeightsData), "w+");
    serializeJsonPretty(jsonDailyWeights, fileWrite);
    fileWrite.close();

    return true;
  }
}

void printTime(void){
  struct tm  ts;
  char buf[80];
  time_t rawtime = ntp.getEpochTime();

  // Format time, "ddd yyyy-mm-dd hh:mm:ss zzz"
  ts = *localtime(&rawtime);
  strftime(buf, sizeof(buf), "%a %Y-%m-%d %H:%M:%S %Z", &ts);
  Serial.println(buf);
}