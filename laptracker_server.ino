#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <HTTPClient.h>
#include <RTClib.h>
#include <vector>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>

#include "button.h"
#include "data_management.h"
#include "DHT.h"
#include "Grafcet.h"
#include "helpers.h"
#include "led.h"
#include "led_state.h"


#define BUTTON_PIN_POWER 18
#define BUTTON_PIN_RECORD 33
#define DHTPIN 32
#define DHTTYPE DHT11
#define I2C_FREQ 400000
#define IOT_DEVICE_NAME "ESP32-1"
#define LED_BLUE_PIN 18
#define LED_GREEN_PIN 19
#define LED_RED_PIN 4
#define SCL_GYR 26
#define SCL_RTC 22
#define SDA_GYR 25
#define SDA_RTC 21

WebServer server(80);

const char* ssid = "";
const char* password = "";

const String updateTimePath = "/api/iot/" + String(IOT_DEVICE_NAME) + "/update-time";
const String getDateTimePath = "/api/iot/" + String(IOT_DEVICE_NAME) + "/get-date-time";
const String getDataListPath = "/api/iot/" + String(IOT_DEVICE_NAME) + "/get-data-list";
const String deleteDataListPath = "/api/iot/" + String(IOT_DEVICE_NAME) + "/delete-data-list";
const String turnOnBlueLedPath = "/api/iot/" + String(IOT_DEVICE_NAME) + "/turn-on-blue-led";
const String turnOnGreenPath = "/api/iot/" + String(IOT_DEVICE_NAME) + "/turn-on-green-led";
const String turnOnRedLedPath = "/api/iot/" + String(IOT_DEVICE_NAME) + "/turn-on-red-led";
const String turnOfAllLedsPath = "/api/iot/" + String(IOT_DEVICE_NAME) + "/turn-off-all-leds";

unsigned long currentTime = millis();
unsigned long previousTime = 0;
const long timeoutTime = 2000;

const uint8_t I2C_GY_521_ADDRESS = 0x68;
const uint16_t MAX_RECORDS = 2000;
const uint16_t RECORDING_TIME = 2000;
const int32_t GYR_SENSOR_ID = 0;

const size_t bufferSize = 14;

Adafruit_MPU6050 mpu;
Button* bpPower;
Button* bpRecord;
DataManagement* dataManagement;
DateTime clock0;
DHT dht(DHTPIN, DHTTYPE);
Led* blueLed;
Led* greenLed;
Led* redLed;
RTC_DS1307 rtc;
TwoWire i2c_rtc = TwoWire(0);
TwoWire i2c_gyr = TwoWire(1);

bool dataTimeUpdateRequired = false, readDateTimeRequired = false, readDataListRequired = false, razDataListRequired = false, maxRecordReached = false;
int recordSize = 0;
const std::vector<int> grafcetStepNumbers = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 };
Grafcet grafcet(grafcetStepNumbers);
std::vector<CustomDataStruct> dataList;

void handleButtonInterruptRecordFalling() {
  bpRecord->handleInterrupt();
}

void connectToWiFi() {
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.print("Connected to SSID : ");
  Serial.println(WiFi.SSID());

  Serial.print("Connected to localIP : ");
  Serial.println(WiFi.localIP());

  Serial.print("Signal strength (RSSI) : ");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");
}

void setup_routing() {
  server.on(updateTimePath, updateTimePathCallback);
  server.on(getDateTimePath, getDateTimePathCallback);
  server.on(getDataListPath, getDataListPathCallback);
  server.on(deleteDataListPath, deleteDataListPathCallback);

  server.begin();
}

void updateTimePathCallback() {
  if (server.hasArg("plain") == false) {
    logError("updateTimePathCallback : ");
  }

  String body = server.arg("plain");

  logInfo("body");
  logInfo(body);

  int posUpdated = body.indexOf("updated");

  Serial.print("posUpdated : ");
  Serial.println(String(posUpdated));

  if (posUpdated != -1) {
    int quoteIndex = body.indexOf("\"", posUpdated + 15);

    Serial.print("quoteIndex : ");
    Serial.println(String(quoteIndex));

    if (quoteIndex != -1) {
      String dateTime = body.substring(posUpdated + 11, quoteIndex);

      Serial.print("dateTime : ");
      Serial.println(dateTime);

      dateTime.trim();  // Supprimer les espaces éventuels
      clock0 = decodeDateTimeString(dateTime);
      dataTimeUpdateRequired = true;
      transitions();
      posterieur();
    } else {
      Serial.println("Erreur lors de l'extraction de la valeur de la clé 'updated'.");
    }
  } else {
    Serial.println("Clé 'updated' non trouvée dans la chaîne JSON.");
  }
}


void getDateTimePathCallback() {
  readDateTimeRequired = true;
  transitions();
  posterieur();
}

void getDataListPathCallback() {
  readDataListRequired = true;
  transitions();
  posterieur();
}

void deleteDataListPathCallback() {
  logInfo("deleteDataListPathCallback");
  razDataListRequired = true;
  transitions();
  posterieur();
}

void setup() {
  Serial.begin(115200);

  connectToWiFi();
  setup_routing();

  pinMode(BUTTON_PIN_POWER, INPUT_PULLUP);
  pinMode(BUTTON_PIN_RECORD, INPUT_PULLUP);
  pinMode(LED_BLUE_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);

  bpPower = new Button(BUTTON_PIN_POWER);
  bpRecord = new Button(BUTTON_PIN_RECORD);
  clock0 = DateTime(2020, 12, 31, 23, 59, 30);
  blueLed = new Led(LED_BLUE_PIN);
  greenLed = new Led(LED_GREEN_PIN);
  redLed = new Led(LED_RED_PIN);

  i2c_gyr.setPins(SDA_GYR, SCL_GYR);

  bool isI2cGyrOK = i2c_gyr.begin(SDA_GYR, SCL_GYR, I2C_FREQ);
  bool isI2cRtcOK = i2c_rtc.begin(SDA_RTC, SCL_RTC, I2C_FREQ);

  delay(10);

  mpu.begin(I2C_GY_521_ADDRESS, &i2c_gyr, GYR_SENSOR_ID);

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  rtc.begin(&i2c_rtc);
  rtc.adjust(clock0);  // only for tests

  dht.begin();

  dataManagement = new DataManagement(&dht, &rtc, &mpu, clock0, RECORDING_TIME, MAX_RECORDS);

  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN_RECORD), handleButtonInterruptRecordFalling, FALLING);

  delay(10);
  grafcet.update(0);
  displayMemorySizes();
}

void loop() {
  server.handleClient();
  transitions();
  posterieur();
  delay(10);
}

void transitions() {
  // Etape 0 : Ajouter des initialisations nécessaires
  if (grafcet.getActiveStepNumber() == 0) {
    if (bpRecord->isPressed()) {
      // Etape 1 : la led verte est allumé pour signifier que le module est pret
      logInfo("Transition de l'étape 0 vers 1");
      grafcet.update(1);
    }
  } else if (grafcet.getActiveStepNumber() == 1) {
    if (bpRecord->isPressed() && !dataTimeUpdateRequired && !readDataListRequired && !razDataListRequired && !readDateTimeRequired) {
      // Etape 2 : la led verte clignote et on enregistre la température, l'humidité et la date et l'heure toutes les 2 secondes
      logInfo("Transition de l'étape 1 vers 2");
      grafcet.update(2);
    }
    if (!bpRecord->isPressed() && dataTimeUpdateRequired && !readDataListRequired && !razDataListRequired && !readDateTimeRequired) {
      // Etape 3 : Mise à jour de l'heure en Bluetooth avec l'app Flutter
      logInfo("Transition de l'étape 1 vers 3");
      grafcet.update(3);
    }
    if (!bpRecord->isPressed() && !dataTimeUpdateRequired && readDataListRequired && !razDataListRequired && !readDateTimeRequired) {
      // Etape 4 :  l'app Flutter vient lire en Bluetooth le contenu de la liste des données que l'on récupère par "dataManagement.getdataList()"
      logInfo("Transition de l'étape 1 vers 4");
      grafcet.update(4);
    }
    if (!bpRecord->isPressed() && !dataTimeUpdateRequired && !readDataListRequired && razDataListRequired && !readDateTimeRequired) {
      // Etape 5 : effacement de la liste de données via le Bluetooth avec l'app Flutter avec "dataManagement.eraseList()"
      logInfo("Transition de l'étape 1 vers 5");
      grafcet.update(5);
    }
    if (!bpRecord->isPressed() && !dataTimeUpdateRequired && !readDataListRequired && !razDataListRequired && readDateTimeRequired) {
      // Etape 9 : Lecture de la date et l'heure de l'esp
      logInfo("Transition de l'étape 1 vers 9");
      grafcet.update(9);
    }
  } else if (grafcet.getActiveStepNumber() == 2) {
    if (bpRecord->isPressed() && !maxRecordReached) {
      logInfo("Transition de l'étape 2 vers 1");
      grafcet.update(1);
      recordSize = dataManagement->getdataList().size();
      logInfo("Step 2 : il y a " + String(recordSize) + " enregistrements");
    } else if (!bpRecord->isPressed() && maxRecordReached) {
      logInfo("Transition de l'étape 6 vers 6");
      grafcet.update(6);
      recordSize = dataManagement->getdataList().size();
      logInfo("Step 2 : il y a " + String(recordSize) + " enregistrements");
    }
  } else if (grafcet.getActiveStepNumber() == 3) {
    // Date and time update
    if (!dataTimeUpdateRequired) {
      logInfo("Transition de l'étape 3 vers 1");
      grafcet.update(1);
    }
  } else if (grafcet.getActiveStepNumber() == 4) {
    // Send records
    if (!readDataListRequired) {
      logInfo("Transition de l'étape 4 vers 1");
      grafcet.update(1);
    }
  } else if (grafcet.getActiveStepNumber() == 5) {
    // Erease records
    if (!razDataListRequired) {
      logInfo("Transition de l'étape 5 vers 1");
      grafcet.update(1);
    }
  } else if (grafcet.getActiveStepNumber() == 6) {
    // Max records reached
    if (bpRecord->isPressed()) {
      logInfo("Transition de l'étape 6 vers 1");
      grafcet.update(1);
    }
  } else if (grafcet.getActiveStepNumber() == 9) {
    // Get Date and time
    if (!readDateTimeRequired) {
      logInfo("Transition de l'étape 9 vers 1");
      grafcet.update(1);
    }
  }
  bpRecord->setPressed(false);
}

void posterieur() {
  switch (grafcet.getActiveStepNumber()) {
    case 0:
      blueLed->toggle(LED_OFF);
      greenLed->toggle(LED_ON);
      redLed->toggle(LED_OFF);
      break;
    case 1:
      blueLed->toggle(LED_ON);
      greenLed->toggle(LED_OFF);
      redLed->toggle(LED_OFF);
      break;
    case 2:
      blueLed->toggle(LED_FLASHING);
      greenLed->toggle(LED_OFF);
      redLed->toggle(LED_OFF);
      dataManagement->recordingData(millis());
      recordSize = dataManagement->getdataList().size();
      maxRecordReached = recordSize > MAX_RECORDS ? true : false;
      break;
    case 3:
      logInfo("Step 3 : update date and time");
      posteriorUpdateDateAndTime();
      dataTimeUpdateRequired = false;
      break;
    case 4:
      logInfo("Step 4 : reading data list for WebApp");
      dataList = dataManagement->getdataList();
      posteriorSendDataList();
      readDataListRequired = false;
      break;
    case 5:
      logInfo("Step 5 : raz data list");
      posteriorDeleteDataList();
      razDataListRequired = false;
      logInfo("Step 5 : records ereased from Bluetooth");
      break;
    case 6:
      logInfo("Step 6 : maximum records reached");
      break;
    case 7:
      logInfo("Step 7 : not used");
      break;
    case 8:
      logInfo("Step 8 : not used");
      break;
    case 9:
      logInfo("Step 9 : reading date and time");
      posteriorSendDateTime();
      readDateTimeRequired = false;
      break;
    default:
      break;
  }
}

void appRequestBlueLedOn() {
  blueLed->toggle(LED_ON);
  greenLed->toggle(LED_OFF);
  redLed->toggle(LED_OFF);
}

void appRequestGreenLedOn() {
  blueLed->toggle(LED_OFF);
  greenLed->toggle(LED_ON);
  redLed->toggle(LED_OFF);
}

void appRequestRedLedOn() {
  blueLed->toggle(LED_OFF);
  greenLed->toggle(LED_OFF);
  redLed->toggle(LED_ON);
}

void appRequestLedsOff() {
  blueLed->toggle(LED_OFF);
  greenLed->toggle(LED_OFF);
  redLed->toggle(LED_OFF);
}

void posteriorUpdateDateAndTime() {
  rtc.adjust(clock0);

  int year = clock0.year();
  int month = clock0.month();
  int day = clock0.day();
  int hour = clock0.hour();
  int minute = clock0.minute();
  int second = clock0.second();

  String jsonResponse = "{\"updated\":\"" + String(year) + "-" + String(month) + "-" + String(day) + " " + String(hour) + ":" + String(minute) + ":" + String(second) + "\"}";

  logInfo("Heure du module une fois mis à jour " + jsonResponse);

  server.send(200, "application/json", jsonResponse);
}

void posteriorSendDateTime() {
  if (!rtc.isrunning()) {
    logError("RTC module is not running !");
    server.send(409, "application/json", "{\"error\":\"RTC module is not running !\"}");
  }
  String dateTimeFromEsp = formatDateTime(rtc.now().year(), rtc.now().month(), rtc.now().day(), rtc.now().hour(), rtc.now().minute(), rtc.now().second());
  server.send(200, "application/json", "{\"dateTimeFromEsp\":\"" + dateTimeFromEsp + "\"}");
}

void posteriorSendDataList() {
  String jsonDataList;
  //if (dataList.empty()) { return; }

  String headerVariables = "\"iotDeviceName\":\"" + String(IOT_DEVICE_NAME) + "\",";

  for (const CustomDataStruct& data : dataList) {
    jsonDataList += "{";
    jsonDataList += "\"year\":\"" + String(data.year) + "\",";
    jsonDataList += "\"month\":\"" + String(data.month) + "\",";
    jsonDataList += "\"day\":\"" + String(data.day) + "\",";
    jsonDataList += "\"hour\":\"" + String(data.hour) + "\",";
    jsonDataList += "\"minute\":\"" + String(data.minute) + "\",";
    jsonDataList += "\"second\":\"" + String(data.second) + "\",";
    jsonDataList += "\"temperature\":\"" + String(data.temperature) + "\",";
    jsonDataList += "\"humidity\":\"" + String(data.humidity) + "\",";
    jsonDataList += "\"roll\":\"" + String(data.roll) + "\",";
    jsonDataList += "\"pitch\":\"" + String(data.pitch) + "\",";
    jsonDataList += "\"yaw\":\"" + String(data.yaw) + "\"";
    jsonDataList += "},";
  }
  jsonDataList = "{" + headerVariables + "\"data\":[" + jsonDataList.substring(0, jsonDataList.length() - 1) + "]}";

  server.send(200, "application/json", jsonDataList);
}

void posteriorDeleteDataList() {
  dataManagement->eraseList();
  server.send(200, "application/json", "{\"data\":\"[]\"}");
}

void sendStepChangebyOta(bool isRecording) {
  String jsonDataList;

  for (const CustomDataStruct& data : dataList) {
    jsonDataList += "{";
    jsonDataList += "\"setNumber\":\"" + String(grafcet.getActiveStepNumber()) + "\",";
    jsonDataList += "\"recordingState\":\"" + String(isRecording) + "\"";
    jsonDataList += "},";
  }
  jsonDataList = "[" + jsonDataList.substring(0, jsonDataList.length() - 1) + "]";
}

DateTime decodeDateTimeString(String dateTimeString) {
  int yearPos = dateTimeString.indexOf("-");
  int monthPos = dateTimeString.indexOf("-", yearPos + 1);
  int dayPos = dateTimeString.indexOf(" ", monthPos + 1);
  int hourPos = dateTimeString.indexOf(":", dayPos + 1);
  int minutePos = dateTimeString.indexOf(":", hourPos + 1);
  int secondPos = minutePos + 1;

  int year = dateTimeString.substring(0, yearPos).toInt();
  int month = dateTimeString.substring(yearPos + 1, monthPos).toInt();
  int day = dateTimeString.substring(monthPos + 1, dayPos).toInt();
  int hour = dateTimeString.substring(dayPos + 1, hourPos).toInt();
  int minute = dateTimeString.substring(hourPos + 1, minutePos).toInt();
  int second = dateTimeString.substring(minutePos + 1).toInt();

  return DateTime(year, month, day, hour, minute, second);
}

String formatDateTime(int year, int month, int day, int hour, int minute, int second) {
  char formatted[20];
  sprintf(formatted, "%04d-%02d-%02d %02d:%02d:%02d", year, month, day, hour, minute, second);
  return String(formatted);
}

void displayMemorySizes() {
  logInfo("\n==================================");
  logInfo("Tailles de la mémoire de l'ESP32 :");

  logInfo("Mémoire SRAM libre : ");
  logInfo(String(ESP.getFreeHeap()));

  logInfo("Taille du programme : ");
  logInfo(String(ESP.getSketchSize()));

  logInfo("Espace libre pour le programme : ");
  logInfo(String(ESP.getFreeSketchSpace()));

  logInfo("Taille totale de la puce flash : ");
  logInfo(String(ESP.getFlashChipSize()));

  logInfo("==================================\n");
}

/*
void create_json(char* tag, float value, char* unit) {
  jsonDocument.clear();
  jsonDocument["type"] = tag;
  jsonDocument["value"] = value;
  jsonDocument["unit"] = unit;
  serializeJson(jsonDocument, buffer);
}*/
/*
void getTemperature() {
  Serial.println("Get temperature");
  create_json("temperature", 25, "°C");
  server.send(200, "application/json", buffer);
}
*/
