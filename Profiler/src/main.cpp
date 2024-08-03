#include <esp_now.h>
#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <MS5837.h>
#include <ElegantOTA.h>
#include <ESPAsyncWebServer.h>
#include <Array.h>

#define SDAPIN 22
#define SCLPIN 21

//https://www.ti.com/lit/ds/symlink/drv8231a.pdf page 11 for dc
#define IN1PIN 14
#define IN2PIN 25
#define BUTTONPIN 27
#define TIMEOUT 19000
#define SINKTIME 60000

const char companyNumber[5] = {'E', 'X', '2', '6', '\0'};

typedef struct struct_message {
  uint8_t mod = 1;
  unsigned long time;
  char text[33];
  char companyNumber[5];
} struct_message;
struct_message message;

typedef struct struct_depth {
  uint8_t mod = 2;
  unsigned long time;
  float depth;
  float pressure;
  char companyNumber[5];
} struct_depth;
struct_depth depth;

struct DepthAndPressure {
  unsigned long time;
  float depth;
  float pressure;
};

uint8_t connectivityCheck = 0;

Array<DepthAndPressure, 900> depthQueue;

uint8_t broadcastAddress[] = {0xA0, 0xA3, 0xB3, 0x2D, 0x05, 0xB4};
esp_now_peer_info_t peerInfo;

MS5837 sensor;

AsyncWebServer server(80);

bool profiling = false;
bool priming = false;

bool recieverAvailable = false;

void conCheck(void * parameter) {
  for(;;) {
    esp_now_send(broadcastAddress, (uint8_t *) &connectivityCheck, sizeof(uint8_t));
    vTaskDelay(1000 / portTICK_RATE_MS);
  }
}

float totalDepth = 0;
float totalPressure = 0;
int profiles = 0;

void getDepthData(void * parameter) {
  for(;;) {
    if (profiling) {
      depthQueue.push_back({millis(), sensor.depth(), sensor.pressure()/10});
    } else if (!profiling && (totalDepth != 0 || totalPressure != 0 || profiles != 0)) {
      totalDepth = 0;
      totalPressure = 0;
      profiles = 0;
    }
    vTaskDelay(1000 * portTICK_PERIOD_MS);
  }
}

void sendDepthData(void * parameter) {
  for(;;) {
    if (recieverAvailable && !depthQueue.empty()) {
      DepthAndPressure tmp = depthQueue.front();
      depth.depth = tmp.depth;
      depth.pressure = tmp.pressure;
      depth.time = tmp.time;
      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &depth, sizeof(depth));
      if (result == ESP_OK) depthQueue.remove(0);
    }
    vTaskDelay(200 / portTICK_RATE_MS);
  }
}

void profile(void * parameter) {
  profiling = true;
  message.time = millis();
  strcpy(message.text, "Profile commencing!");
  esp_now_send(broadcastAddress, (uint8_t *) &message, sizeof(message));

  digitalWrite(IN1PIN, HIGH);
  TickType_t startTick = xTaskGetTickCount();
  while (digitalRead(BUTTONPIN) == 1) vTaskDelay(10 * portTICK_PERIOD_MS);
  digitalWrite(IN2PIN, HIGH);
  vTaskDelay(SINKTIME * portTICK_PERIOD_MS);
  digitalWrite(IN1PIN, LOW);
  vTaskDelay(TIMEOUT * portTICK_PERIOD_MS);
  digitalWrite(IN2PIN, LOW);

  profiling = false;
  message.time = millis();
  strcpy(message.text, "Profile complete!");
  esp_now_send(broadcastAddress, (uint8_t *) &message, sizeof(message));
  vTaskSuspend(xTaskGetCurrentTaskHandle());
}

void prime(void * parameter) {
  priming = true;
  message.time = millis();
  strcpy(message.text, "Priming Profiler!");
  esp_now_send(broadcastAddress, (uint8_t *) &message, sizeof(message));

  digitalWrite(IN1PIN, HIGH);
  TickType_t startTick = xTaskGetTickCount();
  while (digitalRead(BUTTONPIN) == 1) vTaskDelay(10 * portTICK_PERIOD_MS);
  digitalWrite(IN1PIN, LOW);
  digitalWrite(IN2PIN, HIGH);
  vTaskDelay(TIMEOUT * portTICK_PERIOD_MS);
  digitalWrite(IN2PIN, LOW);

  priming = false;
  message.time = millis();
  strcpy(message.text, "Profiler ready for deployment!");
  esp_now_send(broadcastAddress, (uint8_t *) &message, sizeof(message));
  vTaskSuspend(xTaskGetCurrentTaskHandle());
}

void onSend(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS && !recieverAvailable) {
    recieverAvailable = true;
  } else if (status == ESP_NOW_SEND_FAIL && recieverAvailable) {
    recieverAvailable = false;
  }
}

void onReceive(const uint8_t * mac, const uint8_t *incomingData, int len) {
  uint8_t mod;
  memcpy(&mod, incomingData, sizeof(uint8_t));
  switch(mod) {
    case(1):
      if (profiling || priming) {
        message.time = millis();
        strcpy(message.text, "Profile already ongoing!");
        esp_now_send(broadcastAddress, (uint8_t *) &message, sizeof(message));
      }
      else xTaskCreate(profile, "Profile", 1000, NULL, 1, NULL);
      break;
    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);

  Wire.begin();

  sensor.setModel(MS5837::MS5837_02BA);
  sensor.init();
  
  sensor.setFluidDensity(997);

  sensor.read();

  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP("EasternEdgeProfilerOTA", "crazyasspassword74");
  WiFi.softAPConfig(IPAddress(192, 168, 0 , 1), IPAddress(192, 168, 0, 1), IPAddress(255, 255, 255, 0));

  if (esp_now_init() != ESP_OK) {
    Serial.println("Unable to start ESP-NOW!");
    return;
  }

  pinMode(BUTTONPIN, INPUT);
  pinMode(IN1PIN, OUTPUT);
  pinMode(IN2PIN, OUTPUT);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Unable to add peer!");
    return;
  }

  esp_now_register_send_cb(onSend);
  esp_now_register_recv_cb(onReceive);

  strcpy(message.companyNumber, companyNumber);
  strcpy(depth.companyNumber, companyNumber);

  ElegantOTA.begin(&server);
  server.begin();

  xTaskCreate(conCheck, "Connectivity Check", 1000, NULL, 1, NULL);
  xTaskCreate(getDepthData, "Get Depth Data", 1000, NULL, 1, NULL);
  xTaskCreate(sendDepthData, "Send Depth Data", 1000, NULL, 1, NULL);
  xTaskCreate(prime, "Prime Profiler", 1000, NULL, 1, NULL);
}

void loop() {
  sensor.read();
  ElegantOTA.loop();
}