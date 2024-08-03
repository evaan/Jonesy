#include <WiFi.h>
#include <esp_now.h>
#include <LittleFS.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <rgb_lcd.h>

AsyncWebServer server(80);
String logs = "";

rgb_lcd lcd;

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

JsonDocument depthOverTime;

// A0:A3:B3:0F:7B:A0
uint8_t broadcastAddress[] = {0xA0, 0xA3, 0xB3, 0x0F, 0x7B, 0xA0};
esp_now_peer_info_t peerInfo;

char tmp[250];

int profileTime = 0;

String zeroPrefix(int number) {return number < 10 ? "0" : "";}

String millisToTimeStr(const unsigned long time) {
  const unsigned int seconds = (time / 1000) % 60;
  const unsigned int minutes = (time / 60000) % 60;
  const unsigned int hours = (time / 3600000) % 24;
  char output[9];
  sprintf(output, "%s%u:%s%u:%s%u", zeroPrefix(hours), hours, zeroPrefix(minutes), minutes, zeroPrefix(seconds), seconds);
  return output;
}

void onSend(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) logs += "Unable to send signal to profiler.\n";
}

void onReceive(const uint8_t * mac, const uint8_t *incomingData, int len) {
  uint8_t mod;
  memcpy(&mod, incomingData, sizeof(uint8_t));
  switch(mod) {
    case(1):
      memcpy(&message, incomingData, sizeof(message));
      sprintf(tmp, "[%s] - %s - %s\n", millisToTimeStr(message.time), message.companyNumber, message.text);
      logs += tmp;
      if (String(message.text) == "Profiler ready for deployment!") depthOverTime.clear();
      if (String(message.text) == "Profile commencing!") {
        // depthOverTime.clear();
        profileTime = millis() - 1000;
      }
      if (String(message.text) == "Profile complete!") profileTime = 0;
      break;
    case(2):
      memcpy(&depth, incomingData, sizeof(depth));
      if(abs(depth.depth) < 10) {
        depthOverTime[depthOverTime.size()][0] = (depth.time/1000);
        depthOverTime[depthOverTime.size()-1][1] = depth.depth;
        sprintf(tmp, "[%s] - %s - Depth: %f - Pressure: %f\n", millisToTimeStr(depth.time), depth.companyNumber, depth.depth, depth.pressure);
        logs += tmp;
      }
    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP("EasternEdgeStation", "crazyasspassword74");
  WiFi.softAPConfig(IPAddress(192, 168, 0 , 1), IPAddress(192, 168, 0, 1), IPAddress(255, 255, 255, 0));

  if (!LittleFS.begin()) {
    Serial.println("Unable to start LittleFS!");
    return;
  }
  if (esp_now_init() != ESP_OK) {
    Serial.println("Unable to start ESP-NOW!");
    return;
  }

  esp_now_register_send_cb(onSend);
  esp_now_register_recv_cb(onReceive);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  lcd.begin(16, 2);
  lcd.setRGB(69, 120, 120);
  lcd.print("Float Time: 00:00:00");

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Unable to add peer!");
    return;
  }

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {request->send(LittleFS, "/index.html");});
  server.on("/jquery.min.js", [](AsyncWebServerRequest *request) {request->send(LittleFS, "/jquery.min.js");});
  server.on("/dygraph.min.js", [](AsyncWebServerRequest *request) {request->send(LittleFS, "/dygraph.min.js");});
  server.on("/logs", [](AsyncWebServerRequest *request) {request->send_P(200, "text/plain", logs.c_str());});
  server.on("/data", [](AsyncWebServerRequest *request) {
    String tmp;
    serializeJson(depthOverTime, tmp);
    request->send_P(200, "text/json", tmp.c_str());
  });
  server.on("/start", [](AsyncWebServerRequest *request) {
    uint8_t startSignal = 1;
    esp_now_send(broadcastAddress, (uint8_t *) &startSignal, sizeof(uint8_t));
  });
  
  server.begin();
}

void loop() {
  //untested: no lcd screen
  if (profileTime > 0) {
    lcd.setCursor(0, 0);
    lcd.print("Float Time: ");
    lcd.print(millisToTimeStr(millis() - profileTime));
    // Serial.println(millisToTimeStr(millis() - profileTime));
  }

  sleep(1);
}