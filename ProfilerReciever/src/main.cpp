#include <WiFi.h>
#include <esp_now.h>
#include <LittleFS.h>
#include <ESPAsyncWebServer.h>
#include <vector>

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

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

struct DepthAndTime {
  unsigned long time;
  float depth;
};

std::vector<DepthAndTime> dataPoints;

// A0:A3:B3:0F:7B:A0
uint8_t broadcastAddress[] = {0xA0, 0xA3, 0xB3, 0x0F, 0x7B, 0xA0};
esp_now_peer_info_t peerInfo;

char tmp[250];

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
  if (status != ESP_NOW_SEND_SUCCESS) ws.textAll("Unable to send signal to profiler\n");
}

void onReceive(const uint8_t * mac, const uint8_t *incomingData, int len) {
  uint8_t mod;
  memcpy(&mod, incomingData, sizeof(uint8_t));
  switch(mod) {
    case(1):
      memcpy(&message, incomingData, sizeof(message));
      sprintf(tmp, "[%s FLT] - %s - %s\n", millisToTimeStr(message.time), message.companyNumber, message.text);
      ws.textAll(tmp);
      // if (String(message.text) == "Profiler ready for deployment!" || String(message.text) == "Profile commencing!") dataPoints.clear();
      break;
    case(2):
      memcpy(&depth, incomingData, sizeof(depth));
      if(abs(depth.depth) < 10) {
        DepthAndTime tmp1;
        tmp1.time = depth.time/1000;
        tmp1.depth = depth.depth;
        dataPoints.push_back(tmp1);
        sprintf(tmp, "[%s FLT] - %s - Depth: %f m - Pressure: %f kPa\n", millisToTimeStr(depth.time), depth.companyNumber, depth.depth, depth.pressure);
        ws.textAll(tmp);
      }
      break;
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
    Serial.println("Failed to mount LittleFS");
    return;
  }
  if (esp_now_init() != ESP_OK) {
    Serial.println("Failed to start ESP-NOW!");
    return;
  }

  esp_now_register_recv_cb(onReceive);
  esp_now_register_send_cb(onSend);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Unable to add peer!");
    return;
  }

  ws.onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client,
                AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
      Serial.println("WebSocket client connected");
    }
  });

  server.addHandler(&ws);
  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
  server.serveStatic("/jquery.min.js", LittleFS, "/").setDefaultFile("jquery.min.js");
  server.serveStatic("/dyngraph.min.js", LittleFS, "/").setDefaultFile("dyngraph.min.js");
  server.on("/data", [](AsyncWebServerRequest *request) {
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    response->print("[");
    for (size_t i = 0; i < dataPoints.size(); i++) {
      response->print("[");
      response->print(dataPoints[i].time);
      response->print(",");
      response->print(dataPoints[i].depth);
      response->print("]");
      if (i < dataPoints.size() - 1) response->print(",");
    }
    response->print("]");
    request->send(response);
  });
  server.on("/start", [](AsyncWebServerRequest *request) {
    uint8_t startSignal = 1;
    esp_now_send(broadcastAddress, (uint8_t *) &startSignal, sizeof(uint8_t));
  });

  server.begin();
}

void loop() {}

