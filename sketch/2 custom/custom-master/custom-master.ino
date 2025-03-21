#include <NPK.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>

#define RX_PIN 3
#define TX_PIN 1

SoftwareSerial nanoSerial(RX_PIN, TX_PIN);

const char* ssid = "Nopike";
const char* password = "11111111";

ESP8266WebServer server(80);

void espBlink() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(200);
  digitalWrite(LED_BUILTIN, LOW);
  delay(200);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(200);
  digitalWrite(LED_BUILTIN, LOW);
  delay(200);
  digitalWrite(LED_BUILTIN, HIGH);
}

void sendPack(const DataPack& pack) {
  uint8_t* data = pack.get_serialized_data();
  size_t size = pack.get_serialized_size();
  
  nanoSerial.write(data, size);
  free(data);
  espBlink();
}

DataPack receiveResponse() {
  unsigned long startTime = millis();
  
  while (nanoSerial.available() < 2) { // Ждем начало передачи пакета
    if (millis() - startTime > 1000) { // Таймаут в 1 секунду
      DataPack pack0 = DataPack(DataPack::Command::RESPONSE);
      pack0.append_arg("RESPONSE TIMEOUT!");
      return pack0;
    }
  }
  DataPack pack(nanoSerial);

  return pack;
}

void handleCommand() {
  if (server.method() != HTTP_POST) {
    server.send(405, "text/plain", "Method Not Allowed");
    return;
  }

  String jsonStr = server.arg("plain");
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, jsonStr);

  if (error) {
    server.send(400, "text/plain", "Invalid JSON");
    return;
  }

  const char* cmd = doc["cmd"];
  JsonArray args = doc["args"];

  if (strcmp(cmd, "go") == 0) {
    uint8_t direction = args[0];
    uint8_t speed = args[1];
    uint16_t time = args[2];
    DataPack pack(DataPack::Command::GO);
    pack.append_arg(direction);
    pack.append_arg(speed);
    pack.append_arg(time);
    sendPack(pack);
    server.send(200, "application/json", "{\"status\": \"success\"}");
  }
  else if (strcmp(cmd, "lcd") == 0) {
    String data = args[0];
    DataPack pack(DataPack::Command::LCD);
    pack.append_arg(data.c_str());
    sendPack(pack);
    server.send(200, "text/plain", "LCD command sent");
  }
  else if (strcmp(cmd, "getrange") == 0) {
    String data = args[0];
    DataPack pack(DataPack::Command::GETRANGE);
    sendPack(pack);

    DataPack response = receiveResponse();
    if (response.is_valid()) {
      auto s = response.get_next_val<const char*>();
      DataPack pack1(DataPack::Command::LCD);
      pack1.append_arg(s);
      sendPack(pack1);
      server.send(200,"text/plain", s);
    } else {
      server.send(500, "application/json", "{\"status\": \"error\"}");
    }
  }
  else if (strcmp(cmd, "getspeed") == 0) {
    uint8_t data = args[0];
    DataPack pack(DataPack::Command::GETSPEED);
    pack.append_arg(data);
    sendPack(pack);

    DataPack response = receiveResponse();
    if (response.is_valid()) {
      auto s = response.get_next_val<const char*>();
      DataPack pack1(DataPack::Command::LCD);
      pack1.append_arg(s);
      sendPack(pack1);
      server.send(200,"text/plain", s);
    } else {
      server.send(500, "application/json", "{\"status\": \"error\"}");
    }
  }
  
  else {
    server.send(404, "text/plain", "Command not found");
  }

}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

   nanoSerial.begin(9600);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
  
  String localIp = WiFi.localIP().toString();
  DataPack pack(DataPack::Command::LCD);
  pack.append_arg(localIp.c_str());
  sendPack(pack);
  digitalWrite(LED_BUILTIN, HIGH);

  server.on("/command", HTTP_POST, handleCommand);
  server.begin();
}

void loop() {
  server.handleClient();
}