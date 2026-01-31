#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>

// Настройки вашей Wi-Fi сети
const char* ssid = "Nopike";
const char* password = "11111111";

// Настройки порта
WiFiServer server(2000); // Создаем TCP-сервер на порту 2000
WiFiClient client;

// Создаем софтверный UART (RX = D5 (GPIO14), TX = D6 (GPIO12) - можно поменять)
SoftwareSerial SerialPort(D5, D6); // RX, TX

void setup() {
  // Запускаем оба последовательных порта
  Serial.begin(115200);       // Для отладки через USB
  SerialPort.begin(57600);    // Для общения с контроллером

  // Подключаемся к Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Подключение к WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nПодключено! IP адрес: ");
  Serial.println(WiFi.localIP()); // Выводим IP, он понадобится для ноутбука

  // Запускаем TCP-сервер
  server.begin();
  Serial.println("TCP-сервер запущен на порту 2000");
}

void loop() {
  // Проверяем, есть ли новое подключение по TCP
  if (!client || !client.connected()) {
    client = server.available(); // Принимаем нового клиента
    if (client) {
      Serial.println("Новый клиент подключен!");
    }
  }

  // Если клиент подключен и есть данные от него - читаем и шлем в UART
  if (client && client.available() > 0) {
    while (client.available()) {
      char c = client.read();
      SerialPort.write(c); // Пересылаем байт в контроллер
    }
  }

  // Если есть данные от контроллера (по UART) - читаем и шлем обратно клиенту по TCP
  if (SerialPort.available() > 0) {
    while (SerialPort.available()) {
      char c = SerialPort.read();
      if (client && client.connected()) {
        client.write(c); // Пересылаем данные на ноутбук
      }
    }
  }

  // Небольшая задержка для стабильности
  delay(1);
}
