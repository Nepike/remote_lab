// Библиотека для работы с платой
#include <ESP8266WiFi.h>

// Библиотека для создания сервера, прослушивающего HTTP запросы
#include <ESP8266WebServer.h> 

// Данные сети Wi-Fi
const char* ssid = "Nepike";
const char* password = "123453119670";

// Создаем объект server, который будет слушать HTTP-запросы на порту 80 (стандартный порт HTTP)
ESP8266WebServer server(80);

// Создаем объект localIp для хранения локального IP-адреса устройства.
IPAddress localIp;

const int ledPin = 2; // Встроенный светодиод подключен к GPIO-2

void setup() {
  Serial.begin(115200); // Инициализируем последовательный порт для отладки со скоростью 115200 бод
  pinMode(ledPin, OUTPUT); // Установка GPIO-2 как выход
  digitalWrite(ledPin, HIGH); // Выключаем светодиод
  
  // Подключение к Wi-Fi
  WiFi.begin(ssid, password);
  
  // Ожидаем пока плата успешно подключится
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("connecting Wi-Fi...");
  }
  Serial.println("connected Wi-Fi!");

  // Получаем IP-адрес устройства и выводим его в консоль
  localIp = WiFi.localIP(); 
  Serial.print("IP: ");
  Serial.println(localIp); // Именно по этому адресу мы будем обращаться к машинке

  // Настройка HTTP-обработчиков
  server.on("/lumos", HTTP_GET, handle_lumos); // при запросе http://<localIp>/lumos вызывается функция handle_lumos
  server.on("/nox", HTTP_GET, handle_nox); // при запросе http://<localIp>/nox вызывается функция handle_nox
  
  // Запускаем HTTP-сервер
  server.begin();
}

void loop() {
  // Проверяем, есть ли входящие HTTP-запросы, и вызываем соответствующие обработчики
  server.handleClient();
}


// Функции, обрабатывающие запросы

void handle_lumos() {
  digitalWrite(ledPin, LOW); // Включаем светодиод
  String response = "LED on! IP: " + localIp.toString(); // Формируем сообщение для ответа
  server.send(200, "text/plain", response); // Отправляем ответ
}

void handle_nox() {
  digitalWrite(ledPin, HIGH); // Выключаем светодиод
  String response = "LED off! IP: " + localIp.toString(); // Формируем сообщение для ответа
  server.send(200, "text/plain", response); // Отправляем ответ
}
