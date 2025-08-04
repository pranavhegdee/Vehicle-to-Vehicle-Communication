#include <WiFi.h>

const char* ssid = "----";
const char* password = "----";

const char* host = "10.171.143.40";  // Receiver IP
const uint16_t port = 5000;          // Receiver port

#define UART_RX_PIN 21
#define UART_TX_PIN 22  // Not used here

WiFiClient client;

void setup() {
  Serial.begin(9600);  // USB debug
  Serial2.begin(9600, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);  // STM32 UART input

  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nWiFi connected.");
  Serial.print("ESP32 IP address: ");
  Serial.println(WiFi.localIP());  // Show current IP
}

void loop() {
  if (Serial2.available()) {
   Serial2.setTimeout(2000);  // wait max 2 sec
String uartData = Serial2.readStringUntil('\n');
uartData.trim();

 
  Serial.println("UART Received: " + uartData);
  if (client.connect(host, port)) {
    client.println(uartData);
    Serial.println("Sent: " + uartData);
    client.stop();
  } else {
    Serial.println("❌ WiFi send failed");
  }


  }
}
