#include <WiFi.h>

const char* ssid = "-----";
const char* password = "------";

WiFiServer server(5000);

void setup() {
  Serial.begin(9600);
  delay(1000); 

  Serial.println(" Connecting to Wi-Fi...");

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

 
  Serial.println("\n✅ Connected to Wi-Fi");
  Serial.print("📡 My IP address: ");
  Serial.println(WiFi.localIP());  

  server.begin();
  Serial.println("🚀 TCP Server started on port 5000");
}

void loop() {

  WiFiClient client = server.available();

  if (client) {
    Serial.println("🔗 Client connected.");

    while (client.connected()) {
      if (client.available()) {
        String data = client.readStringUntil('\n');
        data.trim();
        Serial.println("📥 Received: " + data);
      }
    }

    client.stop();
  }
}
