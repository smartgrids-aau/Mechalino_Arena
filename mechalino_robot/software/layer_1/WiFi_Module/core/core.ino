#define SERIAL_TIMEOUT_MS 10000

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>

const char *ssid = "SPONGEBOB69", *password = "Test1410";

ESP8266WebServer server(80);

void handleRoot() {
  Serial.print(server.arg(0));

  unsigned long startTime = millis();
  while (!Serial.available()) {
    if (millis() - startTime > SERIAL_TIMEOUT_MS) {
      break;
    }
    delay(100);
  }

  if (Serial.available()) {
    while (Serial.available()) {
      Serial.read();
    }
    server.send(200, "text/plain", "OK\r\n");
  } else {
    server.send(500, "text/plain", "Error: Timeout waiting for reply from Serial\r\n");
  }
}

void setup(void)
{ 
  Serial.begin(9600);
  delay(3000);
  WiFi.begin(ssid,password);
  while (WiFi.status() != WL_CONNECTED) 
  {
     delay(500);
  }
  delay(1000);
  server.on("/", handleRoot);
  server.begin();
  delay(1000);
}

void loop() 
{
  server.handleClient();
  delay(1);
}
