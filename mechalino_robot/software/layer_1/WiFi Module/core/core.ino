#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>

ESP8266WebServer server(80);

void handleRoot() {
  Serial.print(server.arg(0));
  server.send(200, "text/plain", "OK\r\n");
}

const char* ssid = "ExplorerBotHS", *password = "12345679";


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
  //Serial.printf("Web server started, open %s in a web browser\n", WiFi.localIP().toString().c_str());
  server.handleClient();
  delay(1);
}
