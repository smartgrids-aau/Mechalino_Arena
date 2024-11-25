#include <ESP8266WiFi.h>
#include <WiFiClient.h>
//#include <ESP8266LLMNR.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>

#define FORWARD 'F'
#define BACKWARD 'B'
#define TURNRIGHT 'R'
#define TURNLEFT 'L'
#define GLOBAL_ANGLE 'G'

#define ROBOT_1 "/Robot1/"
#define ROBOT_2 "/Robot2/"
#define ROBOT_3 "/Robot3/"
#define ROBOT_4 "/Robot4/"
#define ROBOT_5 "/Robot5/"
#define ROBOT_6 "/Robot6/"

ESP8266WebServer server(80);
/*
IPAddress ip(10, 0, 1, 1);  //Will be my fixed IP
IPAddress subnet(255, 0, 0, 0);   //mask
IPAddress gateway(10, 0, 2, 200);  //the router
IPAddress dns(10, 0, 2, 200);  //my router have DNS server
*/
void handleRoot() {//recive 192.168.5.218/Robot1/?F=10&L=90
  uint8_t i = 0;
  while(server.arg(i) != "")
  {
//    Serial.print(server.argName(i));
//    Serial.print(" : ");
//    Serial.println(server.arg(i));
    Serial.printf("%S%S\n",server.argName(i),server.arg(i));
    i++;
  }
  server.send(200, "text/plain", "OK\r\n");
}

const char* ssid = "SPONGEBOB69", *password = "Test1410";
const char* deviceName = "esp8266";

void setup(void)
{ 
  Serial.begin(115200);
  delay(3000);
//  
//  IPAddress local_IP(192, 168, 112, 180);//
  
//  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  //WiFi.config(ip, gateway, subnet, dns);

  WiFi.setHostname(deviceName); //define hostname

  WiFi.mode(WIFI_STA);

  WiFi.begin(ssid,password);
  

  while (WiFi.status() != WL_CONNECTED) 
  {
     Serial.printf(".");
     delay(500);
  }
  delay(1000);

  
  /*
  if (!MDNS.begin("esp82661"))             // Start the mDNS responder for Robot1.local
  {
    Serial.println("Error setting up MDNS responder!");
  }*/

  server.on(ROBOT_1, handleRoot);//
  server.begin();
  delay(1000);

//  Serial.printf("Web server started, open %s in a web browser\n", WiFi.localIP().toString().c_str());
  Serial.println(WiFi.localIP());
 // Serial.println(WiFi.dnsIP());
}

void loop() 
{
  //Serial.printf("Web server started, open %s in a web browser\n", WiFi.localIP().toString().c_str());
  server.handleClient();
 // MDNS.update();
  delay(1);
}
