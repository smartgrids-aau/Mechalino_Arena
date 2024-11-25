#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

const char* ssid = "ROBOT_AP";  // Replace with your network credentials
const char* password = "Robot4321";
const char* deviceName = "ESP8266_Robot2";

ESP8266WebServer server(80);

void setup() {
  Serial.begin(9600);
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(deviceName);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi");
  Serial.println("IP Address: " + WiFi.localIP().toString());

  server.on("/start", HTTP_GET, handleStart);
  //server.on("/rotate", HTTP_GET, handleRotate);
  server.on("/stop", HTTP_GET, handleStop);

  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
}

void handleStart() {
  String speedStr = server.arg("speed");  // speed for both motors. If speedM2 also given, then this is speed for servo 1
  String durationStr = server.arg("duration");
  String speedM2Str = server.arg("speedM2");  // speed for servo 2

  if (speedStr != "" && speedM2Str == "") {  // Handle one or two speed parameters
    int speed = speedStr.toInt();
    int duration = durationStr == "" ? (speed == 0 ? 0 : 10000) : durationStr.toInt();

    if (duration <= 0) {
      server.send(400, "text/plain", "Invalid duration. Must be greater than 0.");
      return;
    }

    String command = String('M') + " " + speedStr + " " + String(duration);
    Serial.println(command);
    server.send(200, "text/plain", "Command received: " + command);
  } else if (speedStr != "" && speedM2Str != "") {
    int speedM1 = speedStr.toInt();
    int speedM2 = speedM2Str.toInt();
    int duration = durationStr == "" ? ((speedM1 == 0 && speedM2 == 0) ? 0 : 10000) : durationStr.toInt();

    String command = String('M') + " " + speedStr + " " + speedM2Str + " " +  String(duration);
    Serial.println(command);
    server.send(200, "text/plain", "Command received: " + command);
  } else {
    server.send(400, "text/plain", "Invalid request");
  }
}

/*void handleRotate() {
  String angleStr = server.arg("angle");
  if (angleStr == "") {
    server.send(400, "text/plain", "Missing parameters");
    return;
  }

  int angle = angleStr.toInt();

  if (angle < 0) {
    server.send(400, "text/plain", "Invalid parameters. Speed and duration must be greater than 0.");
    return;
  }

  String command = String('R') + " " + angle;
  Serial.println(command);
  server.send(200, "text/plain", "Command received: " + command);
}*/

void handleStop() {
  Serial.println("S");
  server.send(200, "text/plain", "Stop command received");
}
