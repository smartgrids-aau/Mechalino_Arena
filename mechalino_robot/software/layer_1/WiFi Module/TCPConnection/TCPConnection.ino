#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "ROBOT_AP";
const char* password = "Robot4321";
const uint16_t server_port = 5000;
const int udpPort = 4210;                    // UDP broadcast port
const char* udpAddress = "255.255.255.255";  // Broadcast address
IPAddress server_ip;

WiFiClient client;
WiFiUDP udp;

String robotID = "";
String currentLocation = "";
String targetLocation = "";

enum RobotState { CONNECTING,
                  REGISTERING,
                  REGISTERED,
                  LOCATION_REQUEST,
                  TARGET_REQUEST,
                  END };
RobotState currentState = CONNECTING;

unsigned long lastRequestTime = 0;
unsigned long requestInterval = 500;

void setup() {
  Serial.begin(9600);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  // Connect to Wi-Fi
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi!");
  server_ip = WiFi.gatewayIP();

  // Initialize UDP
  udp.begin(udpPort);

  currentState = CONNECTING;
}

void loop() {
  switch (currentState) {
    case CONNECTING:
      // Connect to ROS server
      if (WiFi.status() == WL_CONNECTED) {
        if (client.connect(server_ip, server_port)) {
          //Serial.println("Connected to server!");
          if (robotID == "") {
            currentState = REGISTERING;
            client.println("REGISTER");
            //Serial.println("REGISTER sent to server");
          } else {
            currentState = REGISTERED;
          }
        } else {
          //Serial.println("Connection to server failed, retrying...");
          delay(1000);  // Wait a bit before retrying
        }
      } else {
        //Serial.println("WiFi disconnected, reconnecting...");
        WiFi.begin(ssid, password);
        while (WiFi.status() != WL_CONNECTED) {
          delay(1000);
          //Serial.println("Connecting to WiFi...");
        }
        //Serial.println("Connected to WiFi");
        server_ip = WiFi.gatewayIP();
      }
      break;

    case REGISTERING:
      if (robotID == "") {
        if (client.connected() && client.available() > 0) {
          String message = client.readStringUntil('\n');
          //Serial.print("Message from server: ");
          //Serial.println(message);
          if (message.startsWith("SPIN")) {
            Serial.println("START_SPINNING");
            //Serial.println("Received READY_TO_REGISTER, spinning initiated");
          } else if (message.startsWith("REGISTER_COMPLETE")) {
            robotID = message.substring(message.indexOf(' ') + 1);
            //Serial.println("Registration complete. Robot ID: " + robotID);
            Serial.println("STOP");
            currentState = REGISTERED;
          }
        }
      } else {
        currentState = REGISTERED;
      }
      break;

    case REGISTERED:
      if (client.connected() && (robotID != "")) {
        client.println("REQUEST_LOCATION_UPDATE " + robotID);
        //Serial.println("REQUEST_LOCATION_UPDATE with robotID sent to server");
        currentState = LOCATION_REQUEST;
      } else {
        //Serial.println("Server connection lost, reconnecting...");
        currentState = CONNECTING;
      }
      break;

    case LOCATION_REQUEST:
      if (millis() - lastRequestTime >= requestInterval) {
        lastRequestTime = millis();
        if (client.connected() && (robotID != "")) {
          client.println("REQUEST_LOCATION_UPDATE " + robotID);
          //Serial.println("REQUEST_LOCATION_UPDATE with robotID sent to server");
        } else {
          //Serial.println("Server connection lost, reconnecting...");
          currentState = CONNECTING;
        }
      }

      if (client.connected() && client.available() > 0) {
        String message = client.readStringUntil('\n');
        //Serial.print("Message from server: ");
        //Serial.println(message);
        if (message.startsWith("LOCATION_UPDATE") && message.endsWith(robotID)) {  // "LOCATION_UPDATE x:x_now;y:y_now;yaw:yaw_now robotID"
          // Extract x, y, yaw values
          int xIndex = message.indexOf("x:") + 2;
          int yIndex = message.indexOf("y:") + 2;
          int yawIndex = message.indexOf("yaw:") + 4;

          String xValue = message.substring(xIndex, message.indexOf(';', xIndex));
          String yValue = message.substring(yIndex, message.indexOf(';', yIndex));
          String yawValue = message.substring(yawIndex, message.lastIndexOf(' '));

          // Create a formatted string for the STM32: "x;y;yaw"
          currentLocation = xValue + ";" + yValue + ";" + yawValue;
          Serial.println("LOCATION_UPDATE " + currentLocation);
          currentState = TARGET_REQUEST;
        }
      }
      break;

    case TARGET_REQUEST:
      if (millis() - lastRequestTime >= requestInterval) {
        lastRequestTime = millis();
        if (client.connected() && (robotID != "")) {
          client.println("REQUEST_TARGET_UPDATE " + robotID);
        } else {
          //Serial.println("Server connection lost, reconnecting...");
          currentState = CONNECTING;
        }
      }

      if (client.connected() && client.available() > 0) {
        String message = client.readStringUntil('\n');
        //Serial.print("Message from server: ");
        //Serial.println(message);
        if (message.startsWith("TARGET_UPDATE") && message.endsWith(robotID)) {  // "LOCATION_UPDATE x:x_next;y:y_next;yaw:yaw_next robotID"
          // Extract x, y, yaw values
          int xIndex = message.indexOf("x:") + 2;
          int yIndex = message.indexOf("y:") + 2;
          int yawIndex = message.indexOf("yaw:") + 4;

          String xValue = message.substring(xIndex, message.indexOf(';', xIndex));
          String yValue = message.substring(yIndex, message.indexOf(';', yIndex));
          String yawValue = message.substring(yawIndex, message.lastIndexOf(' '));

          // Create a formatted string for the STM32: "x;y;yaw"
          targetLocation = xValue + ";" + yValue + ";" + yawValue;
          Serial.println("TARGET_UPDATE " + targetLocation);
          currentState = LOCATION_REQUEST;

        } else if (message.startsWith("STOP_MOVEMENT")) {
          //Serial.println("Received STOP_MOVEMENT, listening for further commands");
          currentState = END;
          Serial.println("STOP");
        }
      }
      break;

    case END:
      if (client.connected() && client.available() > 0) {
        String message = client.readStringUntil('\n');
        Serial.print("Message from server: ");
        Serial.println(message);
        /*if (message.startsWith("TARGET_UPDATE")) {
          String targetData = message.substring(message.indexOf(' ') + 1);  // "x_next;y_next;yaw_next;robotID"
          //Serial.println("Location update: " + locationData);
          Serial.println("TARGET_UPDATE " + targetData);
        } else if (message == "STOP_MOVEMENT") {
          //Serial.println("Received STOP_MOVEMENT, listening for further commands");
          currentState = REGISTERED;
        }*/
      }
      break;
  }
  if ((currentState != CONNECTING) && !client.connected()) {
    //Serial.println("Server connection lost, reconnecting...");
    currentState = CONNECTING;
  }

  // Send UDP broadcast with robot status
  if (robotID != "") {
    String udpMessage = robotID + ";" + currentLocation + ";" + targetLocation;
    udp.beginPacket(udpAddress, udpPort);
    udp.print(udpMessage);
    udp.endPacket();
  }
}


/*
// Connect to ROS server
  if (!client.connect(server_ip, server_port)) {
    Serial.println("Connection to server failed");
    return;
  }
  Serial.println("Connected to server!");

  // Start registration process
  client.println("REGISTER");
*/

/*
if (client.connected()) {
    // Handle incoming messages from ROS
    while (client.available() > 0) {
      String message = client.readStringUntil('\n');
      if (message == "READY_TO_REGISTER") {
        // Start spinning
        Serial.println("START_SPINNING");
        Serial.println("Received READY_TO_REGISTER, spinning initiated");
      } else if (message.startsWith("REGISTRATION_COMPLETE")) {
        // Extract and save robot ID
        robotID = message.substring(message.indexOf(' ') + 1);
        Serial.println("Registration complete. Robot ID: " + robotID);

        // Stop spinning
        Serial.println("STOP_SPINNING");
      } else if (message.startsWith("LOCATION_UPDATE")) {
        // Parse and handle location and target information
        String locationData = message.substring(message.indexOf(' ') + 1);
        Serial.println("Location update: " + locationData);

        // Optionally send the data to STM32 or handle it within ESP
        Serial.println("LOCATION_UPDATE " + locationData);
      }
    }

    // Periodic location request
    static unsigned long lastRequestTime = 0;
    if (millis() - lastRequestTime >= 1000) {
      lastRequestTime = millis();
      client.println("REQUEST_LOCATION_UPDATE");
    }

  } else {
    // Attempt to reconnect if connection is lost
    if (!client.connect(server_ip, server_port)) {
      Serial.println("Reconnection to server failed");
      delay(1000);
    } else {
      Serial.println("Reconnected to server");
      if (robotID != "") {
        client.println("RECONNECT " + robotID);
      }
    }
  }
*/