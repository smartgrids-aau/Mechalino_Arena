#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

/* Wi-Fi and Server Configuration */
const char* ssid = "ROBOT_AP";
const char* password = "Robot4321";
const uint16_t serverPort = 5000;
const int udpPort = 4210;                    // UDP broadcast port
const char* udpAddress = "255.255.255.255";  // Broadcast address
IPAddress serverIP;

/* Wi-Fi and UDP Objects */
WiFiClient client;
WiFiUDP udp;

/* Robot State Variables */
String robotID = "";
String currentLocation = "";
String targetLocation = "";
String pathLocations = "";

/* Enumeration for Robot States */
enum RobotState {
  CONNECTING,
  REGISTERING,
  SPINNING,
  REGISTERED,
  LOCATION_REQUEST,
  //TARGET_REQUEST,
  PATH_REQUEST,
  END
};
RobotState currentState = CONNECTING;

/* Timing Variables */
unsigned long lastRequestTime = 0;
unsigned long requestInterval = 100;  // sending request every 100ms
bool sentRequest = false;
bool pathRequested = false;

/* Function Prototypes */
void setup();
void loop();
void sendTCP(const String& message);
void handleState(RobotState& state);
String listenToSerial();
void sendStatusUDP(const String& stmData);

/* Setup Function */
void setup() {
  Serial.begin(9600);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  // Connect to Wi-Fi
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
  }
  serverIP = WiFi.gatewayIP();

  // Initialize UDP
  udp.begin(udpPort);

  currentState = CONNECTING;
}

/* Main Loop */
void loop() {
  handleState(currentState);

  // Optional: Uncomment to send status over UDP for monitoring
  // String stmData = listenToSerial();
  // sendStatusUDP(stmData);

  delay(10);
}

/**
 * @brief Send a message over TCP to the server.
 * @param message The message to send.
 */
void sendTCP(const String& message) {
  if (millis() - lastRequestTime >= requestInterval) {
    lastRequestTime = millis();
    if (!sentRequest) {
      sentRequest = true;
      client.print(message);
    }
  }
}

/**
 * @brief Handle the current state of the robot.
 * @param state Reference to the current robot state.
 */
void handleState(RobotState& state) {
  String messageToSend = "";

  switch (state) {
    case CONNECTING:
      // Connect to ROS server
      if (WiFi.status() == WL_CONNECTED) {
        if (client.connect(serverIP, serverPort)) {
          if (robotID == "") {
            state = REGISTERING;
            messageToSend = "REGISTER";
            sendTCP(messageToSend);
          } else {
            state = REGISTERED;
          }
        } else {
          delay(1000);  // Wait a bit before retrying
        }
      } else {
        WiFi.begin(ssid, password);
        while (WiFi.status() != WL_CONNECTED) {
          delay(1000);
        }
        serverIP = WiFi.gatewayIP();
      }
      break;

    case REGISTERING:
      if (robotID == "") {
        if (client.connected() && client.available() > 0) {
          String message = client.readStringUntil('\n');
          if (message.startsWith("SPIN")) {
            Serial.println("START_SPINNING");
            state = SPINNING;
          }
        }
      } else {
        state = REGISTERED;
      }
      break;

    case SPINNING:
      if (client.connected() && client.available() > 0) {
        String message = client.readStringUntil('\n');
        if (message.startsWith("REGISTER_COMPLETE")) {
          robotID = message.substring(message.indexOf(' ') + 1);
          Serial.println("STOP");
          state = REGISTERED;
        }
      }
      break;

    case REGISTERED:
      if (client.connected() && (robotID != "")) {
        state = LOCATION_REQUEST;
        sentRequest = false;
      } else {
        state = CONNECTING;
      }
      break;

    case LOCATION_REQUEST:
      if (client.connected() && (robotID != "")) {
        messageToSend = "REQUEST_LOCATION_UPDATE " + robotID;
        sendTCP(messageToSend);
      } else {
        state = CONNECTING;
      }

      if (client.connected() && client.available() > 0) {
        String message = client.readStringUntil('\n');
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

          if (!pathRequested) {
            state = PATH_REQUEST;
            pathRequested = true;
          }
          sentRequest = false;
        } else if (message.startsWith("STOP_MOVEMENT")) {
          state = END;
          Serial.println("STOP");
        } else if (message.startsWith("REGISTER")) {
          robotID = "";
          state = CONNECTING;
        }
      }
      break;

      /*case TARGET_REQUEST:
      if (client.connected() && (robotID != "")) {
        messageToSend = "REQUEST_TARGET_UPDATE " + robotID;
        sendTCP(messageToSend);
      } else {
        state = CONNECTING;
      }

      if (client.connected() && client.available() > 0) {
        String message = client.readStringUntil('\n');
        if (message.startsWith("TARGET_UPDATE") && message.endsWith(robotID)) {  // "TARGET_UPDATE x:x_next;y:y_next robotID"
          // Extract x, y values
          int xIndex = message.indexOf("x:") + 2;
          int yIndex = message.indexOf("y:") + 2;

          String xValue = message.substring(xIndex, message.indexOf(';', xIndex));
          String yValue = message.substring(yIndex, message.indexOf(' ', yIndex));

          // Create a formatted string for the STM32: "x;y"
          targetLocation = xValue + ";" + yValue;
          Serial.println("TARGET_UPDATE " + targetLocation);
          state = LOCATION_REQUEST;
          sentRequest = false;
        } else if (message.startsWith("STOP_MOVEMENT")) {
          state = END;
          Serial.println("STOP");
        } else if (message.startsWith("REGISTER")) {
          robotID = "";
          state = CONNECTING;
        }
      }
      break;*/

    case PATH_REQUEST:
      if (client.connected() && (robotID != "")) {
        messageToSend = "REQUEST_PATH_UPDATE " + robotID;
        sendTCP(messageToSend);
      } else {
        state = CONNECTING;
      }

      if (client.connected() && client.available() > 0) {
        String message = client.readStringUntil('\n');
        if (message.startsWith("PATH_UPDATE") && message.endsWith(robotID)) {  // "PATH_UPDATE x:x0_next:x1_next:xn_next;y:y0_next:y1_next:yn_next;amount_coordinates robotID"
          // Extract x, y values
          int xIndex = message.indexOf("x:") + 2;
          int yIndex = message.indexOf("y:") + 2;
          int amountIndex = message.indexOf(';', yIndex) + 1;  // After the y values

          String xValues = message.substring(xIndex, message.indexOf(';', xIndex));
          String yValues = message.substring(yIndex, message.indexOf(';', yIndex));
          String amountCoordinates = message.substring(amountIndex, message.indexOf(' ', amountIndex));

          // Create a formatted string for the STM32: "x;y;amount"
          pathLocations = xValues + ";" + yValues + ";" + amountCoordinates;
          Serial.println("PATH_UPDATE " + pathLocations);
          state = LOCATION_REQUEST;
          sentRequest = false;
        } else if (message.startsWith("STOP_MOVEMENT")) {
          state = END;
          Serial.println("STOP");
        } else if (message.startsWith("REGISTER")) {
          robotID = "";
          state = CONNECTING;
        }
      }
      break;

    case END:
      if (client.connected() && client.available() > 0) {
        sentRequest = false;
        pathRequested = false;
        String message = client.readStringUntil('\n');
        Serial.print("Message from server: ");
        Serial.println(message);
      }
      break;
  }

  // Reconnect if disconnected
  if ((state != CONNECTING) && !client.connected()) {
    state = CONNECTING;
    sentRequest = false;
  }
}

/**
 * @brief Listen to serial input from STM32F4.
 * @return The data received from STM32F4.
 */
String listenToSerial() {
  String stmData = "";
  if (Serial.available()) {
    stmData = Serial.readStringUntil('\n');
  }
  return stmData;
}

/**
 * @brief Send the robot's status over UDP.
 * @param stmData The data received from STM32F4.
 */
void sendStatusUDP(const String& stmData) {
  // Send UDP broadcast with robot status
  if (robotID != "") {
    String udpMessage = robotID + ";" + currentLocation + ";" + stmData;
    udp.beginPacket(udpAddress, udpPort);
    udp.print(udpMessage);
    udp.endPacket();
  }
}