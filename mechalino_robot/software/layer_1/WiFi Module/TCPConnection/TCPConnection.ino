#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

// Wi-Fi Credentials
const char* ssid = "ROBOT_AP";
const char* password = "Robot4321";

// Server Configuration
const uint16_t server_port = 5000;
const int udpPort = 4210;                    // UDP broadcast port
const char* udpAddress = "255.255.255.255";  // Broadcast address
IPAddress serverIP;

/* Wi-Fi and UDP Objects */
WiFiClient client;
WiFiUDP udp;

/* Robot State Variables */
char robotID[3] = "";           // Two-digit integer + null terminator
char currentLocation[32] = "";  // "x;y;yaw" format
char pathLocations[512] = "";   // "PATH:x0:x1:...:xn;y0:y1:...:yn;amount"

/* Global Buffers */
char messageToSend[32] = "";
char message[528] = "";  // Buffer for incoming messages
char xValue[8] = "";
char yValue[8] = "";
char yawValue[10] = "";
char xValues[250] = "";  // Adjust size as needed
char yValues[250] = "";  // Adjust size as needed
char amountCoordinates[4] = "";
char udpMessage[1024];
char stmData[256] = "";

/* Enumeration for Robot States */
enum RobotState {
  CONNECTING,
  REGISTERING,
  SPINNING,
  REGISTERED,
  LOCATION_REQUEST,
  PATH_REQUEST,
  END
};
RobotState currentState = CONNECTING;

/* Timing Variables */
unsigned long lastRequestTime = 0;
unsigned long lastUdpTime = 0;
unsigned long requestInterval = 150;  // sending request in ms
unsigned long sendUdpInterval = 500;
unsigned long previousLoopTime = 0;
const unsigned long loopInterval = 10;  // 10ms
bool sentRequest = false;
bool pathRequested = false;

/* Serial Buffer Definitions */
#define SERIAL_BUFFER_SIZE 512
char serial_buffer[SERIAL_BUFFER_SIZE];
size_t serial_index = 0;

/* Function Prototypes */
void setup();
void loop();
void sendTCP(const char* message);
void handleState(RobotState& state);
bool listenToSerial(char* stmData, size_t maxLength);
void sendStatusUDP(const char* stmData);

/* Setup Function */
void setup() {
  Serial.begin(500000);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  // Connect to Wi-Fi
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi!");
  serverIP = WiFi.gatewayIP();

  // Initialize UDP
  udp.begin(udpPort);

  currentState = CONNECTING;
}

/* Main Loop */
void loop() {
  unsigned long currentTime = millis();
  if (currentTime - previousLoopTime >= loopInterval) {
    previousLoopTime = currentTime;

    handleState(currentState);

    if (listenToSerial(stmData, sizeof(stmData))) {
      // Send status over UDP
      sendStatusUDP(stmData);
    }
  }
}

/**
 * @brief Send a message over TCP to the server.
 * @param message The message to send.
 */
void sendTCP(const char* message) {
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
  switch (state) {
    case CONNECTING:
      // Connect to ROS server
      if (WiFi.status() == WL_CONNECTED) {
        if (client.connect(serverIP, server_port)) {
          if (strlen(robotID) == 0) {
            state = REGISTERING;
            strcpy(messageToSend, "REGISTER");
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
      if (strlen(robotID) == 0) {
        if (client.connected() && client.available() > 0) {
          client.readBytesUntil('\n', message, sizeof(message) - 1);
          message[sizeof(message) - 1] = '\0';  // Ensure null-termination
          if (strncmp(message, "SPIN", 4) == 0) {
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
        client.readBytesUntil('\n', message, sizeof(message) - 1);
        message[sizeof(message) - 1] = '\0';  // Ensure null-termination
        if (strncmp(message, "REGISTER_COMPLETE", 17) == 0) {
          // Extract robotID
          char* idStart = strchr(message, ' ');
          if (idStart != NULL) {
            idStart++;
            strncpy(robotID, idStart, sizeof(robotID) - 1);
            robotID[sizeof(robotID) - 1] = '\0';  // Ensure null-termination
          }
          Serial.println("STOP");
          state = REGISTERED;
        }
      }
      break;

    case REGISTERED:
      if (client.connected() && (strlen(robotID) > 0)) {
        state = LOCATION_REQUEST;
        sentRequest = false;
      } else {
        state = CONNECTING;
      }
      break;

    case LOCATION_REQUEST:
      if (client.connected() && (strlen(robotID) > 0)) {
        snprintf(messageToSend, sizeof(messageToSend), "REQUEST_LOCATION_UPDATE %s", robotID);
        sendTCP(messageToSend);
      } else {
        state = CONNECTING;
      }

      if (client.connected() && client.available() > 0) {
        client.readBytesUntil('\n', message, sizeof(message) - 1);
        message[sizeof(message) - 1] = '\0';                                                     // Ensure null-termination
        if (strncmp(message, "LOCATION_UPDATE", 15) == 0 && strstr(message, robotID) != NULL) {  // "LOCATION_UPDATE x:x_now;y:y_now;yaw:yaw_now robotID"
          // Extract x, y, yaw values
          sscanf(message, "LOCATION_UPDATE x:%7[^;];y:%7[^;];yaw:%9s", xValue, yValue, yawValue);

          // Create a formatted string for the STM32: "x;y;yaw"
          snprintf(currentLocation, sizeof(currentLocation), "%s;%s;%s", xValue, yValue, yawValue);
          Serial.print("LOCATION_UPDATE ");
          Serial.println(currentLocation);

          if (!pathRequested) {
            state = PATH_REQUEST;
            pathRequested = true;
          }
          sentRequest = false;
        } else if (strncmp(message, "STOP_MOVEMENT", 13) == 0) {
          state = END;
          Serial.println("STOP");
        } else if (strncmp(message, "REGISTER", 8) == 0) {
          robotID[0] = '\0';
          state = CONNECTING;
        }
      }
      break;

    case PATH_REQUEST:
      if (client.connected() && (strlen(robotID) > 0)) {
        snprintf(messageToSend, sizeof(messageToSend), "REQUEST_PATH_UPDATE %s", robotID);
        sendTCP(messageToSend);
      } else {
        state = CONNECTING;
      }

      if (client.connected() && client.available() > 0) {
        client.readBytesUntil('\n', message, sizeof(message) - 1);
        message[sizeof(message) - 1] = '\0';                                                 // Ensure null-termination
        if (strncmp(message, "PATH_UPDATE", 11) == 0 && strstr(message, robotID) != NULL) {  // "PATH_UPDATE x:x0_next:x1_next:xn_next;y:y0_next:y1_next:yn_next;amount_coordinates robotID"
          // Extract x, y values

          // Parse the message
          sscanf(message, "PATH_UPDATE x:%249[^;];y:%249[^;];%3s", xValues, yValues, amountCoordinates);

          // Create a formatted string for the STM32: "x_values;y_values;amount"
          snprintf(pathLocations, sizeof(pathLocations), "%s;%s;%s", xValues, yValues, amountCoordinates);
          Serial.print("PATH_UPDATE ");
          Serial.println(pathLocations);

          state = LOCATION_REQUEST;
          sentRequest = false;
        } else if (strncmp(message, "STOP_MOVEMENT", 13) == 0) {
          state = END;
          Serial.println("STOP");
        } else if (strncmp(message, "REGISTER", 8) == 0) {
          robotID[0] = '\0';
          state = CONNECTING;
        }
      }
      break;

    case END:
      if (client.connected() && client.available() > 0) {
        sentRequest = false;
        pathRequested = false;
        client.readBytesUntil('\n', message, sizeof(message) - 1);
        message[sizeof(message) - 1] = '\0';  // Ensure null-termination
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
 * @param stmData Buffer to store data.
 * @param maxLength Maximum length of the buffer.
 */
bool listenToSerial(char* stmData, size_t maxLength) {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      // Null-terminate the accumulated string
      serial_buffer[serial_index] = '\0';

      // Copy to stmData if it fits
      if (serial_index < maxLength) {
        strncpy(stmData, serial_buffer, maxLength - 1);
        stmData[maxLength - 1] = '\0';
      } else {
        // Handle buffer overflow
        strncpy(stmData, serial_buffer, maxLength - 1);
        stmData[maxLength - 1] = '\0';
        //Serial.println("Warning: stmData buffer overflow.");
      }

      // Log the complete message
      //Serial.print("Complete Message Received: ");
      //Serial.println(stmData);

      // Reset the serial buffer index for the next message
      serial_index = 0;

      return true;  // Complete message received
    } else {
      if (serial_index < SERIAL_BUFFER_SIZE - 1) {
        serial_buffer[serial_index++] = c;
      } else {
        // Buffer overflow, reset
        //Serial.println("Error: Serial buffer overflow.");
        serial_index = 0;
      }
    }
  }

  return false;  // No complete message received yet
}

/**
 * @brief Send the robot's status over UDP.
 * @param stmData The data received from STM32F4.
 */
void sendStatusUDP(const char* stmData) {
  // Send UDP broadcast with robot status
  if ((millis() - lastUdpTime) >= sendUdpInterval) {
    lastUdpTime = millis();
    if (strlen(robotID) > 0 && strlen(currentLocation) > 0 && strlen(stmData) > 0) {
      size_t offset = 0;

      // Construct the UDP message
      offset += snprintf(udpMessage + offset, sizeof(udpMessage) - offset, "%s;%s;%s", robotID, currentLocation, stmData);

      // Ensure null-termination
      udpMessage[sizeof(udpMessage) - 1] = '\0';

      // Send UDP packet
      udp.beginPacket(udpAddress, udpPort);
      udp.write((uint8_t*)udpMessage, strlen(udpMessage));
      udp.endPacket();

      // Include path data if updated
      if (strlen(pathLocations) > 0) {
        udp.beginPacket(udpAddress, udpPort);
        udp.write((uint8_t*)pathLocations, strlen(pathLocations));
        udp.endPacket();
      }
    }
  }
}