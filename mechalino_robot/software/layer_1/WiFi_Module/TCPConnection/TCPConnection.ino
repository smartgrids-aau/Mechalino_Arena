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
char xValues[250] = "";
char yValues[250] = "";
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
unsigned long lastReceivedTime = 0;  // Keeps track of the last time data was received from ROS
unsigned long lastSpinTime = 0;
unsigned long requestInterval = 150;  // sending request in ms
unsigned long previousLoopTime = 0;
const unsigned long loopInterval = 10;  // 10ms
bool sentRequest = false;
bool pathRequested = false;

/* New Timing Variables */
unsigned long wifiConnectStartTime = 0;
const unsigned long wifiTimeout = 10000;  // 10 seconds
unsigned long clientConnectStartTime = 0;
const unsigned long clientConnectInterval = 5000;  // 5 seconds

/* Serial Buffer Definitions */
#define SERIAL_BUFFER_SIZE 512
char serial_buffer[SERIAL_BUFFER_SIZE];
size_t serial_index = 0;

/* Client Buffer for Non-blocking Reads */
#define CLIENT_BUFFER_SIZE 528
char clientBuffer[CLIENT_BUFFER_SIZE];
size_t clientBufferIndex = 0;

/* Function Prototypes */
void setup();
void loop();
void sendTCP(const char* message);
void handleState(RobotState& state);
bool listenToSerial(char* stmData, size_t maxLength);
void sendStatusUDP(const char* stmData);
bool readLineFromClient(char* destBuffer, size_t destBufferSize);

/* Setup Function */
void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  // Start Wi-Fi connection without blocking
  WiFi.begin(ssid, password);
  wifiConnectStartTime = millis();

  // Initialize UDP
  udp.begin(udpPort);

  currentState = CONNECTING;

  lastReceivedTime = millis();  // Initialize to current time
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
  yield();  // Allow background tasks to run
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
      // Non-blocking Wi-Fi connection
      if (WiFi.status() != WL_CONNECTED) {
        if (millis() - wifiConnectStartTime >= wifiTimeout) {
          Serial.println("Wi-Fi connection timeout.");
          wifiConnectStartTime = millis();  // Reset timer
        }
        // Return to allow other tasks to run
        return;
      }

      // Wi-Fi is connected
      serverIP = WiFi.gatewayIP();

      // Non-blocking server connection
      if (!client.connected()) {
        if (millis() - clientConnectStartTime >= clientConnectInterval) {
          clientConnectStartTime = millis();  // Reset timer
          if (client.connect(serverIP, server_port)) {
            state = REGISTERING;
            if (strlen(robotID) == 0) {
              strcpy(messageToSend, "REGISTER");
            } else {
              snprintf(messageToSend, sizeof(messageToSend), "REGISTER %s", robotID);
            }
            sendTCP(messageToSend);
            lastReceivedTime = millis();  // Update last received time
          } else {
            Serial.println("Server connection failed. Retrying...");
          }
        }
        // Return to allow other tasks to run
        return;
      }
      break;

    case REGISTERING:
      if (client.connected() && readLineFromClient(message, sizeof(message))) {
        lastReceivedTime = millis();  // Update last received time
        if (strncmp(message, "SPIN", 4) == 0) {
          Serial.println("START_SPINNING");
          state = SPINNING;
        } else if (strncmp(message, "REGISTERED", 10) == 0) {
          state = REGISTERED;
        }
      }

      if ((millis() - lastReceivedTime) >= 10000) {  // 10 seconds timeout
        Serial.println("STOP");
        state = CONNECTING;   // Switch state to CONNECTING
        sentRequest = false;  // Reset request flag
      }
      break;

    case SPINNING:
      if (client.connected()) {
        if ((millis() - lastSpinTime) >= 2000) {  // 10 seconds timeout
          lastSpinTime = millis();
          Serial.println("START_SPINNING");
        }
        if (readLineFromClient(message, sizeof(message))) {
          lastReceivedTime = millis();  // Update last received time
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

      if (client.connected() && readLineFromClient(message, sizeof(message))) {
        lastReceivedTime = millis();  // Update last received time
        if (strncmp(message, "LOCATION_UPDATE", 15) == 0 && strstr(message, robotID) != NULL) {
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
          state = CONNECTING;
          Serial.println("STOP");
        }
      }

      if ((millis() - lastReceivedTime) >= 10000) {  // 10 seconds timeout
        state = CONNECTING;                          // Switch state to CONNECTING
        sentRequest = false;                         // Reset request flag
      }
      break;

    case PATH_REQUEST:
      if (client.connected() && (strlen(robotID) > 0)) {
        snprintf(messageToSend, sizeof(messageToSend), "REQUEST_PATH_UPDATE %s", robotID);
        sendTCP(messageToSend);
      } else {
        state = CONNECTING;
      }

      if (client.connected() && readLineFromClient(message, sizeof(message))) {
        lastReceivedTime = millis();  // Update last received time
        if (strncmp(message, "PATH_UPDATE", 11) == 0 && strstr(message, robotID) != NULL) {
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
          state = CONNECTING;
          Serial.println("STOP");
        }
      }

      if ((millis() - lastReceivedTime) >= 10000) {  // 10 seconds timeout
        state = CONNECTING;                          // Switch state to CONNECTING
        sentRequest = false;                         // Reset request flag
      }
      break;

    case END:
      if (client.connected() && readLineFromClient(message, sizeof(message))) {
        lastReceivedTime = millis();  // Update last received time
        Serial.print("Message from server: ");
        Serial.println(message);
      }
      break;
  }

  // Reconnect if disconnected
  if ((state != CONNECTING) && !client.connected()) {
    Serial.println("STOP");
    state = CONNECTING;
    sentRequest = false;
  }
}

/**
 * @brief Read a line from the client non-blockingly.
 * @param destBuffer Buffer to store the message.
 * @param destBufferSize Size of the buffer.
 * @return True if a complete message was read, false otherwise.
 */
bool readLineFromClient(char* destBuffer, size_t destBufferSize) {
  while (client.available() > 0) {
    char c = client.read();
    if (c == '\n') {
      // Null-terminate and copy the message
      clientBuffer[clientBufferIndex] = '\0';
      if (clientBufferIndex < destBufferSize) {
        strncpy(destBuffer, clientBuffer, destBufferSize - 1);
        destBuffer[destBufferSize - 1] = '\0';
      } else {
        strncpy(destBuffer, clientBuffer, destBufferSize - 1);
        destBuffer[destBufferSize - 1] = '\0';
      }
      clientBufferIndex = 0;
      return true;
    } else {
      if (clientBufferIndex < CLIENT_BUFFER_SIZE - 1) {
        clientBuffer[clientBufferIndex++] = c;
      } else {
        clientBufferIndex = 0;
      }
    }
    yield();  // Allow background tasks to run
  }
  return false;
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
      }

      // Reset the serial buffer index for the next message
      serial_index = 0;
      return true;  // Complete message received
    } else {
      if (serial_index < SERIAL_BUFFER_SIZE - 1) {
        serial_buffer[serial_index++] = c;
      } else {
        serial_index = 0;
      }
    }
    yield();  // Allow background tasks to run
  }

  return false;  // No complete message received yet
}

/**
 * @brief Send the robot's status over UDP.
 * @param stmData The data received from STM32F4.
 */
void sendStatusUDP(const char* stmData) {
  // Send UDP broadcast with robot status
  if (strlen(robotID) > 0 && strlen(currentLocation) > 0 && strlen(stmData) > 0) {
    int len = snprintf(udpMessage, sizeof(udpMessage), "%s;%s;%s", robotID, currentLocation, stmData);
    if (len < 0 || len >= (int)sizeof(udpMessage)) {
      return;  // Avoid sending an incomplete message
    }

    // Ensure null-termination
    udpMessage[sizeof(udpMessage) - 1] = '\0';

    // Send UDP packet
    udp.beginPacket(udpAddress, udpPort);
    udp.write((uint8_t*)udpMessage, strlen(udpMessage));
    udp.endPacket();

    // Include path data if updated
    if (strlen(pathLocations) > 0) {
      int lenPath = snprintf(udpMessage, sizeof(udpMessage), "%s", pathLocations);
      if (lenPath < 0 || lenPath >= (int)sizeof(udpMessage)) {
        return;
      }
      udp.beginPacket(udpAddress, udpPort);
      udp.write((uint8_t*)udpMessage, strlen(udpMessage));
      udp.endPacket();
    }
  }
}
