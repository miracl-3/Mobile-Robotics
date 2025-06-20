#include "wifi_server.h"
#include <WiFi.h>
#include "motor_control.h"  // So you can call moveForward(), etc.

const char* ssid = "Robot";
const char* password = "28022005";

WebServer server(80);  // Define global server

void handleRoot() {
  String page = R"rawliteral(
  <!DOCTYPE html>
  <html>
  <head>
    <title>ESP32 Robot</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
      body { font-family: sans-serif; text-align: center; margin-top: 40px; }
      button {
        width: 120px;
        height: 50px;
        margin: 10px;
        font-size: 18px;
        border-radius: 8px;
        background-color: #4CAF50;
        color: white;
        border: none;
      }
      button:hover {
        background-color: #45a049;
      }
    </style>
    <script>
      function sendCmd(cmd) {
        fetch("/cmd?move=" + cmd).catch(err => console.log(err));
      }
    </script>
  </head>
  <body>
    <h2>Robot Control</h2>
    <div>
      <button onclick="sendCmd('F')">Forward</button><br>
      <button onclick="sendCmd('L')">Left</button>
      <button onclick="sendCmd('S')">Stop</button>
      <button onclick="sendCmd('R')">Right</button><br>
      <button onclick="sendCmd('B')">Backward</button>
    </div>
  </body>
  </html>
  )rawliteral";
  server.send(200, "text/html", page);
}

void handleCommand() {
  String move = server.arg("move");
  if (move == "F") moveForward(230);
  else if (move == "B") moveBackward(230);
  else if (move == "L") turnLeft(230);
  else if (move == "R") turnRight(230);
  else if (move == "S") stop();
  server.send(204);  // No content = faster
}

void setupWiFiServer() {
  WiFi.softAP("Robot", "28022005");
  Serial.println("Access Point started: http://192.168.4.1");

  server.on("/", handleRoot);
  server.on("/cmd", handleCommand);
  server.begin();
}

