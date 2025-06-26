#include "wifi_server.h"
#include <WiFi.h>
#include <WebServer.h>
#include "motor_control.h"
#include "odometry.h"

extern Odometry currentPose;  // Shared from main.cpp

WebServer server(80);

void handleRoot() {
  String page = R"rawliteral(
  <!DOCTYPE html>
  <html>
  <head>
    <title>ESP32 Robot</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
      body { font-family: sans-serif; text-align: center; margin-top: 20px; }
      button {
        width: 100px; height: 40px; margin: 5px;
        font-size: 16px; border-radius: 6px;
        background-color: #4CAF50; color: white; border: none;
      }
      canvas { border: 1px solid #ccc; margin-top: 20px; }
    </style>
    <script>
      let canvas, ctx, points = [];
      function sendCmd(cmd) {
        fetch("/cmd?move=" + cmd).catch(err => console.log(err));
      }
      function fetchPose() {
        fetch("/pose")
          .then(res => res.json())
          .then(data => {
            points.push({x: data.x, y: data.y});
            if (points.length > 1000) points.shift();
            drawPath();
          });
      }
      function drawPath() {
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        ctx.beginPath();
        if (points.length > 0) {
          ctx.moveTo(points[0].x * 100 + 250, 250 - points[0].y * 100);
          for (let i = 1; i < points.length; i++) {
            ctx.lineTo(points[i].x * 100 + 250, 250 - points[i].y * 100);
          }
        }
        ctx.strokeStyle = "blue";
        ctx.lineWidth = 2;
        ctx.stroke();
      }
      window.onload = () => {
        canvas = document.getElementById("canvas");
        ctx = canvas.getContext("2d");
        setInterval(fetchPose, 100);
      }
    </script>
  </head>
  <body>
    <h2>Robot Control</h2>
    <div>
      <button onclick="sendCmd('F')">Forward</button>
      <button onclick="sendCmd('L')">Left</button>
      <button onclick="sendCmd('S')">Stop</button>
      <button onclick="sendCmd('R')">Right</button>
      <button onclick="sendCmd('B')">Backward</button>
    </div>
    <canvas id="canvas" width="500" height="500"></canvas>
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
  server.send(204);
}

void setupWiFiServer() {
  WiFi.softAP("Robot", "28022005");
  Serial.println("Access Point started: http://192.168.4.1");

  server.on("/", handleRoot);
  server.on("/cmd", handleCommand);

  server.on("/pose", []() {
    String json = String("{\"x\":") + currentPose.x +
                  ",\"y\":" + currentPose.y +
                  ",\"theta\":" + currentPose.theta +
                  ",\"v\":" + currentPose.linear_velocity +
                  ",\"omega\":" + currentPose.angular_velocity + "}";
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", json);
  });

  server.begin();
}