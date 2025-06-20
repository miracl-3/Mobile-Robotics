#ifndef WIFI_SERVER_H
#define WIFI_SERVER_H

#include <WebServer.h>

extern WebServer server;  // Expose globally if needed

void setupWiFiServer();   // Call this in `setup()`

#endif