#ifndef WebManagement_h
#define WebManagement_h

#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <LEAmDNS.h>
#include <LittleFS.h>
#include <ArduinoOTA.h>
#include "Debug.h"

#include "html.h"
#include "definitions.h"

/**
 * Provides a class for managing a WiFi connection and HTTP server for remote firmware updates and configuration management.
 */
class WebManagement {
public:
  WebManagement(String configFile) : confFile(configFile), server(80) {}
  void begin(const char* ssid, const char* password, const char* hostname);
  void handle();
private:
  void setupServer();
  void setupOTA(const char* hostname);

  void handleUpdate();
  void handleConfig();

  String readFile(String path);

  String confFile;
  File tempFile;
  WebServer server;
};

#endif
