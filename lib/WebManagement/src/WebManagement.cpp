#include "WebManagement.h"

/**
 * Initializes the WiFi connection, mDNS, OTA, and HTTP server.
 * @param ssid The SSID of the WiFi network to connect to.
 * @param password The password of the WiFi network to connect to.
 * @param hostname The hostname to use for the device.
 */
void WebManagement::begin(const char* ssid, const char* password, const char* hostname) {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(hostname);
  WiFi.begin(ssid, password);
  MDNS.begin(hostname);

  setupOTA(hostname);
  setupServer();
  server.begin();

  MDNS.addService("http", "tcp", 80);

  Debug.println("WiFi ready with IP: " + WiFi.localIP().toString());
}

/**
 * Handles incoming HTTP requests and OTA updates.
 */
void WebManagement::handle() {
  server.handleClient();
  //ArduinoOTA.handle();
  MDNS.update();
}

/**
 * Sets up the HTTP server routes.
 */
void WebManagement::setupServer() {
  server.on("/", HTTP_GET, [this]() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", serverIndex);
  });

  server.on("/getconfig", HTTP_GET, [this]() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", readFile(confFile));
  });

  server.on("/update", HTTP_POST, [this]() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    rp2040.restart();
  }, [this]() { handleUpdate(); });

  server.on("/config", HTTP_POST, [this]() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", "OK");
  }, [this]() { handleConfig(); });
}

/**
 * Sets up OTA for remote firmware updates.
 * @param hostname The hostname to use for the device.
 */
void WebManagement::setupOTA(const char* hostname) {
  ArduinoOTA.setHostname(hostname);
  ArduinoOTA.setPassword(OTA_PWD);

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {  // U_FS
      type = "filesystem";
    }
    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Debug.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Debug.println("End");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Debug.println("Progress: " + String(progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Debug.print("Error: ");
    if (error == OTA_AUTH_ERROR) {
      Debug.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Debug.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Debug.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Debug.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Debug.println("End Failed");
    }
  });

  ArduinoOTA.begin();
}

/**
 * Handles firmware updates via HTTP POST requests.
 */
void WebManagement::handleUpdate() {
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    WiFiUDP::stopAll();
    Debug.println("Update: " + String(upload.filename.c_str()));
    FSInfo i;
    LittleFS.info(i);
    uint32_t maxSketchSpace = i.totalBytes - i.usedBytes;
    if (!Update.begin(maxSketchSpace)) {  // start with max available size
      Update.printError(Serial); // TODO avoid serial without debug
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Update.printError(Serial); // TODO avoid serial without debug
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) {  // true to set the size to the current progress
      Debug.println("Update Success: " + String(upload.totalSize));
      Debug.println("Rebooting...");
    } else {
      Update.printError(Serial);
    }
  }
}

/**
 * Handles HTTP GET requests for the configuration file.
 */
void WebManagement::handleConfig() {
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    tempFile = LittleFS.open(confFile, "w");
    if (!tempFile) {
      Debug.println("Error while opening file.");
      return;
    }
    Debug.println("Upload: START");
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (tempFile) {
      size_t bytesWritten = tempFile.write(upload.buf, upload.currentSize);
      if (bytesWritten != upload.currentSize) {
        Debug.println("WRITE FAILED");
        return;
      }
    }
    Debug.println(String("Upload: WRITE, Bytes: ") + upload.currentSize);
  } else if (upload.status == UPLOAD_FILE_END) {
    if (tempFile) {
      tempFile.close();
    }
    Debug.println(String("Upload: END, Size: ") + upload.totalSize);
  }
}

/**
 * Reads a file from the file system.
 * @param path The path of the file to read.
 * @return The contents of the file as a string.
 */
String WebManagement::readFile(String path) {
  String out = "";
  File file = LittleFS.open(path, "r");
  if (file) {
    while (file.available()) out += (char) file.read();
    file.close();
  }
  return out;
}
