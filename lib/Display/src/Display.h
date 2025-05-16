#ifndef Display_h
#define Display_h

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <WiFi.h>

#include "../../../include/definitions.h"
#include "../../../include/bitmap_logos.h"
#include "../../../include/mod_config.h"
#include "../../Battery/src/Battery.h"

/**
 * Display class, handles display itself and GUI.
 */
class Display {
public:
  Display();
  void begin();

  void handleGUI();
  void okInterrupt();
  void navInterrupt();
  void showLogo();
  void showBattery();

  void showVersion();

private:

  void showWifi();

  Adafruit_SH1106G display = Adafruit_SH1106G(DISPLAY_WIDTH, DISPLAY_HEIGHT, &Wire1, -1);

  //menu handling variables
  int ok = 0;
  int lastok = 0;
  int nav = 0;
  int lastnav = 0;
  int menupos = 0;
  int menutime = 0;
};

#endif
