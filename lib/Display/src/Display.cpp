#include "Display.h"

Display::Display() {
  
}

/*
 * Initialization of the display.
 * Sets basic graphic settings and shows the team's logo.
 */
void Display::begin() {
  display.begin(DISPLAY_ADDR, true);
  display.setRotation(2);
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.clearDisplay();
  display.display();
  showLogo();
}

/**
 * Displays the team's logo.
 */
void Display::showLogo() {
  display.clearDisplay();
  display.drawBitmap(44, 4,  bitmap_logo_isaac, 41, 58, 1);
  display.display();
}

/**
 * Displays WiFi status and properties.
 */
void Display::showWifi() {
  display.clearDisplay();
  display.drawBitmap(0, 0,  bitmap_logo_wifi, 26, 21, 1);
  display.setCursor(0, 32);
  display.printf("IP: %s\n", WiFi.localIP().toString().c_str());
  display.display();
}

/**
 * Displays battery status.
 */
void Display::showBattery() {
  display.clearDisplay();
  display.drawBitmap(0, 0,  bitmap_logo_bat, 23, 11, 1);
  display.setCursor(0, 24);
  display.printf("Voltage:  %.2fV\n\n", battery.readVoltage());
  display.printf("Charge:   %.1f%%\n", battery.chargePercent());
  display.display();
}

/**
 * Display software version, and module address.
 */
void Display::showVersion() {
  display.clearDisplay();
  display.drawBitmap(0, 0,  bitmap_logo_upd, 24, 24, 1);
  display.setCursor(0, 32);
  display.printf("Version: %s\n\n", VERSION);
  display.printf("Can ID:  %#04X",CAN_ID);
  display.display();
}

/**
 * Handles display via recorded interrupts. 
 * This function needs to be called as often as possible.
 * If no button is pressed for more than #MENUTIMEOUT automatically returns to the logo.
 */
void Display::handleGUI() {
  bool change = false;

  if(nav > 0) {
    change = true;
    nav--;
    menupos++;
    if (menupos >= NMENUS) menupos = 0;
    else menutime = millis();
  } else if(menupos != 0 && millis() - menutime > (MENUTIMEOUT * 1000)) {
    change = true;
    menupos = 0;
  }

  switch (menupos) {
    case 0:
      if(change) showLogo();
      break;
    case 1:
      if (change) showBattery();
      break;
    case 2:
      if(change) showWifi();
      break;
    case 3:
      if (change) showVersion();
      break;
  }
}

/**
 * NAV button ISR.
 */
void Display::navInterrupt() {
  int now = millis();
  if (now - lastnav > DEBOUNCE) {
    nav++;
    lastnav = now;
  }
}


/**
 * OK button ISR.
 */
void Display::okInterrupt() {
  int now = millis();
  if (now - lastok > DEBOUNCE) {
    ok++;
    lastok = now;
  }
}