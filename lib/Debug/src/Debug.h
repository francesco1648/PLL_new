#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>


enum class Levels{
  OFF, WARN, INFO, DEBUG
};

/**
 * Debug class that allows a finer control over serial debug.
 *
 * Any message can have a level given by the #Levels enum.
 * Only messages with level lower than the one set by using setLevel(Levels lvl) are printed.
 * The level is also written before the message in the serial output so even when all levels
 * are being printed it's easy to distinguish and eventually remove unnecessary output.
 */
class SerialDebug {
public:
  SerialDebug(HardwareSerial* serial = &Serial) : serial(serial), ended(true), level(Levels::DEBUG) {}

  void print(String st, Levels level);
  void println(String st, Levels level);
  void print(String st);
  void println(String st);

  // generic functions, accepting all parameters accepted by String
  template<class T>
  void print(T any, Levels level) {print(String(any), level);}
  template<class T>
  void println(T any, Levels level) {println(String(any), level);}
  template<class T>
  void print(T any) {print(String(any));}
  template<class T>
  void println(T any) {println(String(any));}

  void delayd(int t);
  void setLevel(Levels lvl);
  void setSerial(HardwareSerial* serial);
private:
  HardwareSerial* serial;
  String getLevel(Levels level);
  bool ended;
  Levels level;
};

// making the default instance extern so any class can use it
extern SerialDebug Debug;

#endif
