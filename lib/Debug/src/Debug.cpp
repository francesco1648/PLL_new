#include "Debug.h"

/**
 * Sends a string to the Serial port with a debug level, only if it is lower than the setted one. 
 * The debug level will be sent only if it's a new line.
 * @param st String to print.
 * @param level Debug level.
 */
void SerialDebug::print(String st, Levels level) {
	if (level <= this->level) {
    String toPrint = "";
		if (ended) toPrint += getLevel(level);
		toPrint += st;
    serial->print(toPrint);
		ended = st.endsWith("\n");
	}
}

/**
 * Sends a line to the serial port with a debug level, only if it is lower than the setted one. 
 * The debug level will be sent only if it's a new line.
 * @param st String to print.
 * @param level Debug level.
 */
void SerialDebug::println(String st, Levels level) {
  print(st+"\n", level);
}

/**
 * Sends a debug string to the serial port, only debug is accepted.
 * The debug level will be sent only if it's a new line.
 * @param st String to print.
 */
void SerialDebug::print(String st) {
  print(st, Levels::DEBUG);
}

/**
 * Sends a debug line to the serial port, only debug is accepted.
 * The debug level will be sent only if it's a new line.
 * @param st String to print.
 */
void SerialDebug::println(String st) {
  println(st, Levels::DEBUG);
}

/**
 * Delay used only in debug. This should be used to avoid errors while running in competition.
 * @param t Time to wait in microseconds.
 */
void SerialDebug::delayd(int t) {
  if(level>=Levels::DEBUG) delay(t);
}

/**
 * Sets the level to be used for the debug.
 * @param lvl Debug level.
 */
void SerialDebug::setLevel(Levels level) {
  this->level = level;
}

/**
 * Get the string corresponding the actual debug level.
 * @param level Debug level.
 * @return The string representing the level.
 */
String SerialDebug::getLevel(Levels level) {
  switch (level) {
    case Levels::DEBUG: return "debug ";
    case Levels::INFO: return "information ";
    case Levels::WARN: return "warning ";
    default: return "";
  }
}

// creation of the default instance
SerialDebug Debug;
