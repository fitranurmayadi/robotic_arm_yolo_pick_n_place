#include "command.h"

Command::Command() {
  currentCmd.id = -1;
  currentCmd.num = 0;
  currentCmd.valueX = currentCmd.valueY = currentCmd.valueZ =  NAN;
  currentCmd.valueE = currentCmd.valueF = currentCmd.valueT = NAN;
}

// Original handleGcode for serial buffer reading (kept for compatibility if needed elsewhere)
bool Command::handleGcode() {
  static String buffer = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (buffer.length() > 0) {
        // Check if the line starts with 'G' or 'M' before parsing
        if (buffer.charAt(0) != 'G' && buffer.charAt(0) != 'M') {
            buffer = ""; // Clear buffer for next line
            return false; // Not a G or M command
        }
        parseLine(buffer);
        buffer = "";
        return true;
      }
    } else {
      buffer += c;
    }
  }
  return false;
}

// Definition for handleGcodeLine - processes a single G-code line
bool Command::handleGcodeLine(const String &line) {
    // Check if the line starts with 'G' or 'M' before parsing
    if (line.length() == 0 || (line.charAt(0) != 'G' && line.charAt(0) != 'M')) {
        currentCmd.id = 0; // Indicate an invalid command type
        return false; // Not a G or M command
    }
    parseLine(line); // Pass the const String& directly
    return true; // Successfully parsed a G or M command (even if the number is unknown)
}

Cmd Command::getCmd() const {
  return currentCmd;
}

// Modified parseLine to accept a const String&
void Command::parseLine(const String &line) {
  // Example parsing: "G1 X10.0 Y20.0 Z5.0 F2000"
  currentCmd.id = line.charAt(0);
  // No need for validation here, as it's done in handleGcodeLine/handleGcode
  currentCmd.num = line.substring(1).toInt();

  // Reset values
  currentCmd.valueX = currentCmd.valueY = currentCmd.valueZ = NAN;
  currentCmd.valueE = currentCmd.valueF = currentCmd.valueT = NAN;

  int idx = 1;
  while (idx < line.length()) {
    char axis = line.charAt(idx);
    idx++;
    // Get the number after the axis until a space
    int nextSpace = line.indexOf(' ', idx);
    String numStr;
    if (nextSpace == -1) {
      numStr = line.substring(idx);
      idx = line.length();
    } else {
      numStr = line.substring(idx, nextSpace);
      idx = nextSpace + 1;
    }
    float val = numStr.toFloat();
    switch (axis) {
      case 'X': currentCmd.valueX = val; break;
      case 'Y': currentCmd.valueY = val; break;
      case 'Z': currentCmd.valueZ = val; break;
      case 'E': currentCmd.valueE = val; break;
      case 'F': currentCmd.valueF = val; break;
      case 'T': currentCmd.valueT = val; break;
      default: break;
    }
  }
}
