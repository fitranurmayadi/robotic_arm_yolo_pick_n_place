#ifndef COMMAND_H
#define COMMAND_H

#include <Arduino.h>

struct Cmd {
  char id;
  int num;
  float valueX, valueY, valueZ, valueE, valueF, valueT;
};

class Command {
public:
  Command();
  // Original method to handle G-code from serial buffer
  bool handleGcode();
  // New method to handle a single G-code line passed as a String
  bool handleGcodeLine(const String &line);
  Cmd getCmd() const;
  // Modified parseLine to accept a const String&
  void parseLine(const String &line);
private:
  Cmd currentCmd;
};

#endif
