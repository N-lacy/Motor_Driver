#ifndef USSENSORCLASS_H
#define USSENSORCLASS_H

#include <Arduino.h>
#include <SoftwareSerial.h>

class US_SensorClass {
  public:
  US_SensorClass(SoftwareSerial &USSl);
  void initSensor(int baudr);
  void distance();
  // to reset, set plus to gnd, and back
  SoftwareSerial &USSerial;
  unsigned int mmDist = 1234;

  private:
  unsigned int MSByteDist = 0, LSByteDist = 0;
};

#endif
