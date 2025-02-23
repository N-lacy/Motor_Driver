#include "US_SensorClass.h"


US_SensorClass::US_SensorClass(SoftwareSerial &USSl)
  : USSerial(USSl) {}


void US_SensorClass::initSensor(int baudr) {
  USSerial.begin(baudr);
  USSerial.flush();
  USSerial.write(0x55);
}


void US_SensorClass::distance() {
  if(USSerial.available() >= 2) 
  {
    MSByteDist = USSerial.read(); 
    LSByteDist = USSerial.read();
    mmDist = MSByteDist * 256 + LSByteDist; 
  } else mmDist = 1234;
}
