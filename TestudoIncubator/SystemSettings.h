#include "IPAddress.h"

class SystemSettings {
public:
  unsigned int id = 1;
  double setpoint_temp = 31.5; // Default target temperature
  double setpoint_humidity = 70.0; // Default target humidity
  
  IPAddress ipAddress;//(192, 168, 60, 177);
  IPAddress subnetMask;//(255, 255, 254, 0);
  
  byte macAddress[6]; //= { 0x90, 0xA2, 0xDA, 0x00, 0x52, 0xC7 };

  SystemSettings(): 
    id(1),
    setpoint_temp(31.5),
    setpoint_humidity(70.0),
    ipAddress(192, 168, 60, 177),
    subnetMask(255, 255, 254, 0),
    macAddress{ 0x90, 0xA2, 0xDA, 0x00, 0x52, 0xC7 }
   {}

};