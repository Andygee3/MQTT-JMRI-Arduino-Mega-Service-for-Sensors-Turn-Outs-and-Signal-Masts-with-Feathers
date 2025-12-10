// Sensor.cpp
#include "Sensor.h"

Sensor::Sensor() {
  
}

Sensor::Sensor(String JMRIId, int SensorNo, int Pin, int Invert) {
  jMRIId = JMRIId;
  sensorNo = SensorNo;
  pin = Pin;
  invert = Invert;
}




