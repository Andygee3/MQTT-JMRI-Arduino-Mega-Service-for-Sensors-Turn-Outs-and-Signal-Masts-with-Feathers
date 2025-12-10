// Sensor.h
#ifndef Sensor_h
#define Sensor_h

#include <Arduino.h>

class Sensor {
private:

public:
  String jMRIId;
  int sensorNo;
  int pin;
  int invert;
  int currentState = 0;

  Sensor();
  Sensor(String JMRIId, int SensorNo, int Pin, int Invert);

    
    void updateSensor(int NewState) {
      currentState = NewState;
    }

  String getName() {
    return jMRIId;
  }
  int getSensorNo() {
    return sensorNo;
  }
  int getPin() {
    return pin;
  }
  int getInvert() {
    return invert;
  }
  int getCurrentState() {
    return currentState;
  }
};

#endif
