// Turnout.cpp
#include "Turnout.h"

Turnout::Turnout() {
  currentPWMVal = 1500;
  requiredPWMVal = 1500;
}

Turnout::Turnout(String JMRIId, int PwmBoard, int PwmPin, int ThrownVal, int ClosedVal) {
  jMRIId = JMRIId;
  pwmBoard = PwmBoard;
  pwmPin = PwmPin;
  thrownVal = ThrownVal;
  closedVal = ClosedVal;
  currentPWMVal = 1500;
  requiredPWMVal = 1500;
}


