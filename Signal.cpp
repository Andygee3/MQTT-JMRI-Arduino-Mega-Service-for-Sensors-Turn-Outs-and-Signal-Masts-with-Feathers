// Turnout.cpp
#include "Signal.h"

Signal::Signal() {
  currentPWMVal = 0;
}

Signal::Signal(String JMRIId, int PwmBoard, int PwmPin, int ThrownVal, int ClosedVal) {
  jMRIId = JMRIId;
  pwmBoard = PwmBoard;
  pwmPin = PwmPin;
  thrownVal = ThrownVal;
  closedVal = ClosedVal;
  currentPWMVal = 0;
}


