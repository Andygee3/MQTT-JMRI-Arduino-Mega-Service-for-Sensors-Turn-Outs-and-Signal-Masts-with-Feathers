// Signal.h
#ifndef Signal_h
#define Signal_h

#include <Arduino.h>

class Signal {
  private:

  public: 
    String jMRIId;
    int thrownVal;
    int closedVal;
    int pwmBoard;
    int pwmPin;   
    int currentPWMVal;
    Signal();
    Signal(String JMRIId, int PwmBoard, int PwmPin, int ThrownVal, int ClosedVal);

    
    String getName() { return jMRIId; }
    int getTv() { return thrownVal; }
    int getCv() { return closedVal; }
    int getPwmBoard() { return pwmBoard; }
    int getPwmPin() { return pwmPin; }
    int getPwmVal() { return currentPWMVal; }

};

#endif
