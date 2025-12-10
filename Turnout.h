// Turnout.h
#ifndef Turnout_h
#define Turnout_h

#include <Arduino.h>

class Turnout {
  private:

  public: 
    String jMRIId;
    int thrownVal;
    int closedVal;
    int pwmBoard;
    int pwmPin;   
    int currentPWMVal;
    int previousMillis;
    int requiredPWMVal;
    Turnout();
    Turnout(String JMRIId, int PwmBoard, int PwmPin, int ThrownVal, int ClosedVal);

    
    String getName() { return jMRIId; }
    int getTv() { return thrownVal; }
    int getCv() { return closedVal; }
    int getPwmBoard() { return pwmBoard; }
    int getPwmPin() { return pwmPin; }
    int getPwmVal() { return currentPWMVal; }
    int getPrevMillis() { return previousMillis; }
    int getrequiredPWMVal() { return requiredPWMVal; }    

};

#endif
