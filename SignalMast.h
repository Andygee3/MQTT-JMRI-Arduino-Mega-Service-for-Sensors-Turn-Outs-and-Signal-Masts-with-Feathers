// SignalMast.h
#ifndef SignalMast_h
#define SignalMast_h

#include <Arduino.h>

class SignalMast {
  private:
    String jMRIId;
    int pwmBoard;
    int prPin;   // Proceed (green)
    int caPin;   // Caution (single yellow)
    int cpPin;   // Preliminary Caution (second yellow)
    int daPin;   // Danger (red)
    int offVal;  // PWM value for "off"
    int litVal;  // PWM value for "on"

    int axPins[6]; // Ax1..Ax6 = feathers One..Six

  public:
    SignalMast();
    SignalMast(String id,
               int board,
               int pr, int ca, int cp, int da,
               int ov, int lv,
               int ax1 = -1, int ax2 = -1, int ax3 = -1,
               int ax4 = -1, int ax5 = -1, int ax6 = -1);

    String getName() const     { return jMRIId; }
    int    getPwmBoard() const { return pwmBoard; }
    int    getPrPin() const    { return prPin; }
    int    getCaPin() const    { return caPin; }
    int    getCpPin() const    { return cpPin; }
    int    getDaPin() const    { return daPin; }
    int    getOffVal() const   { return offVal; }
    int    getLitVal() const   { return litVal; }

    int getFeatherPin(int idx) const {
      if (idx < 1 || idx > 6) return -1;
      return axPins[idx - 1];
    }
};

#endif
