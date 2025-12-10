// SignalMast.cpp
#include "SignalMast.h"

SignalMast::SignalMast()
  : jMRIId(""),
    pwmBoard(-1),
    prPin(-1),
    caPin(-1),
    cpPin(-1),
    daPin(-1),
    offVal(0),
    litVal(4096) {
  for (int i = 0; i < 6; i++) {
    axPins[i] = -1;
  }
}

SignalMast::SignalMast(String id,
                       int board,
                       int pr, int ca, int cp, int da,
                       int ov, int lv,
                       int ax1, int ax2, int ax3,
                       int ax4, int ax5, int ax6)
  : jMRIId(id),
    pwmBoard(board),
    prPin(pr),
    caPin(ca),
    cpPin(cp),
    daPin(da),
    offVal(ov),
    litVal(lv) {
  axPins[0] = ax1;
  axPins[1] = ax2;
  axPins[2] = ax3;
  axPins[3] = ax4;
  axPins[4] = ax5;
  axPins[5] = ax6;
}
