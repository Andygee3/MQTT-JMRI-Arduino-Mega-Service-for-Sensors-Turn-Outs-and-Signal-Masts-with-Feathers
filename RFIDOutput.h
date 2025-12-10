// RFIDInput.h
#ifndef RFIDOutput_h
#define RFIDOutput_h

#include <Arduino.h>

class RFIDOutput {
  private:
    byte checksum;
    int _bytesRead;
    Stream &ser;
    int mfr;         // Manufacturer (?) Code (2 bytes), only useful in UART Mode
    long id;         // Tag ID (3 bytes)
    byte chk;        // Checksum (1 byte), only useful in UART Mode
    boolean valid;   // Validity of the Tag, based on the Checksum (UART Mode) or the parity bits (Wiegand Mode)
    char lastTag[14];
    String lastId;
    unsigned long millisAtLastSend;

    long hex2dec(String hexCode);

  public:
    bool ValidTagRead;
    String Name;
    char Tag[14];
   
    RFIDOutput(Stream &SerialPort, String SensorName);

    bool CheckForTag();

};

#endif
