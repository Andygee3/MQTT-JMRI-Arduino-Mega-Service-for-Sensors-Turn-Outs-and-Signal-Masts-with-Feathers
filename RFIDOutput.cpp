#include "RFIDOutput.h"
#include <Arduino.h>

#define numberOfManufacturers 4
#define millisTimeoutBetweenSends 20000

const String rfidTagManufacturers[numberOfManufacturers] = {
  "0800", "6B00", "4300", "0500"
};

RFIDOutput::RFIDOutput(Stream &SerialPort, String SensorName)
  : ser(SerialPort)
  , Name(SensorName)
{
  ValidTagRead = false;
  millisAtLastSend = millis();
  lastId = "";
}

bool RFIDOutput::CheckForTag() {
  String thisId = "";
  ValidTagRead = false;
  bool success = false;
  if (ser.available())
  {
    while (ser.available())
    {
      char val = ser.read();
      //Serial.print(val);
      if (val == 0x02) {
        //Serial.println("Start");
        _bytesRead = 0;
        checksum = 0;
        ValidTagRead = false;
      }
      else if (val == 0x03 && _bytesRead == 12) {
        // ID completely read
        thisId = Tag;
        //Serial.println("ThisID "+thisId);
        mfr = hex2dec(thisId.substring(0, 4));
        id  = hex2dec(thisId.substring(4, 10));
        chk = hex2dec(thisId.substring(10, 12));

        // Do checksum calculation
        int i2;
        for (int i = 0; i < 5; i++) {
          i2 = 2 * i;
          checksum ^= hex2dec(thisId.substring(i2, i2 + 2));
        }
        //Serial.println("checksum " + String(checksum) + " chk " + String(chk));
        if (checksum == chk) {
          ValidTagRead = true;
        }
        else {
          ValidTagRead = false;
          Serial.println("Failed xs " + thisId);
        }
        bool manufacturerRecognised = false;
        for (int m = 0; m < numberOfManufacturers; m++) {
          if (rfidTagManufacturers[m] == thisId.substring(0, 4))
            manufacturerRecognised = true;
        }
        if (!manufacturerRecognised) {
          Serial.println("Mf not recognised " + thisId.substring(0, 4));
          ValidTagRead = false;
        }
      }
      else if (_bytesRead <= 12) {
        Tag[_bytesRead++] = val;
      }

      if (_bytesRead > 12) {
        Serial.println("BR " + String(_bytesRead));
        return false;
      }
      if (val == 0x03) {
        //Serial.println("End char " + String(_bytesRead));
      }


    }
  }

  unsigned long millisSinceLastSend = millis() - millisAtLastSend;
  if (ValidTagRead)
  {
    Serial.println("Valid tag");
    if (thisId != lastId)
    {
      Serial.println("Different IDs - "+thisId+" - "+lastId);
      success = true;
      millisAtLastSend = millis();
      lastId = thisId;
    }
    else
    {
      Serial.println("Same IDs - "+thisId+" - "+lastId);
      if (millisSinceLastSend > millisTimeoutBetweenSends) {
        Serial.println("Timeout OK "+String(millisSinceLastSend));
        success = true;
        millisAtLastSend = millis();
        lastId = thisId;
      }
      else {
        Serial.println("Timeout not hit "+String(millisSinceLastSend));
      }
    }  
  } 

  return success;
}

long RFIDOutput::hex2dec(String hexCode) {
  char buf[19] = "";
  hexCode = "0x" + hexCode;
  hexCode.toCharArray(buf, 18);
  //Serial.print("Decoding ");
  //Serial.print(hexCode);
  //Serial.print(": ");
  //Serial.println(strtol(buf, NULL, 0));
  return strtol(buf, NULL, 0);
}
