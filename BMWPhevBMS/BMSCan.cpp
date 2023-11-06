#include <Arduino.h>
#include "BMSCan.h"
#include <ACAN_ESP32.h>
#include <ACAN2515.h>


bool started[] = {false, false};
     
CANMessage BMSCan::convert(const BMS_CAN_MESSAGE &msg) {
  CANMessage ret;

  ret.id = msg.id;
  ret.len = msg.len;
  ret.ext = msg.flags.extended;

  for(int i = 0; i < msg.len; i++) {
    ret.data[i] = msg.buf[i];
  }
  return ret;
}

BMS_CAN_MESSAGE BMSCan::convert(const CANMessage &msg) {
  BMS_CAN_MESSAGE ret;

  ret.id = msg.id;
  ret.len = msg.len;
  ret.flags.extended = msg.ext;
  for(int i = 0; i < msg.len; i++) {
    ret.buf[i] = msg.data[i];
  }
  return ret;
}

int BMSCan::read (BMS_CAN_MESSAGE &msg, int interfaceIndex) {
  CANMessage readMesg;
  int response;
  if (interfaceIndex == 0) {
    response = ACAN_ESP32::can.receive(readMesg);
  } else if (interfaceIndex == 1) {
   response = can1->receive(readMesg);
  }
  msg = convert(readMesg);
  return response;
}

uint32_t BMSCan::available (int interfaceIndex) {

  if (interfaceIndex == 0 && started[interfaceIndex]) {
    return ACAN_ESP32::can.available();
  } else if (interfaceIndex == 1 && started[interfaceIndex]) {
    return can1->available();
  }
  return 0;
}


void BMSCan::begin(uint32_t baud, int interfaceIndex) {

  if (interfaceIndex == 0 && !started[interfaceIndex]) {
    ACAN_ESP32_Settings canSettings(CAN_BAUD);
    canSettings.mRxPin = GPIO_NUM_16;
    canSettings.mTxPin = GPIO_NUM_17;
    uint16_t errorCode = ACAN_ESP32::can.begin(canSettings);
    if (errorCode > 0) {
      Serial.print ("Can0 Configuration error 0x") ;
      Serial.println (errorCode, HEX) ;
    }
    started[interfaceIndex] = true;
  } else if (interfaceIndex == 1 && !started[interfaceIndex]) {

    started[interfaceIndex] = true;
  }

}

int BMSCan::write(const BMS_CAN_MESSAGE &msg, int interfaceIndex) {
  CANMessage toSend = convert(msg);

  if (interfaceIndex == 0) {
    ACAN_ESP32::can.tryToSend(toSend);
  } else if (interfaceIndex == 1) {
    can1->tryToSend(toSend);
  }
  return 0;
}
