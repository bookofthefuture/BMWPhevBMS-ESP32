#include <Arduino.h>
#include "BMSCan.h"
#include <ACAN.h>
#include <ACAN2515.h>
#include "SerialConsole.h"

ACAN2515* can2;
ACAN2515* can3;
bool started[] = {false, false, false, false};
     
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
    response = ACAN::can0.receive(readMesg);
  } else if (interfaceIndex == 1) {
    #ifdef __MK66FX1M0__
    response = ACAN::can1.receive(readMesg);
    #endif
  } else if (interfaceIndex == 2) {
    response = can2->receive(readMesg);
  } else if (interfaceIndex == 3) {
    #ifdef __MK66FX1M0__
    response = can3->receive(readMesg);
    #endif
  }
  msg = convert(readMesg);
  return response;
}

uint32_t BMSCan::available (int interfaceIndex) {

  if (interfaceIndex == 0 && started[interfaceIndex]) {
    return ACAN::can0.available();
  } else if (interfaceIndex == 1 && started[interfaceIndex]) {
    #ifdef __MK66FX1M0__
    return ACAN::can1.available();
    #endif
  } else if (interfaceIndex == 2 && started[interfaceIndex]) {
    return can2->available();
  } else if (interfaceIndex == 3 && started[interfaceIndex]) {
    #ifdef __MK66FX1M0__
    return can3->available();
    #endif
  }
  return 0;
}
void BMSCan::begin(uint32_t baud, int interfaceIndex) {
   
  if (interfaceIndex == 0 && !started[interfaceIndex]) {
    ACANSettings settings(baud);
    ACAN::can0.begin(settings);
    started[interfaceIndex] = true;
  } else if (interfaceIndex == 1 && !started[interfaceIndex]) {
    #ifdef __MK66FX1M0__
    ACANSettings settings(baud);
    ACAN::can1.begin(settings);
    #endif
    started[interfaceIndex] = true;
  } else if (interfaceIndex == 2 && !started[interfaceIndex]) {
   can2 = new ACAN2515 (MCP2515_CS, SPI, MCP2515_INT) ;
   ACAN2515Settings settings(16 * 1000 * 1000, baud);
   can2->begin(settings, [] { can2->isr () ; });
   started[interfaceIndex] = true;
  } else if (interfaceIndex == 3 && !started[interfaceIndex]) {
   #ifdef __MK66FX1M0__
   can3 = new ACAN2515 (MCP2515_CS_2, SPI1, MCP2515_INT_2) ;
   ACAN2515Settings settings(16 * 1000 * 1000, baud);
   can3->begin(settings, [] { can3->isr () ; });
   started[interfaceIndex] = true;
   #endif
  }

}

int BMSCan::write(const BMS_CAN_MESSAGE &msg, int interfaceIndex) {
  CANMessage toSend = convert(msg);

  if (interfaceIndex == 0) {
    ACAN::can0.tryToSend(toSend);
  } else if (interfaceIndex == 1) {
    #ifdef __MK66FX1M0__
    ACAN::can1.tryToSend(toSend); 
    #endif
  } else if (interfaceIndex == 2 && can2 != NULL) {
    can2->tryToSend(toSend);
  } else if (interfaceIndex == 3 && can3 != NULL) {
    #ifdef __MK66FX1M0__
    can3->tryToSend(toSend);
    #endif
  }
  return 0;
}
