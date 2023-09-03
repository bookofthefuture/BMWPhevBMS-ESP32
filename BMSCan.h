#pragma once
#include <ACAN.h>
#include <ACAN2515.h>
#include <SPI.h>
#define DEFAULT_CAN_INTERFACE_INDEX 0



static const byte MCP2515_SCK = 14 ; // SCK input of MCP2515
static const byte MCP2515_SI  = 7 ; // SI input of MCP2515
static const byte MCP2515_SO  = 8 ; // SO output of MCP2515
static const byte MCP2515_CS  = 15 ; // CS input of MCP2515
static const byte MCP2515_INT = 2 ; // INT output of MCP2515
static const byte MCP2515_CS_2  = 20 ; // CS input of MCP2515
static const byte MCP2515_INT_2 = 16 ; // INT output of MCP2515


typedef struct BMS_CAN_MESSAGE {
    uint32_t id;
    uint16_t timestamp;
    struct {
        uint8_t extended:1; // identifier is extended (29-bit)
        uint8_t remote:1;   // remote transmission request packet type
        uint8_t overrun:1;  // message overrun
        uint8_t reserved:5;
    } flags;
    uint8_t len;          // length of data
    uint8_t buf[8];
} BMS_CAN_MESSAGE;


class BMSCan
{
  public:
     int write(const BMS_CAN_MESSAGE &msg, int interfaceIndex);
     void begin(uint32_t baud, int interfaceIndex);
     uint32_t available (int interfaceIndex);
     int read (BMS_CAN_MESSAGE &msg, int interfaceIndex);
     ACAN2515* can2;
  private:
     CANMessage convert(const BMS_CAN_MESSAGE &msg);
     BMS_CAN_MESSAGE convert(const CANMessage &msg);
     
};
