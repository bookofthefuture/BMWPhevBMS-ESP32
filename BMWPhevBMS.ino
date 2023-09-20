/*
  Copyright (c) 2019 Simp ECO Engineering
  Permission is hereby granted, free of charge, to any person obtaining
  a copy of this software and associated documentation files (the
  "Software"), to deal in the Software without restriction, including
  without limitation the rights to use, copy, modify, merge, publish,
  distribute, sublicense, and/or sell copies of the Software, and to
  permit persons to whom the Software is furnished to do so, subject to
  the following conditions:
  The above copyright notice and this permission notice shall be included
  in all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
  CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
  TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

  Thank you to James Warner for proving out canbus decoding and finding balancing bits
/////////////////////////////////////////////////////////////////////////////////////////////////

░██████╗██╗███╗░░░███╗██████╗░██████╗░███╗░░░███╗░██████╗  
██╔════╝██║████╗░████║██╔══██╗██╔══██╗████╗░████║██╔════╝  
╚█████╗░██║██╔████╔██║██████╔╝██████╦╝██╔████╔██║╚█████╗░  
░╚═══██╗██║██║╚██╔╝██║██╔═══╝░██╔══██╗██║╚██╔╝██║░╚═══██╗  
██████╔╝██║██║░╚═╝░██║██║░░░░░██████╦╝██║░╚═╝░██║██████╔╝  
╚═════╝░╚═╝╚═╝░░░░░╚═╝╚═╝░░░░░╚═════╝░╚═╝░░░░░╚═╝╚═════╝░  

░██████╗██████╗░░█████╗░░█████╗░███████╗  ██████╗░░█████╗░██╗░░░░░██╗░░░░░░██████╗
██╔════╝██╔══██╗██╔══██╗██╔══██╗██╔════╝  ██╔══██╗██╔══██╗██║░░░░░██║░░░░░██╔════╝
╚█████╗░██████╔╝███████║██║░░╚═╝█████╗░░  ██████╦╝███████║██║░░░░░██║░░░░░╚█████╗░
░╚═══██╗██╔═══╝░██╔══██║██║░░██╗██╔══╝░░  ██╔══██╗██╔══██║██║░░░░░██║░░░░░░╚═══██╗
██████╔╝██║░░░░░██║░░██║╚█████╔╝███████╗  ██████╦╝██║░░██║███████╗███████╗██████╔╝
╚═════╝░╚═╝░░░░░╚═╝░░╚═╝░╚════╝░╚══════╝  ╚═════╝░╚═╝░░╚═╝╚══════╝╚══════╝╚═════╝░

███████╗██████╗░██╗████████╗██╗░█████╗░███╗░░██╗
██╔════╝██╔══██╗██║╚══██╔══╝██║██╔══██╗████╗░██║
█████╗░░██║░░██║██║░░░██║░░░██║██║░░██║██╔██╗██║
██╔══╝░░██║░░██║██║░░░██║░░░██║██║░░██║██║╚████║
███████╗██████╔╝██║░░░██║░░░██║╚█████╔╝██║░╚███║
╚══════╝╚═════╝░╚═╝░░░╚═╝░░░╚═╝░╚════╝░╚═╝░░╚══╝


This version of SimpBMS has been modified as the Space Balls edition utilising the Teensey 3.6, alowingfor upto 4 Canbus' 
  2 Native(Flexcans) + 2 MCP2515/SPI Cans									 
*/

#include "BMSModuleManager.h"
#include <Arduino.h>
#include "CONFIG.h"
#include "SerialConsole.h"
#include "Logger.h"
#include <EEPROM.h>
#include <SPI.h>
#include "BMSUtil.h"
#include "CRC8.h"
#include "BMSCan.h"

#include "BMSWebServer.h"
#include <esp_task_wdt.h>
#include <SPIFFS.h>

#define CPU_REBOOT (ESP.restart());
#define WDT_TIMEOUT 3
#define HOSTNAME "ESP32-BMS"



BMSModuleManager bms;
SerialConsole console;
EEPROMSettings settings;
BMSWebServer bmsWebServer(settings, bms);

/////Version Identifier/////////
int firmver = 220225;

//Simple BMS V2 wiring//

const int AC_PRESENT = 34; // input 1 - high active //AC Present
const int INVERTER_START = 32;// output 1 - high active //repurpose for fan
const int led = 2;
const int BMBfault = 11;

byte bmsstatus = 0;
//bms status values
#define Boot 0
#define Ready 1
#define Drive 2
#define Charge 3
#define Precharge 4
#define RapidCharge 5
#define Error 6
//

//Current sensor values
#define Undefined 0
#define Canbus 1			 
//

// Can current sensor values
#define IsaScale 1
#define BMWSBox 2
#define CurCanMax 2 // max value
//
//Charger Types
//Charger Types
#define NoCharger 0
#define Outlander 1

//
int outlander_charger_reported_voltage = 0;
int outlander_charger_reported_current = 0;
int outlander_charger_reported_temp1 = 0;
int outlander_charger_reported_temp2 = 0;
byte outlander_charger_reported_status = 0;
byte evse_duty = 0;
bool secondPackFound = false;

//CSC Variants
#define BmwI3 0
#define MiniE 1
//


int Discharge;
int ErrorReason = 0;

//variables for VE can
uint16_t chargevoltage = 49100; //max charge voltage in mv
int chargecurrent;
uint16_t disvoltage = 42000; // max discharge voltage in mv
int discurrent;
uint16_t SOH = 100; // SOH place holder

unsigned char bmsAlarm[4] = {0, 0, 0, 0};
unsigned char bmsWarning[4] = {0, 0, 0, 0};
unsigned char mes[8] = {0, 0, 0, 0, 0, 0, 0, 0};

long unsigned int rxId;
unsigned char len = 0;
byte rxBuf[8];
char msgString[128];                        // Array to store serial string
uint32_t inbox;
int32_t CANmilliamps;
signed long voltage1, voltage2, voltage3 = 0; //mV only with ISAscale sensor
double amphours, kilowatthours, kilowatts; //only with ISAscale sensor

//variables for current calulation
int value;
float currentact, RawCur;
float ampsecond;
unsigned long lasttime;
unsigned long inverterLastRec;
byte inverterStatus;
bool inverterInDrive = false;
bool rapidCharging = false;
unsigned long looptime, looptime1, looptime2, UnderTime, cleartime, chargertimer, balancetimer = 0; //ms
int currentsense = 1;
int sensor = 1;

//Variables for SOC calc
int SOC = 100; //State of Charge
int SOCset = 0;
int SOCtest = 0;
int SOCoverride = -1;

int ncharger = 1; // number of chargers

//variables
int outputstate = 0;
int incomingByte = 0;
int storagemode = 0;
int x = 0;
int cellspresent = 0;
int dashused = 1;
int Charged = 0;
bool balancepauze = 0;


//Debugging modes//////////////////
int debug = 1;
int inputcheck = 0; //read digital inputs
int outputcheck = 0; //check outputs
int candebug = 0; //view can frames
int gaugedebug = 0;
int debugCur = 0;
int CSVdebug = 0;
int menuload = 0;
int balancecells;
int debugdigits = 2; //amount of digits behind decimal for voltage reading
int balancedebug = 0;
int chargeOverride = 0;

//BMW Can Variables///

//uint8_t check1[8] = {0x13, 0x76, 0xD9, 0xBC, 0x9A, 0xFF, 0x50, 0x35};
//uint8_t check2[8] = {0x4A, 0x2F, 0x80, 0xE5, 0xC3, 0xA6, 0x09, 0x6C};
uint8_t Imod, mescycle = 0;
uint8_t nextmes = 0;
uint16_t commandrate = 50;
uint8_t testcycle = 0;
uint8_t DMC[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t Unassigned, NextID = 0;

//BMW checksum variable///

CRC8 crc8;
uint8_t checksum;
const uint8_t finalxor [12] = {0xCF, 0xF5, 0xBB, 0x81, 0x27, 0x1D, 0x53, 0x69, 0x02, 0x38, 0x76, 0x4C};

void getcurrent();
static void receivedFiltered (const CANMessage & inMsg) {
    if (candebug == 1) {
          Serial.print(millis());
          if ((inMsg.id & 0x80000000) == 0x80000000)    // Determine if ID is standard (11 bits) or extended (29 bits)
            sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (inMsg.id & 0x1FFFFFFF), inMsg.len);
          else
            sprintf(msgString, ",0x%.3lX,false,%1d", inMsg.id, inMsg.len);

          Serial.print(msgString);

          Serial.println(" Filtered Can ");
    }

    if (inMsg.id == 0x389) {
      outlander_charger_reported_voltage = inMsg.data[0] * 2;
      outlander_charger_reported_current = inMsg.data[2];
      outlander_charger_reported_temp1 = inMsg.data[3] - 40;
      outlander_charger_reported_temp2 = inMsg.data[4] - 40;

    } else if (inMsg.id == 0x38A) {
       outlander_charger_reported_status = inMsg.data[4];
       evse_duty = inMsg.data[3];
    } else if (inMsg.id == 0x527) {
        long ampseconds = inMsg.data[2] + (inMsg.data[3] << 8) + (inMsg.data[4] << 16) + (inMsg.data[5] << 24);
        amphours = ampseconds/3600.0f;
    } else if(inMsg.id == 0x521) {
        CANmilliamps = inMsg.data[2] + (inMsg.data[3] << 8) + (inMsg.data[4] << 16) + (inMsg.data[5] << 24);
        RawCur = CANmilliamps; 
        getcurrent();
    } else if(inMsg.id == 0x522) {
        voltage1 = inMsg.data[2] + (inMsg.data[3] << 8) + (inMsg.data[4] << 16) + (inMsg.data[5] << 24);
    } else if(inMsg.id == 0x523) {
         voltage2 = inMsg.data[2] + (inMsg.data[3] << 8) + (inMsg.data[4] << 16) + (inMsg.data[5] << 24);
    } else if(inMsg.id == 0x526) {
        long watt = inMsg.data[2] + (inMsg.data[3] << 8) + (inMsg.data[4] << 16) + (inMsg.data[5] << 24);
        kilowatts = watt/1000.0f;
    } else if(inMsg.id == 0x527) {
        long ampseconds = inMsg.data[2] + (inMsg.data[3] << 8) + (inMsg.data[4] << 16) + (inMsg.data[5] << 24);
        amphours = ampseconds/3600.0f;
    } else if(inMsg.id == 0x528) {
        long wh = inMsg.data[2] + (inMsg.data[3] << 8) + (inMsg.data[4] << 16) + (inMsg.data[5] << 24);
        kilowatthours = wh/1000.0f;
    } else if (inMsg.id == 0x377) {
      inverterLastRec = millis();
      inverterStatus = inMsg.data[7];
    }
    //canio
    else if (inMsg.id == 0x01) {
      inverterInDrive = inMsg.data[1] & 0x80 == 0x80;
    }
    //chademo
    else if (inMsg.id == 0x354 && inMsg.data[0] == 0x01) {
      rapidCharging = true;
    }
    else if (inMsg.id == 0x52A) { //param set value
      if (inMsg.data[0] == 0x01) { //set ac current max
        settings.chargecurrentmax = inMsg.data[1] * 10;
        Serial.print("Setting max current ");
        Serial.println(settings.chargecurrentmax);

      }
    }
  
}

bool chargeEnabled() {
  return digitalRead(AC_PRESENT) == HIGH || chargeOverride == 1;
}

bool inverterControlledContactorsStatus() {
  // if inverter in RUN mode
  if (inverterStatus == 0x22) {
    return true;
  }

  return false;
}


void loadSettings()
{
  Logger::console("Resetting to factory defaults");
  settings.version = EEPROM_VERSION;
  settings.checksum = 2;
  settings.canSpeed = 500000;
  settings.batteryID = 0x01; //in the future should be 0xFF to force it to ask for an address
  settings.OverVSetpoint = 4.2f;
  settings.UnderVSetpoint = 3.0f;
  settings.ChargeVsetpoint = 4.1f;
  settings.ChargeHys = 0.2f; // voltage drop required for charger to kick back on
  settings.WarnOff = 0.1f; //voltage offset to raise a warning
  settings.DischVsetpoint = 3.2f;
  settings.DischHys = 0.2f; // Discharge voltage offset
  settings.CellGap = 0.2f; //max delta between high and low cell
  settings.OverTSetpoint = 65.0f;
  settings.UnderTSetpoint = -10.0f;
  settings.ChargeTSetpoint = 0.0f;
  settings.triptime = 500;//mS of delay before counting over or undervoltage
  settings.DisTSetpoint = 40.0f;
  settings.WarnToff = 5.0f; //temp offset before raising warning
  settings.IgnoreTemp = 0; // 0 - use both sensors, 1 or 2 only use that sensor
  settings.IgnoreVolt = 0.5;//
  settings.balanceVoltage = 3.9f;
  settings.balanceHyst = 0.04f;
  settings.balanceDuty = 60;
  settings.logLevel = 2;
  settings.CAP = 100; //battery size in Ah
  settings.Pstrings = 1; // strings in parallel used to divide voltage of pack
  settings.Scells = 12;//Cells in series
  settings.StoreVsetpoint = 3.8; // V storage mode charge max
  settings.discurrentmax = 300; // max discharge current in 0.1A
  settings.DisTaper = 0.3f; //V offset to bring in discharge taper to Zero Amps at settings.DischVsetpoint
  settings.chargecurrentmax = 300; //max charge current in 0.1A
  settings.rapidchargecurrentmax = 1200;
  settings.chargecurrentend = 50; //end charge current in 0.1A
  settings.socvolt[0] = 3100; //Voltage and SOC curve for voltage based SOC calc
  settings.socvolt[1] = 10; //Voltage and SOC curve for voltage based SOC calc
  settings.socvolt[2] = 4100; //Voltage and SOC curve for voltage based SOC calc
  settings.socvolt[3] = 90; //Voltage and SOC curve for voltage based SOC calc
  settings.invertcur = 0; //Invert current sensor direction
  settings.cursens = 2;
  settings.chargerCanIndex = DEFAULT_CAN_INTERFACE_INDEX; //default to can0
  settings.veCanIndex = DEFAULT_CAN_INTERFACE_INDEX; //default to can0
  settings.secondBatteryCanIndex = DEFAULT_CAN_INTERFACE_INDEX; //default to can0, effectivly no second pack
  settings.curcan = IsaScale;																   
  settings.voltsoc = 0; //SOC purely voltage based
  settings.Pretime = 5000; //ms of precharge time
  settings.conthold = 50; //holding duty cycle for contactor 0-255
  settings.Precurrent = 1000; //ma before closing main contator
  settings.convhigh = 58; // mV/A current sensor high range channel
  settings.convlow = 643; // mV/A current sensor low range channel
  settings.changecur = 20000;//mA change overpoint
  settings.offset1 = 1750; //mV mid point of channel 1
  settings.offset2 = 1750;//mV mid point of channel 2
  settings.gaugelow = 50; //empty fuel gauge pwm
  settings.gaugehigh = 255; //full fuel gauge pwm
  settings.ESSmode = 0; //activate ESS mode
  settings.ncur = 1; //number of multiples to use for current measurement
  settings.chargertype = 2; // 1 - Brusa NLG5xx 2 - Volt charger 0 -No Charger
  settings.chargerspd = 100; //ms per message
  settings.UnderDur = 5000; //ms of allowed undervoltage before throwing open stopping discharge.
  settings.CurDead = 5;// mV of dead band on current sensor
  settings.ChargerDirect = 1; //1 - charger is always connected to HV battery // 0 - Charger is behind the contactors
  settings.TempOff = 0.5; //V of allowable difference between measurements
  settings.tripcont = 1; //in ESSmode 1 - Main contactor function, 0 - Trip function
  settings.triptime = 500;//mS of delay before counting over or undervoltage
}


BMSCan bmscan;
BMS_CAN_MESSAGE msg;
BMS_CAN_MESSAGE inMsg;

uint32_t lastUpdate;


void setup()
{
  
  Serial.begin(115200);
  Serial.println("Starting up!");
  Serial.println("SimpBMS V2 BMW");
  delay(4000);  //just for easy debugging. It takes a few seconds for USB to come up properly on most OS's

  pinMode(AC_PRESENT, INPUT);
  pinMode(INVERTER_START, OUTPUT); // fan relay
  digitalWrite(INVERTER_START, LOW);
  pinMode(led, OUTPUT);


  // enable WDT
  noInterrupts();                                         // don't allow interrupts while setting up WDOG
  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch
  interrupts();
  /////////////////

  EEPROM.begin(sizeof(settings));
  EEPROM.get(0, settings);
  if (settings.version != EEPROM_VERSION)
  {
    Serial.print("Version");
    Serial.println(settings.version);
    Serial.println();
    loadSettings();
  }

  bmscan.begin(500000, DEFAULT_CAN_INTERFACE_INDEX);

  SPI.begin(MCP2515_SCK, MCP2515_MISO, MCP2515_MOSI, MCP2515_CS) ;
  bmscan.can1 = new ACAN2515 (MCP2515_CS, SPI, MCP2515_INT) ;
  ACAN2515Settings cansettings(16 * 1000 * 1000, 500000);
  const ACAN2515Mask rxm0 = standard2515Mask(0x7FF, 0, 0) ; // For filter #0 and #1
  const ACAN2515Mask rxm1 = standard2515Mask(0x7F0, 0, 0) ; // For filter #2 to #
  const ACAN2515AcceptanceFilter filters [] = {
    {standard2515Filter(0x377, 0, 0), receivedFiltered},
    {standard2515Filter(0x01, 0, 0), receivedFiltered},
    {standard2515Filter(0x520, 0, 0), receivedFiltered},
    {standard2515Filter(0x380, 0, 0), receivedFiltered},
    {standard2515Filter(0x354, 0, 0), receivedFiltered}
  };

  const uint16_t errorCode = bmscan.can1->begin (cansettings, [] { bmscan.can1->isr () ; }, rxm0, rxm1, filters, 5) ;
  bmscan.begin(500000, settings.chargerCanIndex);
  bmscan.begin(500000, settings.veCanIndex);

  Logger::setLoglevel(Logger::Off); //Debug = 0, Info = 1, Warn = 2, Error = 3, Off = 4

  lastUpdate = 0;

    // Initialize SPIFFS
  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

   //AP and Station Mode
  WiFi.mode(WIFI_AP_STA);

  WiFi.hostname(HOSTNAME);
  // Connect to Wi-Fi
  WiFi.begin();

  Serial.println(WiFi.localIP());

  bmsWebServer.setup();
  crc8.begin();


  digitalWrite(led, HIGH);
  
  bms.setPstrings(settings.Pstrings);
  bms.setSensors(settings.IgnoreTemp, settings.IgnoreVolt, settings.TempOff);

  cleartime = millis();
}

void loop()
{

  canread(DEFAULT_CAN_INTERFACE_INDEX, 0);
  
  bmscan.can1->dispatchReceivedMessage () ;
  
  if (Serial.available() > 0)
  {
    menu();
  }

  switch (bmsstatus)
  {
    case (Boot):
      Discharge = 0;
      bmsstatus = Ready;
      break;

    case (Ready):
      Discharge = 0;
      if (bms.getHighCellVolt() > settings.balanceVoltage && bms.getHighCellVolt() > bms.getLowCellVolt() + settings.balanceHyst)
      {
        balancecells = 1;
      }
      else
      {
        balancecells = 0;
      }
      if (chargeEnabled() && (bms.getHighCellVolt() < (settings.ChargeVsetpoint - settings.ChargeHys))) //detect AC present for charging and check not balancing
      {
        if (inverterControlledContactorsStatus())
        {
          bmsstatus = Charge;
        }
        else
        {
          bmsstatus = Precharge;
        }
      }
      if (inverterInDrive) {
        if (inverterControlledContactorsStatus())
        {
          bmsstatus = Drive;
        }
        else
        {
          bmsstatus = Precharge;
        }
      }
      if (rapidCharging) {
        if (inverterControlledContactorsStatus())
        {
          bmsstatus = RapidCharge;
        }
        else
        {
          bmsstatus = Precharge;
        }
      }

      break;

    case (Precharge):
      Discharge = 0;
      if(digitalRead(AC_PRESENT) == HIGH) {
        digitalWrite(INVERTER_START, HIGH);
      }
      if (!rapidCharging && inverterControlledContactorsStatus() && chargeEnabled()) {
         bmsstatus = Charge;
      }
      if (!rapidCharging &&inverterControlledContactorsStatus() && inverterInDrive) {
         bmsstatus = Drive;
      }
      if (rapidCharging &&inverterControlledContactorsStatus()) {
         bmsstatus = RapidCharge;
      }
      break;


    case (Drive):
      Discharge = 1;
      if (!inverterInDrive)//Key OFF
      {
        bmsstatus = Ready;
      }
      if (chargeEnabled() && (bms.getHighCellVolt() < (settings.ChargeVsetpoint - settings.ChargeHys))) //detect AC present for charging and check not balancing
      {
        bmsstatus = Charge;
      }

      break;

    case (Charge):
      Discharge = 0;
      if (bms.getHighCellVolt() > settings.balanceVoltage)
      {
        balancecells = 1;
      }
      else
      {
        balancecells = 0;
      }
      if (bms.getHighCellVolt() > settings.ChargeVsetpoint)
      {
        if (bms.getAvgCellVolt() > (settings.ChargeVsetpoint - settings.ChargeHys))
        {
          SOCcharged(2);
        }
        else
        {
          resetISACounters();
        }
        bmsstatus = Ready;
      }
      if (rapidCharging) {
        bmsstatus = RapidCharge;
      }
      if (!chargeEnabled() || !inverterControlledContactorsStatus())//detect AC not present for charging or inverter not closed the contactors
      {
        //send a 0 amp request to outlander
        chargecurrent = 0;
        chargercomms();
        bmsstatus = Ready;
      }
      break;
    case (RapidCharge):
      break;
    case (Error):
      Discharge = 0;
      break;
  }

  if (millis() - looptime > 500)
  {
    looptime = millis();
    bms.getAllVoltTemp();

    if (bms.getLowCellVolt() < settings.UnderVSetpoint || bms.getHighCellVolt() < settings.UnderVSetpoint)
      {
        if (UnderTime > millis()) //check is last time not undervoltage is longer thatn UnderDur ago
        {
          bmsstatus = Error;
          ErrorReason = ErrorReason | 0x02;
        }
      }
      else
      {
        UnderTime = millis() + settings.UnderDur;
      ErrorReason = ErrorReason & ~0x02;
    }

    updateSOC();
    currentlimit();
    VEcan();
    sendcommand();
    
    if (cellspresent == 0 && millis() > 3000)
    {
      cellspresent = bms.seriescells();//set amount of connected cells, might need delay
      bms.setSensors(settings.IgnoreTemp, settings.IgnoreVolt, settings.TempOff);
    }
    else
    {
      if (cellspresent != bms.seriescells() || cellspresent != (settings.Scells * settings.Pstrings)) //detect a fault in cells detected
      {
        if (debug != 0)
        {
          SERIALCONSOLE.println("  ");
          SERIALCONSOLE.print("   !!! Series Cells Fault !!!");
          SERIALCONSOLE.println("  ");
        }
        bmsstatus = Error;
        ErrorReason = 3;
      }
    }

    ///stop reading voltages during balancing//
    if ((settings.balanceDuty + 5) > ((balancetimer - millis()) * 0.001))
    {
      bms.setBalIgnore(true);
      /*
        Serial.println();
        Serial.println("Ignore Voltages Balancing Active");
      */
    }
    else
    {
      bms.setBalIgnore(false);
    }

    ///Set Ids to unnasgined//

    if (Unassigned > 0)
    {
      assignID();
    }

    ////
    alarmupdate();
    resetwdog();
  }
  if (millis() - cleartime > 5000)
  {
    //bms.clearmodules(); // Not functional
    if (bms.checkcomms())
    {
      //no missing modules
      /*
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print(" ALL OK NO MODULE MISSING :) ");
        SERIALCONSOLE.println("  ");
      */
      if (  bmsstatus == Error)
      {
        bmsstatus = Boot;
      }
    }
    else
    {
      //missing module
      if (debug != 0)
      {
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("   !!! MODULE MISSING !!!");
        SERIALCONSOLE.println("  ");
      }
      bmsstatus = Error;
      ErrorReason = 4;
    }
    cleartime = millis();
  }
  if (millis() - looptime1 > settings.chargerspd)
  {
    looptime1 = millis();

    if (bmsstatus == Charge)
    {
      chargercomms();
    }
  }
  
  bmsWebServer.execute();
}

void alarmupdate()
{
  bmsAlarm[0] = 0x00;
  if (settings.OverVSetpoint < bms.getHighCellVolt())
  {
    bmsAlarm[0] = 0x04;
  }
  if (bms.getLowCellVolt() < settings.UnderVSetpoint)
  {
    bmsAlarm[0] |= 0x10;
  }
  if (bms.getHighTemperature() > settings.OverTSetpoint)
  {
    bmsAlarm[0] |= 0x40;
  }
  bmsAlarm[1] = 0;
  if (bms.getLowTemperature() < settings.UnderTSetpoint)
  {
    bmsAlarm[1] = 0x01;
  }
  bmsAlarm[3] = 0;
  if ((bms.getHighCellVolt() - bms.getLowCellVolt()) > settings.CellGap)
  {
    bmsAlarm[3] = 0x01;
  }

  ///warnings///
  bmsWarning[0] = 0;

  if (bms.getHighCellVolt() > (settings.OverVSetpoint - settings.WarnOff))
  {
    bmsWarning[0] = 0x04;
  }
  if (bms.getLowCellVolt() < (settings.UnderVSetpoint + settings.WarnOff))
  {
    bmsWarning[0] |= 0x10;
  }

  if (bms.getHighTemperature() > (settings.OverTSetpoint - settings.WarnToff))
  {
    bmsWarning[0] |= 0x40;
  }
  bmsWarning[1] = 0;
  if (bms.getLowTemperature() < (settings.UnderTSetpoint + settings.WarnToff))
  {
    bmsWarning[1] = 0x01;
  }
}

void printbmsstat()
{
  SERIALCONSOLE.println();
  SERIALCONSOLE.println();
  SERIALCONSOLE.println();
  SERIALCONSOLE.print("BMS Status : ");

  SERIALCONSOLE.print(bmsstatus);
  switch (bmsstatus)
  {
    case (Boot):
      SERIALCONSOLE.print(" Boot ");
      break;

    case (Ready):
      SERIALCONSOLE.print(" Ready ");
      break;

    case (Precharge):
      SERIALCONSOLE.print(" Precharge ");
      break;

    case (Drive):
      SERIALCONSOLE.print(" Drive ");
      break;

    case (Charge):
      SERIALCONSOLE.print(" Charge ");
      break;

    case (RapidCharge):
      SERIALCONSOLE.print(" RapidCharge ");
      break;
      
    case (Error):
      SERIALCONSOLE.print(" Error ");
      if (ErrorReason == 4) {
          SERIALCONSOLE.print(" Module Missing "); 
      }
      break;
  }
  
  SERIALCONSOLE.print("  ");
  if (chargeEnabled())
  {
    SERIALCONSOLE.print("| AC Present |");
  }

  if (balancecells == 1)
  {
    SERIALCONSOLE.print("|Balancing Active");
  }
  SERIALCONSOLE.print("  ");
  SERIALCONSOLE.print(cellspresent);
  SERIALCONSOLE.println();
  SERIALCONSOLE.print("Out:");
  SERIALCONSOLE.print(digitalRead(INVERTER_START));
  //SERIALCONSOLE.print(digitalRead(OUT3));
  //SERIALCONSOLE.print(digitalRead(OUT4));

  SERIALCONSOLE.print(" In:");
  SERIALCONSOLE.print(digitalRead(AC_PRESENT));
  if (bmsstatus == Charge && settings.chargertype == Outlander) {
    SERIALCONSOLE.println();
    SERIALCONSOLE.print("Outlander Charger - Reported Voltage: ");
    SERIALCONSOLE.print(outlander_charger_reported_voltage);
    SERIALCONSOLE.print("V Reported Current: ");
    SERIALCONSOLE.print(outlander_charger_reported_current / 10);
    SERIALCONSOLE.print("A Reported Temp1: ");
    SERIALCONSOLE.print(outlander_charger_reported_temp1);
    SERIALCONSOLE.print("C Reported Temp2: ");
    SERIALCONSOLE.print(outlander_charger_reported_temp2);
    SERIALCONSOLE.print("C status: ");
    if (outlander_charger_reported_status == 0) {
          SERIALCONSOLE.print("Not Charging");
      } else if (outlander_charger_reported_status == 0x04) {
          SERIALCONSOLE.print("Wait for Mains");
      } else if (outlander_charger_reported_status == 0x08) {
          SERIALCONSOLE.print("Ready/Charging");
}
    SERIALCONSOLE.println();
  }
}


void getcurrent()
{

  if (settings.invertcur == 1)
  {
    RawCur = RawCur * -1;
  }

  currentact = RawCur;

  currentact = settings.ncur * currentact;
  RawCur = 0;
}

void updateSOC()
{
  //current shunt based SOC
  SOC = (((settings.CAP) - amphours) / (settings.CAP) ) * 100;
  SOCset = 1;
}

void SOCcharged(int y)
{
  if (y == 1)
  {
    SOC = 95;
    ampsecond = (settings.CAP * settings.Pstrings * 1000) / 0.27777777777778 ; //reset to full, dependant on given capacity. Need to improve with auto correction for capcity.
  }
  if (y == 2)
  {
    SOC = 100;
    ampsecond = (settings.CAP * settings.Pstrings * 1000) / 0.27777777777778 ; //reset to full, dependant on given capacity. Need to improve with auto correction for capcity.
  }
}

void VEcan() //communication with Victron system over CAN
{
  msg.id  = 0x351;
  msg.len = 8;

  msg.buf[0] = lowByte(uint16_t((settings.ChargeVsetpoint * settings.Scells ) * 10));
  msg.buf[1] = highByte(uint16_t((settings.ChargeVsetpoint * settings.Scells ) * 10));
  msg.buf[2] = lowByte(chargecurrent);
  msg.buf[3] = highByte(chargecurrent);
  msg.buf[4] = lowByte(discurrent );
  msg.buf[5] = highByte(discurrent);
  msg.buf[6] = lowByte(uint16_t((settings.DischVsetpoint * settings.Scells) * 10));
  msg.buf[7] = highByte(uint16_t((settings.DischVsetpoint * settings.Scells) * 10));

  bmscan.write(msg, settings.veCanIndex);

  msg.id  = 0x355;
  msg.len = 8;
if (SOCoverride != -1) {
    msg.buf[0] = lowByte(SOCoverride);
    msg.buf[1] = highByte(SOCoverride);
    msg.buf[2] = lowByte(SOH);
    msg.buf[3] = highByte(SOH);
    msg.buf[4] = lowByte(SOCoverride * 10);
    msg.buf[5] = highByte(SOCoverride * 10);
  } else {
  msg.buf[0] = lowByte(SOC);
  msg.buf[1] = highByte(SOC);
  msg.buf[2] = lowByte(SOH);
  msg.buf[3] = highByte(SOH);
  msg.buf[4] = lowByte(SOC * 10);
  msg.buf[5] = highByte(SOC * 10);
}

  //Send Charge if in Precharge for VCU
  if (bmsstatus == Precharge) {
     msg.buf[6] = lowByte(Charge);
     msg.buf[7] = highByte(Charge);
  } else {
  msg.buf[6] = lowByte(bmsstatus);
  msg.buf[7] = highByte(bmsstatus);
}

  bmscan.write(msg, settings.veCanIndex);

  msg.id  = 0x356;
  msg.len = 8;
  msg.buf[0] = lowByte(uint16_t(bms.getPackVoltage() * 100));
  msg.buf[1] = highByte(uint16_t(bms.getPackVoltage() * 100));
  msg.buf[2] = lowByte(long(currentact / 100));
  msg.buf[3] = highByte(long(currentact / 100));
  msg.buf[4] = lowByte(int16_t(bms.getAvgTemperature() * 10));
  msg.buf[5] = highByte(int16_t(bms.getAvgTemperature() * 10));
  msg.buf[6] = lowByte(uint16_t(bms.getAvgCellVolt() * 1000));
  msg.buf[7] = highByte(uint16_t(bms.getAvgCellVolt() * 1000));
  bmscan.write(msg, settings.veCanIndex);

  delay(2);
  msg.id  = 0x35A;
  msg.len = 8;
  msg.buf[0] = bmsAlarm[0];//High temp  Low Voltage | High Voltage
  msg.buf[1] = bmsAlarm[1]; // High Discharge Current | Low Temperature
  msg.buf[2] = bmsAlarm[2]; //Internal Failure | High Charge current
  msg.buf[3] = bmsAlarm[3];// Cell Imbalance
  msg.buf[4] = bmsWarning[0];//High temp  Low Voltage | High Voltage
  msg.buf[5] = bmsWarning[1];// High Discharge Current | Low Temperature
  msg.buf[6] = bmsWarning[2];//Internal Failure | High Charge current
  msg.buf[7] = bmsWarning[3];// Cell Imbalance
  bmscan.write(msg, settings.veCanIndex);

  delay(2);
  msg.id  = 0x373;
  msg.len = 8;
  msg.buf[0] = lowByte(uint16_t(bms.getLowCellVolt() * 1000));
  msg.buf[1] = highByte(uint16_t(bms.getLowCellVolt() * 1000));
  msg.buf[2] = lowByte(uint16_t(bms.getHighCellVolt() * 1000));
  msg.buf[3] = highByte(uint16_t(bms.getHighCellVolt() * 1000));
  msg.buf[4] = lowByte(uint16_t(bms.getLowTemperature() + 273.15));
  msg.buf[5] = highByte(uint16_t(bms.getLowTemperature() + 273.15));
  msg.buf[6] = lowByte(uint16_t(bms.getHighTemperature() + 273.15));
  msg.buf[7] = highByte(uint16_t(bms.getHighTemperature() + 273.15));
  bmscan.write(msg, settings.veCanIndex);

  delay(2);
  msg.id  = 0x379; //Installed capacity
  msg.len = 4;
  msg.buf[0] = lowByte(settings.CAP);
  msg.buf[1] = highByte(settings.CAP);
  msg.buf[2] = lowByte(uint16_t(amphours));
  msg.buf[3] = highByte(uint16_t(amphours));
  // (remaining amp hours * 1.2) * 10
  uint16_t rangeEst = (settings.CAP - amphours) * 12;
  msg.buf[4] = lowByte(rangeEst);
  msg.buf[5] = highByte(rangeEst);
  bmscan.write(msg, settings.veCanIndex);

  /*
      delay(2);
    msg.id  = 0x378; //Installed capacity
    msg.len = 2;
    //energy in 100wh/unit
    msg.buf[0] =
    msg.buf[1] =
    msg.buf[2] =
    msg.buf[3] =
    //energy out 100wh/unit
    msg.buf[4] =
    msg.buf[5] =
    msg.buf[6] =
    msg.buf[7] =
  */
  delay(2);
  msg.id  = 0x372;
  msg.len = 8;
  msg.buf[0] = lowByte(bms.getNumModules());
  msg.buf[1] = highByte(bms.getNumModules());
  msg.buf[2] = 0x00;
  msg.buf[3] = 0x00;
  msg.buf[4] = 0x00;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  bmscan.write(msg, settings.veCanIndex);

}

// Settings menu
// Settings menu
void menu()
{

  incomingByte = Serial.read(); // read the incoming byte:
  if (menuload == 4)
  {
    switch (incomingByte)
    {

      case '1':
        menuload = 1;
        candebug = !candebug;
        incomingByte = 'd';
        break;

      case '6':
        menuload = 1;
        cellspresent = bms.seriescells();
        incomingByte = 'd';
        break;

      case '8':
        menuload = 1;
        CSVdebug = !CSVdebug;
        incomingByte = 'd';
        break;

      case '9':
        menuload = 1;
        if (Serial.available() > 0)
        {
          debugdigits = Serial.parseInt();
        }
        if (debugdigits > 4)
        {
          debugdigits = 2;
        }
        incomingByte = 'd';
        break;

      case 113: //q for quite menu

        menuload = 0;
        incomingByte = 115;
        break;

      default:
        // if nothing else matches, do the default
        // default is optional
        break;
    }
  }

  if (menuload == 2)
  {
    switch (incomingByte)
    {

      case '1':
        menuload = 1;
        settings.invertcur = !settings.invertcur;
        incomingByte = 'c';
        break;

      case '3':
        menuload = 1;
        if (Serial.available() > 0)
        {
          settings.ncur = Serial.parseInt();
        }
        menuload = 1;
        incomingByte = 'c';
        break;

      case '4':
        menuload = 1;
        if (Serial.available() > 0)
        {
          settings.convlow = Serial.parseInt();
        }
        incomingByte = 'c';
        break;

      case '5':
        menuload = 1;
        if (Serial.available() > 0)
        {
          settings.convhigh = Serial.parseInt();
        }
        incomingByte = 'c';
        break;

      case '6':
        menuload = 1;
        if (Serial.available() > 0)
        {
          settings.CurDead = Serial.parseInt();
        }
        incomingByte = 'c';
        break;

      case '8':
        menuload = 1;
        if (Serial.available() > 0)
        {
          settings.changecur = Serial.parseInt();
        }
        menuload = 1;
        incomingByte = 'c';
        break;

      case 113: //q for quite menu

        menuload = 0;
        incomingByte = 115;
        break;

      case 115: //s for switch sensor
        settings.cursens ++;
        if (settings.cursens > 1)
        {
          settings.cursens = 0;
        }

        menuload = 1;
        incomingByte = 'c';
        break;

      case '7': //s for switch sensor
        settings.curcan++;
        if (settings.curcan > CurCanMax) {
          settings.curcan = 1;
        }
        menuload = 1;
        incomingByte = 'c';
        break;

      default:
        // if nothing else matches, do the default
        // default is optional
        break;
    }
  }

  if (menuload == 8)
  {
    switch (incomingByte)
    {
      case '1': //e dispaly settings
        if (Serial.available() > 0)
        {
          settings.IgnoreTemp = Serial.parseInt();
        }
        if (settings.IgnoreTemp > 3)
        {
          settings.IgnoreTemp = 0;
        }
        bms.setSensors(settings.IgnoreTemp, settings.IgnoreVolt, settings.TempOff);
        menuload = 1;
        incomingByte = 'i';
        break;

      case '2':
        if (Serial.available() > 0)
        {
          settings.IgnoreVolt = Serial.parseInt();
          settings.IgnoreVolt = settings.IgnoreVolt * 0.001;
          bms.setSensors(settings.IgnoreTemp, settings.IgnoreVolt, settings.TempOff);
          // Serial.println(settings.IgnoreVolt);
          menuload = 1;
          incomingByte = 'i';
        }
        break;

      case 113: //q to go back to main menu

        menuload = 0;
        incomingByte = 115;
        break;
    }
  }



  if (menuload == 7)
  {
    switch (incomingByte)
    {
      case '1':
        if (Serial.available() > 0)
        {
          settings.WarnOff = Serial.parseInt();
          settings.WarnOff = settings.WarnOff * 0.001;
          menuload = 1;
          incomingByte = 'a';
        }
        break;

      case '2':
        if (Serial.available() > 0)
        {
          settings.CellGap = Serial.parseInt();
          settings.CellGap = settings.CellGap * 0.001;
          menuload = 1;
          incomingByte = 'a';
        }
        break;

      case '3':
        if (Serial.available() > 0)
        {
          settings.WarnToff = Serial.parseInt();
          menuload = 1;
          incomingByte = 'a';
        }
        break;

      case '4':
        if (Serial.available() > 0)
        {
          settings.triptime = Serial.parseInt();
          menuload = 1;
          incomingByte = 'a';
        }
        break;

      case 113: //q to go back to main menu
        menuload = 0;
        incomingByte = 115;
        break;
    }
  }

  if (menuload == 6) //Charging settings
  {
    switch (incomingByte)
    {

      case 113: //q to go back to main menu

        menuload = 0;
        incomingByte = 115;
        break;

      case '1':
        if (Serial.available() > 0)
        {
          settings.ChargeVsetpoint = Serial.parseInt();
          settings.ChargeVsetpoint = settings.ChargeVsetpoint / 1000;
          menuload = 1;
          incomingByte = 'e';
        }
        break;


      case '2':
        if (Serial.available() > 0)
        {
          settings.ChargeHys = Serial.parseInt();
          settings.ChargeHys = settings.ChargeHys / 1000;
          menuload = 1;
          incomingByte = 'e';
        }
        break;


      case '4':
        if (Serial.available() > 0)
        {
          settings.chargecurrentend = Serial.parseInt() * 10;
          menuload = 1;
          incomingByte = 'e';
        }
        break;


      case '3':
        if (Serial.available() > 0)
        {
          settings.chargecurrentmax = Serial.parseInt() * 10;
          menuload = 1;
          incomingByte = 'e';
        }
        break;

      case '5': //1 Over Voltage Setpoint
        settings.chargertype = settings.chargertype + 1;
        if (settings.chargertype > 1)
        {
          settings.chargertype = 0;
        }
        menuload = 1;
        incomingByte = 'e';
        break;

      case '6':
        if (Serial.available() > 0)
        {
          settings.chargerspd = Serial.parseInt();
          menuload = 1;
          incomingByte = 'e';
        }
        break;

      case '7':
        if ( settings.ChargerDirect == 1)
        {
          settings.ChargerDirect = 0;
        }
        else
        {
          settings.ChargerDirect = 1;
        }
        menuload = 1;
        incomingByte = 'e';
        break;

      case '9':
        if (Serial.available() > 0)
        {
          settings.ChargeTSetpoint = Serial.parseInt();
          if (settings.ChargeTSetpoint < settings.UnderTSetpoint)
          {
            settings.ChargeTSetpoint = settings.UnderTSetpoint;
          }
          menuload = 1;
          incomingByte = 'e';
        }
        break;
       case 'c':
        if (Serial.available() > 0)
        {
          settings.chargerCanIndex++;
          if (settings.chargerCanIndex > 1) {
            settings.chargerCanIndex = 0;
          } else if (settings.chargerCanIndex < 0) {
            settings.chargerCanIndex = 0;
          }
          bmscan.begin(500000, settings.chargerCanIndex);
          menuload = 1;
          incomingByte = 'e';
        }
        break;
       case 'v':
        if (Serial.available() > 0)
        {
          settings.veCanIndex++;

          if (settings.veCanIndex > 1) {
            settings.veCanIndex = 0;
          }
          bmscan.begin(500000, settings.veCanIndex);
          menuload = 1;
          incomingByte = 'e';
        }
        break;
        case 'o':
        if (Serial.available() > 0)
        {
          if (chargeOverride == 0) {
            chargeOverride = 1;
          } else {
            chargeOverride = 0;
          }
          menuload = 1;
          incomingByte = 'e';
        }
        break;
      case '0':
        if (Serial.available() > 0)
        {
          settings.chargecurrentcold = Serial.parseInt() * 10;
          if (settings.chargecurrentcold > settings.chargecurrentmax)
          {
            settings.chargecurrentcold = settings.chargecurrentmax;
          }
          menuload = 1;
          incomingByte = 'e';
        }
        break;

    }
  }


  if (menuload == 3)
  {
    switch (incomingByte)
    {
      case 113: //q to go back to main menu

        menuload = 0;
        incomingByte = 115;
        break;

      case 'f': //f factory settings
        loadSettings();
        Serial.println("  ");
        Serial.println("  ");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.println(" Coded Settings Loaded ");
        SERIALCONSOLE.println("  ");
        menuload = 1;
        incomingByte = 'b';
        break;

      case 114: //r for reset
        SOCset = 0;
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print(" mAh Reset ");
        SERIALCONSOLE.println("  ");
        menuload = 1;
        incomingByte = 'b';
        break;




      case '1': //1 Over Voltage Setpoint
        if (Serial.available() > 0)
        {
          settings.OverVSetpoint = Serial.parseInt();
          settings.OverVSetpoint = settings.OverVSetpoint / 1000;
          menuload = 1;
          incomingByte = 'b';
        }
        break;

      case 'g':
        if (Serial.available() > 0)
        {
          settings.StoreVsetpoint = Serial.parseInt();
          settings.StoreVsetpoint = settings.StoreVsetpoint / 1000;
          menuload = 1;
          incomingByte = 'b';
        }

      case 'h':
        if (Serial.available() > 0)
        {
          settings.DisTaper = Serial.parseInt();
          settings.DisTaper = settings.DisTaper / 1000;
          menuload = 1;
          incomingByte = 'b';
        }

      case 'b':
        if (Serial.available() > 0)
        {
          settings.socvolt[0] = Serial.parseInt();
          menuload = 1;
          incomingByte = 'b';
        }
        break;


      case 'c':
        if (Serial.available() > 0)
        {
          settings.socvolt[1] = Serial.parseInt();
          menuload = 1;
          incomingByte = 'b';
        }
        break;

      case 'd':
        if (Serial.available() > 0)
        {
          settings.socvolt[2] = Serial.parseInt();
          menuload = 1;
          incomingByte = 'b';
        }
        break;

      case 'e':
        if (Serial.available() > 0)
        {
          settings.socvolt[3] = Serial.parseInt();
          menuload = 1;
          incomingByte = 'b';
        }
        break;

      case 'k': //Discharge Voltage hysteresis
        if (Serial.available() > 0)
        {
          settings.DischHys = Serial.parseInt();
          settings.DischHys  = settings.DischHys  / 1000;
          menuload = 1;
          incomingByte = 'b';
        }
        break;

      case 'j':
        if (Serial.available() > 0)
        {
          settings.DisTSetpoint = Serial.parseInt();
          menuload = 1;
          incomingByte = 'b';
        }
        break;
     case 'l': //secondary battery pack interface
        if (Serial.available() > 0)
        {
          settings.secondBatteryCanIndex++;
          #ifndef __MK66FX1M0__
          //teensy 3.2 doens't have the 2nd interface
          if (settings.secondBatteryCanIndex == 1) {
            settings.secondBatteryCanIndex++;
          }
          #endif
          if (settings.secondBatteryCanIndex > 3) {
            settings.secondBatteryCanIndex = 0;
          }
          bmscan.begin(500000, settings.secondBatteryCanIndex);
          menuload = 1;
          incomingByte = 'b';
        }
        break;

      case '9': //Discharge Voltage Setpoint
        if (Serial.available() > 0)
        {
          settings.DischVsetpoint = Serial.parseInt();
          settings.DischVsetpoint = settings.DischVsetpoint / 1000;
          menuload = 1;
          incomingByte = 'b';
        }
        break;

      case '0': //c Pstrings
        if (Serial.available() > 0)
        {
          settings.Pstrings = Serial.parseInt();
          menuload = 1;
          incomingByte = 'b';
          bms.setPstrings(settings.Pstrings);
        }
        break;

      case 'a': //
        if (Serial.available() > 0)
        {
          settings.Scells  = Serial.parseInt();
          menuload = 1;
          incomingByte = 'b';
        }
        break;

      case '2': //2 Under Voltage Setpoint
        if (Serial.available() > 0)
        {
          settings.UnderVSetpoint = Serial.parseInt();
          settings.UnderVSetpoint =  settings.UnderVSetpoint / 1000;
          menuload = 1;
          incomingByte = 'b';
        }
        break;

      case '3': //3 Over Temperature Setpoint
        if (Serial.available() > 0)
        {
          settings.OverTSetpoint = Serial.parseInt();
          menuload = 1;
          incomingByte = 'b';
        }
        break;

      case '4': //4 Udner Temperature Setpoint
        if (Serial.available() > 0)
        {
          settings.UnderTSetpoint = Serial.parseInt();
          menuload = 1;
          incomingByte = 'b';
        }
        break;

      case '5': //5 Balance Voltage Setpoint
        if (Serial.available() > 0)
        {
          settings.balanceVoltage = Serial.parseInt();
          settings.balanceVoltage = settings.balanceVoltage / 1000;
          menuload = 1;
          incomingByte = 'b';
        }
        break;

      case '6': //6 Balance Voltage Hystersis
        if (Serial.available() > 0)
        {
          settings.balanceHyst = Serial.parseInt();
          settings.balanceHyst =  settings.balanceHyst / 1000;
          menuload = 1;
          incomingByte = 'b';
        }
        break;

      case '7'://7 Battery Capacity inAh
        if (Serial.available() > 0)
        {
          settings.CAP = Serial.parseInt();
          menuload = 1;
          incomingByte = 'b';
        }
        break;

      case '8':// discurrent in A
        if (Serial.available() > 0)
        {
          settings.discurrentmax = Serial.parseInt() * 10;
          menuload = 1;
          incomingByte = 'b';
        }
        break;

    }
  }

  if (menuload == 1)
  {
    switch (incomingByte)
    {
      case 'R'://restart
        CPU_REBOOT ;
        break;

      case 'i': //Ignore Value Settings
        while (Serial.available()) {
          Serial.read();
        }
        Serial.println();
        Serial.println();
        Serial.println();
        Serial.println();
        Serial.println();
        Serial.println("Ignore Value Settings");
        Serial.print("1 - Temp Sensor Setting:");
        Serial.println(settings.IgnoreTemp);
        Serial.print("2 - Voltage Under Which To Ignore Cells:");
        Serial.print(settings.IgnoreVolt * 1000, 0);
        Serial.println("mV");
        Serial.println("q - Go back to menu");
        menuload = 8;
        break;

      case 'e': //Charging settings
        while (Serial.available()) {
          Serial.read();
        }
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println("Charging Settings");
        SERIALCONSOLE.print("1 - Cell Charge Voltage Limit Setpoint: ");
        SERIALCONSOLE.print(settings.ChargeVsetpoint * 1000, 0);
        SERIALCONSOLE.println("mV");
        SERIALCONSOLE.print("2 - Charge Hystersis: ");
        SERIALCONSOLE.print(settings.ChargeHys * 1000, 0 );
        SERIALCONSOLE.println("mV");
        if (settings.chargertype > 0)
        {
          SERIALCONSOLE.print("3 - Pack Max Charge Current: ");
          SERIALCONSOLE.print(settings.chargecurrentmax * 0.1);
          SERIALCONSOLE.println("A");
          SERIALCONSOLE.print("4 - Pack End of Charge Current: ");
          SERIALCONSOLE.print(settings.chargecurrentend * 0.1);
          SERIALCONSOLE.println("A");
        }
        SERIALCONSOLE.print("5- Charger Type: ");
        switch (settings.chargertype)
        {
          case 0:
            SERIALCONSOLE.print("Relay Control");
            break;
          case Outlander:
            SERIALCONSOLE.print("Mitsubish Outlander Charger");
            break;
        }
        SERIALCONSOLE.println();
        if (settings.chargertype > 0)
        {
          SERIALCONSOLE.print("6 - Charger Can Msg Spd: ");
          SERIALCONSOLE.print(settings.chargerspd);
          SERIALCONSOLE.println("mS");
          SERIALCONSOLE.println();
        }
        /*
          SERIALCONSOLE.print("7- Can Speed:");
          SERIALCONSOLE.print(settings.canSpeed/1000);
          SERIALCONSOLE.println("kbps");
        */
        SERIALCONSOLE.print("7 - Charger HV Connection: ");
        switch (settings.ChargerDirect)
        {
          case 0:
            SERIALCONSOLE.print(" Behind Contactors");
            break;
          case 1:
            SERIALCONSOLE.print("Direct To Battery HV");
            break;
        }
        SERIALCONSOLE.println();

        SERIALCONSOLE.print("9 - Charge Current derate Low: ");
        SERIALCONSOLE.print(settings.ChargeTSetpoint);
        SERIALCONSOLE.println(" C");
        SERIALCONSOLE.print("c - Charger Can Interface Index: ");
        switch (settings.chargerCanIndex)
        {
          case 0:
            SERIALCONSOLE.print("Can0");
            break;
          case 1:
            SERIALCONSOLE.print("SPI");
            break;
        }
        SERIALCONSOLE.println();

        SERIALCONSOLE.print("v - Status (VE CAN) Can Interface Index: ");
        switch (settings.veCanIndex)
        {
          case 0:
            SERIALCONSOLE.print("Can0");
            break;
          case 1:
            SERIALCONSOLE.print("SPI");
            break;
        }
        SERIALCONSOLE.println();

        SERIALCONSOLE.print("0 - Pack Cold Charge Current: ");
        SERIALCONSOLE.print(settings.chargecurrentcold * 0.1);
        SERIALCONSOLE.println("A");

        SERIALCONSOLE.print("o - Override AC present: ");
        if (chargeOverride == true) {
          SERIALCONSOLE.print("ON");
        } else {
          SERIALCONSOLE.print("OFF");
        }
        SERIALCONSOLE.println();
        SERIALCONSOLE.println("q - Go back to menu");
        menuload = 6;
        break;
        
        SERIALCONSOLE.println();
        
      case 'a': //Alarm and Warning settings
        while (Serial.available()) {
          Serial.read();
        }
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println("Alarm and Warning Settings Menu");
        SERIALCONSOLE.print("1 - Voltage Warning Offset: ");
        SERIALCONSOLE.print(settings.WarnOff * 1000, 0);
        SERIALCONSOLE.println("mV");
        SERIALCONSOLE.print("2 - Cell Voltage Difference Alarm: ");
        SERIALCONSOLE.print(settings.CellGap * 1000, 0);
        SERIALCONSOLE.println("mV");
        SERIALCONSOLE.print("3 - Temp Warning Offset: ");
        SERIALCONSOLE.print(settings.WarnToff);
        SERIALCONSOLE.println(" C");
        /*
          SERIALCONSOLE.print("4 - Temp Warning Offset: ");
          SERIALCONSOLE.print(settings.UnderDur);
          SERIALCONSOLE.println(" mS");
        */
        SERIALCONSOLE.print("4 - Over and Under Voltage Delay: ");
        SERIALCONSOLE.print(settings.triptime);
        SERIALCONSOLE.println(" mS");

        menuload = 7;
        break;

      case 113: //q to go back to main menu
        EEPROM.put(0, settings); //save all change to eeprom
        EEPROM.commit();
        menuload = 0;
        debug = 1;
        break;
      case 'd': //d for debug settings
        while (Serial.available()) {
          Serial.read();
        }
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println("Debug Settings Menu");
        SERIALCONSOLE.println("Toggle on/off");
        SERIALCONSOLE.print("1 - Can Debug :");
        SERIALCONSOLE.println(candebug);
        SERIALCONSOLE.print("6 - Cells Present Reset :");
        SERIALCONSOLE.println(cellspresent);
        SERIALCONSOLE.print("8 - CSV Output :");
        SERIALCONSOLE.println(CSVdebug);
        SERIALCONSOLE.print("9 - Decimal Places to Show :");
        SERIALCONSOLE.println(debugdigits);
        SERIALCONSOLE.println("q - Go back to menu");
        menuload = 4;
        break;

      case 99: //c for calibrate zero offset
        while (Serial.available()) {
          Serial.read();
        }
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println("Current Sensor Menu");
        SERIALCONSOLE.print("s - Current Sensor Type : ");
        switch (settings.cursens)
        {
          case Canbus:
            SERIALCONSOLE.println(" Canbus Current Sensor ");
            break;
          default:
            SERIALCONSOLE.println("Undefined");
            break;
        }
        SERIALCONSOLE.print("1 - invert current :");
        SERIALCONSOLE.println(settings.invertcur);
        SERIALCONSOLE.print("3 - Current Multiplication :");
        SERIALCONSOLE.println(settings.ncur);


        if ( settings.cursens == Canbus)
        {
          SERIALCONSOLE.print("7 - Can Current Sensor :");
          if (settings.curcan == IsaScale)
          {
            SERIALCONSOLE.println(" IsaScale IVT-S ");
          }
        }
        SERIALCONSOLE.println("q - Go back to menu");
        menuload = 2;
        break;

      case 98: //c for calibrate zero offset
        while (Serial.available())
        {
          Serial.read();
        }
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println("Battery Settings Menu");
        SERIALCONSOLE.println("r - Reset AH counter");
        SERIALCONSOLE.println("f - Reset to Coded Settings");
        SERIALCONSOLE.println("q - Go back to menu");
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.print("1 - Cell Over Voltage Setpoint: ");
        SERIALCONSOLE.print(settings.OverVSetpoint * 1000, 0);
        SERIALCONSOLE.print("mV");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("2 - Cell Under Voltage Setpoint: ");
        SERIALCONSOLE.print(settings.UnderVSetpoint * 1000, 0);
        SERIALCONSOLE.print("mV");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("3 - Over Temperature Setpoint: ");
        SERIALCONSOLE.print(settings.OverTSetpoint);
        SERIALCONSOLE.print("C");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("4 - Under Temperature Setpoint: ");
        SERIALCONSOLE.print(settings.UnderTSetpoint);
        SERIALCONSOLE.print("C");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("5 - Cell Balance Voltage Setpoint: ");
        SERIALCONSOLE.print(settings.balanceVoltage * 1000, 0);
        SERIALCONSOLE.print("mV");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("6 - Balance Voltage Hystersis: ");
        SERIALCONSOLE.print(settings.balanceHyst * 1000, 0);
        SERIALCONSOLE.print("mV");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("7 - Ah Battery Capacity: ");
        SERIALCONSOLE.print(settings.CAP);
        SERIALCONSOLE.print("Ah");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("8 - Pack Max Discharge: ");
        SERIALCONSOLE.print(settings.discurrentmax * 0.1);
        SERIALCONSOLE.print("A");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("9 - Cell Discharge Voltage Limit Setpoint: ");
        SERIALCONSOLE.print(settings.DischVsetpoint * 1000, 0);
        SERIALCONSOLE.print("mV");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("0 - Slave strings in parallel: ");
        SERIALCONSOLE.print(settings.Pstrings);
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("a - Cells in Series per String: ");
        SERIALCONSOLE.print(settings.Scells );
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("b - setpoint 1: ");
        SERIALCONSOLE.print(settings.socvolt[0] );
        SERIALCONSOLE.print("mV");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("c - SOC setpoint 1:");
        SERIALCONSOLE.print(settings.socvolt[1] );
        SERIALCONSOLE.print("%");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("d - setpoint 2: ");
        SERIALCONSOLE.print(settings.socvolt[2] );
        SERIALCONSOLE.print("mV");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("e - SOC setpoint 2: ");
        SERIALCONSOLE.print(settings.socvolt[3] );
        SERIALCONSOLE.print("%");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("g - Storage Setpoint: ");
        SERIALCONSOLE.print(settings.StoreVsetpoint * 1000, 0 );
        SERIALCONSOLE.print("mV");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("h - Discharge Current Taper Offset: ");
        SERIALCONSOLE.print(settings.DisTaper * 1000, 0 );
        SERIALCONSOLE.print("mV");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("j - Discharge Current Temperature Derate : ");
        SERIALCONSOLE.print(settings.DisTSetpoint);
        SERIALCONSOLE.print("C");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("k - Cell Discharge Voltage Hysteresis: ");
        SERIALCONSOLE.print(settings.DischHys * 1000, 0);
        SERIALCONSOLE.print("mV");
        SERIALCONSOLE.println();
        SERIALCONSOLE.print("l - Secondary Battery Pack Can Interface: ");
        switch (settings.secondBatteryCanIndex)
        {
          case 0:
            SERIALCONSOLE.print("No seconary Battery Pack");
            break;
          case 1:
            SERIALCONSOLE.print("Can1");
            break;
          case 2:
            SERIALCONSOLE.print("SPI");
            break;
        }
        SERIALCONSOLE.println();
        menuload = 3;
        break;

      default:
        // if nothing else matches, do the default
        // default is optional
        break;
    }
  }

  if (incomingByte == 115 & menuload == 0)
  {
    SERIALCONSOLE.println();
    SERIALCONSOLE.println("MENU");
    SERIALCONSOLE.println("Debugging Paused");
    SERIALCONSOLE.print("Firmware Version : ");
    SERIALCONSOLE.println(firmver);
    SERIALCONSOLE.println("b - Battery Settings");
    SERIALCONSOLE.println("a - Alarm and Warning Settings");
    SERIALCONSOLE.println("e - Charging Settings");
    SERIALCONSOLE.println("c - Current Sensor");
    SERIALCONSOLE.println("i - Ignore Value Settings");
    SERIALCONSOLE.println("d - Debug Settings");
    SERIALCONSOLE.println("q - exit menu");
    debug = 0;
    menuload = 1;
  }
}

//ID offset is only applied to battery module frames
void canread(int canInterfaceOffset, int idOffset)
{
  while(bmscan.read(inMsg, canInterfaceOffset)) {
    
    //Serial.print("ID: ");
    //Serial.println(inMsg.id, HEX);
    if (inMsg.id > 0x99 && inMsg.id < 0x180)//do BMS mag nic if ids are ones identified to be modules
    {
      if (candebug == 1 && debug == 1)
      {
        bms.decodecan(inMsg, 1); //do  BMS if ids are ones identified to be modules
      }
      else
      {
        bms.decodecan(inMsg, 0); //do BMS if ids are ones identified to be modules
      }
    }
    if ((inMsg.id & 0xFF0) == 0x180)    // Determine if ID is standard (11 bits) or extended (29 bits)
    {
      if (candebug == 1 && debug == 1)
      {
        bms.decodetemp(inMsg, 1 , settings.CSCvariant);
      }
      else
      {
        bms.decodetemp(inMsg, 0, settings.CSCvariant);
      }
    }

        if (candebug == 1)
    {
      Serial.print(millis());
      if ((inMsg.id & 0x80000000) == 0x80000000)    // Determine if ID is standard (11 bits) or extended (29 bits)
        sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (inMsg.id & 0x1FFFFFFF), inMsg.len);
      else
        sprintf(msgString, ",0x%.3lX,false,%1d", inMsg.id, inMsg.len);

      Serial.print(msgString);

      if ((inMsg.id & 0x40000000) == 0x40000000) {  // Determine if message is a remote request frame.
        sprintf(msgString, " REMOTE REQUEST FRAME");
        Serial.print(msgString);
      } else {
        for (byte i = 0; i < inMsg.len; i++) {
          sprintf(msgString, ", 0x%.2X", inMsg.buf[i]);
          Serial.print(msgString);
        }
      }
      Serial.print(" Can Interface: ");
      Serial.print(canInterfaceOffset);
      Serial.println();
    }
  }
}

void currentlimit()
{
  if (bmsstatus == Error)
  {
    discurrent = 0;
    chargecurrent = 0;
  }
  /*
    settings.PulseCh = 600; //Peak Charge current in 0.1A
    settings.PulseChDur = 5000; //Ms of discharge pulse derating
    settings.PulseDi = 600; //Peak Charge current in 0.1A
    settings.PulseDiDur = 5000; //Ms of discharge pulse derating
  */
  else
  {

    ///Start at no derating///
    discurrent = settings.discurrentmax;
    int maxchargingcurrent;
    if (bmsstatus == RapidCharge) {
       maxchargingcurrent = chargecurrent = settings.rapidchargecurrentmax;
    } else {
       maxchargingcurrent = chargecurrent = settings.chargecurrentmax;
}


    ///////All hard limits to into zeros
    if (bms.getLowTemperature() < settings.UnderTSetpoint)
    {
      //discurrent = 0; Request Daniel
      chargecurrent = settings.chargecurrentcold;
    }
    if (bms.getHighTemperature() > settings.OverTSetpoint)
    {
      discurrent = 0;
      chargecurrent = 0;
    }
    if (bms.getHighCellVolt() > settings.OverVSetpoint)
    {
      chargecurrent = 0;
    }
    if (bms.getHighCellVolt() > settings.OverVSetpoint)
    {
      chargecurrent = 0;
    }
    if (bms.getLowCellVolt() < settings.UnderVSetpoint || bms.getLowCellVolt() < settings.DischVsetpoint)
    {
      discurrent = 0;
    }


    //Modifying discharge current///

    if (discurrent > 0)
    {
      //Temperature based///

      if (bms.getHighTemperature() > settings.DisTSetpoint)
      {
        discurrent = discurrent - map(bms.getHighTemperature(), settings.DisTSetpoint, settings.OverTSetpoint, 0, settings.discurrentmax);
      }
      //Voltagee based///
      if (bms.getLowCellVolt() < (settings.DischVsetpoint + settings.DisTaper))
      {
        discurrent = discurrent - map(bms.getLowCellVolt(), settings.DischVsetpoint, (settings.DischVsetpoint + settings.DisTaper), settings.discurrentmax, 0);
      }

    }

    //Modifying Charge current///
    if (chargecurrent > settings.chargecurrentcold)
    {
      //Temperature based///
      if (bms.getLowTemperature() < settings.ChargeTSetpoint)
      {
        chargecurrent = chargecurrent - map(bms.getLowTemperature(), settings.UnderTSetpoint, settings.ChargeTSetpoint, (maxchargingcurrent - settings.chargecurrentcold), 0);
      }
      //Voltagee based///
      if (storagemode == 1)
      {
        if (bms.getHighCellVolt() > (settings.StoreVsetpoint - settings.ChargeHys))
        {
          chargecurrent = chargecurrent - map(bms.getHighCellVolt(), (settings.StoreVsetpoint - settings.ChargeHys), settings.StoreVsetpoint, settings.chargecurrentend, maxchargingcurrent);
        }
      }
      else
      {
        if (bms.getHighCellVolt() > (settings.ChargeVsetpoint - settings.ChargeHys))
        {
          chargecurrent = chargecurrent - map(bms.getHighCellVolt(), (settings.ChargeVsetpoint - settings.ChargeHys), settings.ChargeVsetpoint, 0, (maxchargingcurrent - settings.chargecurrentend));
        }
      }
    }

  }

  //extra safety check
  if (settings.chargertype == Outlander) {
    uint16_t fullVoltage = uint16_t(settings.ChargeVsetpoint * settings.Scells * 10);
    if (outlander_charger_reported_voltage > fullVoltage) {
//      chargecurrent = 0;
    }
    
  }
  
  ///No negative currents///

  if (discurrent < 0)
  {
    discurrent = 0;
  }
  if (chargecurrent < 0)
  {
    chargecurrent = 0;
  }
}


void resetISACounters() {
  msg.id  = 0x411;
  msg.len = 8;
  msg.buf[0] = 0x3F;
  msg.buf[1] = 0x00;
  msg.buf[2] = 0x00;
  msg.buf[3] = 0x00;
  msg.buf[4] = 0x00;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  bmscan.write(msg, settings.veCanIndex);
}

void sendcommand()
{

  ///////module id cycling/////////

  if (nextmes == 6) {
    mescycle++;
    nextmes = 0;
    if (testcycle < 4) {
      testcycle++;
    }

    if (mescycle == 0xF) {
      mescycle = 0;

      if (balancetimer < millis()) {
        balancepauze = 1;
        if (debug == 1) {
          Serial.println();
          Serial.println("Reset Balance Timer");
          Serial.println();
        }
        balancetimer = millis() + ((settings.balanceDuty + 60) * 1000);
      } else {
        balancepauze = 0;
      }
    }
  }
  if (balancepauze == 1) {
    balancecells = 0;
  }


  msg.id = 0x080 | (nextmes);
  msg.len = 8;
  if (balancecells == 1) {
    msg.buf[0] = lowByte((uint16_t((bms.getLowCellVolt()) * 1000) + 5));
    msg.buf[1] = highByte((uint16_t((bms.getLowCellVolt()) * 1000) + 5));
  } else {
    msg.buf[0] = 0xC7;
    msg.buf[1] = 0x10;
  }
  msg.buf[2] = 0x00;  //balancing bits
  msg.buf[3] = 0x00;  //balancing bits
   if (testcycle < 3) {
    msg.buf[4] = 0x20;
    msg.buf[5] = 0x00;
  } else {

    if (balancecells == 1) {
      msg.buf[4] = 0x48;
    } else {
      msg.buf[4] = 0x40;
    }
    msg.buf[5] = 0x01;
  }

  msg.buf[6] = mescycle << 4;
  if (testcycle == 2) {
    msg.buf[6] = msg.buf[6] + 0x04;
  }

  msg.buf[7] = getcheck(msg, nextmes);

  delay(2);
  bmscan.write(msg, DEFAULT_CAN_INTERFACE_INDEX);
  nextmes++;

  if (bms.checkstatus() == true) {
    resetbalancedebug();
  }
}

void resetwdog()
{
  noInterrupts();                                     //   No - reset WDT
  esp_task_wdt_reset();
  interrupts();
}


void chargercomms()
{

  if (settings.chargertype == Outlander)
  {

    msg.id = 0x285;
    msg.len = 8;
				
    msg.buf[0] = 0x0;
    msg.buf[1] = 0x0;
    msg.buf[2] = 0xb6;
    msg.buf[3] = 0x0;
    msg.buf[4] = 0x0;
    msg.buf[5] = 0x0;
    msg.buf[6] = 0x0;
    bmscan.write(msg, settings.chargerCanIndex);

    
    msg.id  = 0x286;
    msg.len = 8;
    msg.buf[0] = highByte(uint16_t(settings.ChargeVsetpoint * settings.Scells * 10));//volage
    msg.buf[1] = lowByte(uint16_t(settings.ChargeVsetpoint * settings.Scells * 10));
    msg.buf[2] = lowByte(chargecurrent / ncharger);
    msg.buf[3] = 0x0;
    msg.buf[4] = 0x0;
    msg.buf[5] = 0x0;
    msg.buf[6] = 0x0;
    bmscan.write(msg, settings.chargerCanIndex);
  }

}
uint8_t getcheck(BMS_CAN_MESSAGE &msg, int id)
{
  unsigned char canmes [11];
  int meslen = msg.len + 1; //remove one for crc and add two for id bytes
  canmes [1] = msg.id;
  canmes [0] = msg.id >> 8;

  for (int i = 0; i < (msg.len - 1); i++)
  {
    canmes[i + 2] = msg.buf[i];
  }
  /*
    Serial.println();
    for (int i = 0; i <  meslen; i++)
    {
    Serial.print(canmes[i], HEX);
    Serial.print("|");
    }
  */
  return (crc8.get_crc8(canmes, meslen, finalxor[id]));
}

void resetbalancedebug()
{
  msg.id  =  0x0B0; //broadcast to all Elteks
  msg.len = 8;
//  msg.ext = 0;
  msg.buf[0] = 0xFF;
  msg.buf[1] = 0x00;
  msg.buf[2] = 0xCD;
  msg.buf[3] = 0xA2;
  msg.buf[4] = 0x00;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;

//  Can0.write(msg);
   bmscan.write(msg, DEFAULT_CAN_INTERFACE_INDEX);
}

void resetIDdebug()
{
  //Rest all possible Ids
  for (int ID = 0; ID < 15; ID++)
  {
    msg.id  =  0x0A0; //broadcast to all CSC
    msg.len = 8;
//    msg.ext = 0;
    msg.buf[0] = 0xA1;
    msg.buf[1] = ID;
    msg.buf[2] = 0xFF;
    msg.buf[3] = 0xFF;
    msg.buf[4] = 0xFF;
    msg.buf[5] = 0xFF;
    msg.buf[6] = 0xFF;
    msg.buf[7] = 0xFF;

 //   Can0.write(msg);
    bmscan.write(msg, DEFAULT_CAN_INTERFACE_INDEX);
    delay(2);
  }
  //NextID = 0;

  //check for found unassigned CSC
  Unassigned = 0;

  msg.id  =  0x0A0; //broadcast to all CSC
  msg.len = 8;
//  msg.ext = 0;
  msg.buf[0] = 0x37;
  msg.buf[1] = 0xFF;
  msg.buf[2] = 0xFF;
  msg.buf[3] = 0xFF;
  msg.buf[4] = 0xFF;
  msg.buf[5] = 0xFF;
  msg.buf[6] = 0xFF;
  msg.buf[7] = 0xFF;

//  Can0.write(msg);
   bmscan.write(msg, DEFAULT_CAN_INTERFACE_INDEX);
}

void findUnassigned ()
{
  Unassigned = 0;
  //check for found unassigned CSC
  msg.id  =  0x0A0; //broadcast to all CSC
  msg.len = 8;
//  msg.ext = 0;
  msg.buf[0] = 0x37;
  msg.buf[1] = 0xFF;
  msg.buf[2] = 0xFF;
  msg.buf[3] = 0xFF;
  msg.buf[4] = 0xFF;
  msg.buf[5] = 0xFF;
  msg.buf[6] = 0xFF;
  msg.buf[7] = 0xFF;

//  Can0.write(msg);
   bmscan.write(msg, DEFAULT_CAN_INTERFACE_INDEX);
}

void assignID()
{
  msg.id  =  0x0A0; //broadcast to all CSC
  msg.len = 8;
//  msg.ext = 0;
  msg.buf[0] = 0x12;
  msg.buf[1] = 0xAB;
  msg.buf[2] = DMC[0];
  msg.buf[3] = DMC[1];
  msg.buf[4] = DMC[2];
  msg.buf[5] = DMC[3];
  msg.buf[6] = 0xFF;
  msg.buf[7] = 0xFF;

//  Can0.write(msg);
   bmscan.write(msg, DEFAULT_CAN_INTERFACE_INDEX);

  delay(30);

  msg.buf[1] = 0xBA;
  msg.buf[2] = DMC[4];
  msg.buf[3] = DMC[5];
  msg.buf[4] = DMC[6];
  msg.buf[5] = DMC[7];

//  Can0.write(msg);
   bmscan.write(msg, DEFAULT_CAN_INTERFACE_INDEX);

  delay(10);
  msg.buf[0] = 0x5B;
  msg.buf[1] = NextID;
//  Can0.write(msg);
   bmscan.write(msg, DEFAULT_CAN_INTERFACE_INDEX);
  
  delay(10);
  msg.buf[0] = 0x37;
  msg.buf[1] = NextID;
//  Can0.write(msg);
   bmscan.write(msg, DEFAULT_CAN_INTERFACE_INDEX);

  NextID++;

  findUnassigned();
}


int pgnFromCANId(int canId)
{
  if ((canId & 0x10000000) == 0x10000000)
  {
    return (canId & 0x03FFFF00) >> 8;
  }
  else
  {
    return canId; // not sure if this is really right?
  }
}

////////END///////////
