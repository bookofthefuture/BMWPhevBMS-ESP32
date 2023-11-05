#ifndef BMS_WEB_SERVER_H
#define BMS_WEB_SERVER_H
#include "ESPAsyncWebServer.h"
#include "AsyncJson.h"
#include "ArduinoJson.h"
#include "config.h"
#include "BMSModuleManager.h"

class BMSWebServer
{
  public:
    BMSWebServer(EEPROMSettings& settings, BMSModuleManager &bms);
    void setup();
    void execute();
    void broadcast(const char * message);
    AsyncWebSocket& getWebSocket();
  private:
    EEPROMSettings& settings;
    BMSModuleManager& bms;

};
#endif
