#include "BMSWebServer.h"
#include "SPIFFS.h"
#include <EEPROM.h>
#include <Update.h>

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
AsyncEventSource events("/events");

//move from global
extern int SOC;
extern byte bmsstatus;
extern byte evse_duty;
extern double amphours;
extern bool chargeEnabled();
extern int chargecurrent;
extern void resetISACounters();
extern float currentact;
extern int balancecells;

#define Boot 0
#define Ready 1
#define Drive 2
#define Charge 3
#define Precharge 4
#define RapidCharge 5
#define Error 6

// Over-the-air firmware update state, shared between the upload and completion callbacks
static String otaError;

// ESP32 app images start with magic 0xE9 and carry the chip id (0 = ESP32) at bytes 12-13, followed by
// the app descriptor (magic 0xABCD5432, little endian) at byte 32. The descriptor tells app images apart
// from bootloader images, and this also rejects the -full.bin and -spiffs.bin release files.
static bool validAppImage(const uint8_t *data, size_t len) {
  return len >= 36 && data[0] == 0xE9 && data[12] == 0x00 && data[13] == 0x00
         && data[32] == 0x32 && data[33] == 0x54 && data[34] == 0xCD && data[35] == 0xAB;
}

// Writing flash while charging or driving could stall CAN handling, so only update when idle
static bool otaAllowed() {
  return bmsstatus == Boot || bmsstatus == Ready || bmsstatus == Error;
}

BMSWebServer::BMSWebServer(EEPROMSettings& s, BMSModuleManager& b) : settings{ s }, bms { b } {
}

AsyncWebSocket& BMSWebServer::getWebSocket() {
  return ws;
}
void BMSWebServer::execute() {
  ws.cleanupClients();
}

void BMSWebServer::broadcast(const char * message) {
  ws.printfAll(message);
}

void BMSWebServer::setup()
{
  // ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/wifi", [&] (AsyncWebServerRequest * request) {
    bool updated = true;
    if (request->hasParam("apSSID", true) && request->hasParam("apPW", true))
    {
      WiFi.softAP(request->arg("apSSID").c_str(), request->arg("apPW").c_str());
    }
    else if (request->hasParam("staSSID", true) && request->hasParam("staPW", true))
    {
      WiFi.mode(WIFI_AP_STA);
      WiFi.begin(request->arg("staSSID").c_str(), request->arg("staPW").c_str());
    }
    else
    {
      File file = SPIFFS.open("/wifi.html", "r");
      String html = file.readString();
      file.close();
      html.replace("%staSSID%", WiFi.SSID());
      html.replace("%apSSID%", WiFi.softAPSSID());
      html.replace("%staIP%", WiFi.localIP().toString());
      request->send(200, "text/html", html);
      updated = false;
    }

    if (updated)
    {
      request->send(SPIFFS, "/wifi-updated.html");
    }
  });

  server.on("/dashboard", HTTP_GET, [&] (AsyncWebServerRequest * request) {
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    DynamicJsonDocument json(10240);

    json["packVolts"] = bms.getPackVoltage();
    json["minVolt"] = bms.getLowCellVolt();
    json["maxVolt"] = bms.getHighCellVolt();
    json["cellDelta"] = bms.getHighCellVolt() - bms.getLowCellVolt();
    json["avgTemp"] = bms.getAvgTemperature();
    json["evseDuty"] = evse_duty;
//    json["chargerStatus"] = outlanderCharger.reported_status;
//    json["chargerTemp1"] = outlanderCharger.reported_temp1;
//    json["chargerTemp2"] = outlanderCharger.reported_temp2;
//    json["chargerVoltage"] = outlanderCharger.reported_voltage;
//    json["chargerCurrent"] = outlanderCharger.reported_dc_current;
    json["requestedchargecurrent.val"] = chargecurrent;
//    json["contactorStatus"] = bms.contactorsClosed();
    json["chargeEnabled"] = chargeEnabled();
//    json["chargeOverride"] = io.getChargeOverride();
    json["ahUsed"] = amphours;
    json["soc"] = SOC;
    json["capacity.val"] = settings.CAP;
    json["current.val"] = currentact;
    json["balanceActive"] = balancecells;
    
    if (bmsstatus == Boot) {
      json["status"] = "Boot";
    } else if (bmsstatus == Ready) {
      json["status"] = "Ready";
    } else if (bmsstatus == Drive) {
      json["status"] = "Drive";
    } else if (bmsstatus == Charge) {
      json["status"] = "Charge";
    } else if (bmsstatus == Precharge) {
      json["status"] = "Precharge";
    }  else if (bmsstatus == RapidCharge) {
      json["status"] = "Rapid Charge";
    } else if (bmsstatus == Error) {
      json["status"] = "Error";
    }

    serializeJson(json, *response);
    request->send(response);
  });

  server.on("/voltages", HTTP_GET, [&] (AsyncWebServerRequest * request) {
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    DynamicJsonDocument json(20480);

    bms.printPackDetailsJson(json);
    serializeJson(json, *response);
    request->send(response);
  });
//
  server.on("/config", HTTP_GET, [&] (AsyncWebServerRequest * request) {
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    DynamicJsonDocument json(2048);
    json["capacity"] = settings.CAP;
    json["seriesCells"] = settings.Scells;
    json["parallelCells"] = settings.Pstrings ;
    json["overVSetpoint"] = settings.OverVSetpoint * 1000;
    json["underVSetpoint"] = settings.UnderVSetpoint * 1000;
    json["chargeVsetpoint"] = settings.ChargeVsetpoint * 1000;
    json["dischVsetpoint"] = settings.DischVsetpoint * 1000;
    json["overTSetpoint"] = settings.OverTSetpoint;
    json["underTSetpoint"] = settings.UnderTSetpoint;
    json["chargeTSetpoint"] = settings.ChargeTSetpoint;
    json["disTSetpoint"] = settings.DisTSetpoint;
    json["balanceVoltage"] = settings.balanceVoltage * 1000;
    json["balanceHyst"] = settings.balanceHyst * 1000;
    json["carCanIndex"] = settings.veCanIndex;
    json["chargecurrentmax"] = settings.chargecurrentmax;
    serializeJson(json, *response);
    request->send(response);
  });


  server.on(
    "/config",
    HTTP_POST,
  [](AsyncWebServerRequest * request) {},
  NULL,
  [&](AsyncWebServerRequest * request, uint8_t *data, size_t len, size_t index, size_t total) {
    Serial.println("Config POST");
    const size_t JSON_DOC_SIZE = 1024U;
    DynamicJsonDocument jsonDoc(JSON_DOC_SIZE);

    if (DeserializationError::Ok == deserializeJson(jsonDoc, (const char*)data))
    {
      JsonObject json = jsonDoc.as<JsonObject>();
      settings.CAP = json["capacity"];
      settings.Scells= json["seriesCells"];
      settings.Pstrings = json["parallelCells"];
      settings.OverVSetpoint = ((float)json["overVSetpoint"] / 1000);
      settings.UnderVSetpoint = ((float)json["underVSetpoint"]/ 1000);
      settings.ChargeVsetpoint = ((float)json["chargeVsetpoint"]/ 1000);
      settings.DischVsetpoint = ((float)json["dischVsetpoint"]/ 1000);
      settings.OverTSetpoint = json["overTSetpoint"];
      settings.UnderTSetpoint = json["underTSetpoint"];
      settings.ChargeTSetpoint = json["chargeTSetpoint"];
      settings.DisTSetpoint = json["disTSetpoint"];
      settings.balanceVoltage = ((float)json["balanceVoltage"])/ 1000;
      settings.balanceHyst = ((float)json["balanceHyst"])/ 1000;
      settings.veCanIndex = json["carCanIndex"];
      settings.chargecurrentmax = json["chargecurrentmax"];
      EEPROM.put(0, settings); //save all change to eeprom
      EEPROM.commit();
      request->send(200, "application/json", "success");

    } else {
      request->send(200, "application/json", "DeserializationError");
    }
  });


    server.on("/cmd", HTTP_POST, [](AsyncWebServerRequest *request){
        String message;
        if (request->hasParam("cmd", true)) {
            message = request->getParam("cmd", true)->value();
            Serial.print("CMD: ");
            Serial.println(message);
            if (message == "r") {
              resetISACounters();
            }
        }
        request->send(200, "application/json", "{\"result\": \"success\"}");
    });


  server.on("/update",
            HTTP_POST,
  [](AsyncWebServerRequest * request) {
    if (otaError.length() > 0) {
      request->send(otaError.startsWith("Busy") ? 409 : 400, "text/plain", otaError);
    } else {
      AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", "Update complete, restarting");
      response->addHeader("Connection", "close");
      request->onDisconnect([]() {
        Serial.println("Firmware updated, restarting");
        ESP.restart();
      });
      request->send(response);
    }
  },
  [](AsyncWebServerRequest * request, const String & filename, size_t index, uint8_t *data, size_t len, bool final) {
    if (!index) {
      otaError = "";
      if (Update.isRunning()) {
        Update.abort(); // an earlier upload was interrupted
      }
      if (!otaAllowed()) {
        otaError = "Busy: the BMS is not idle (charging or driving)";
      } else if (!filename.endsWith(".bin")) {
        otaError = "Not a .bin file";
      } else if (!validAppImage(data, len)) {
        otaError = "Not an ESP32 firmware image. Use the *-firmware.bin file";
      } else if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        otaError = Update.errorString();
      } else {
        Serial.print("Firmware update started: ");
        Serial.println(filename);
      }
    }

    if (otaError.length() > 0) {
      return; // already failed, ignore the remaining chunks
    }

    if (len && Update.write(data, len) != len) {
      otaError = Update.errorString();
      Update.abort();
      return;
    }

    if (final && !Update.end(true)) {
      otaError = Update.errorString();
    }
  }

           );

  // Uploads a file into SPIFFS (web UI files). Firmware goes to /update instead.
  server.on("/edit",
            HTTP_POST,
  [](AsyncWebServerRequest * request) {},
  [&](AsyncWebServerRequest * request, const String & filename, size_t index, uint8_t *data, size_t len, bool final) {
    bool isBin = filename.endsWith(".bin");

    if (!index && !isBin) {
      // open the file on first call and store the file handle in the request object
      request->_tempFile = SPIFFS.open("/" + filename, "w");

    }

    if (len && !isBin) {
      // stream the incoming chunk to the opened file
      request->_tempFile.write(data, len);
    }

    if (final) {
      if (isBin) {
        request->send(400, "text/plain", "Firmware (.bin) files go through Firmware Update");
        return;
      }
      // close the file handle as the upload is now done
      request->_tempFile.close();
      request->redirect("/");
    }
  }

           );

//  server.on("/data",
//            HTTP_POST,
//  [](AsyncWebServerRequest * request) {},
//  NULL,
//  [&](AsyncWebServerRequest * request, uint8_t *data, size_t len, size_t index, size_t total) {
//    if (request->hasParam("chargeOverride", true)) {
//      AsyncWebParameter* p = request->getParam("chargeOverride", true);
//      bms.getIO().setChargeOverride(p->value().c_str() == "true");
//    }
//
//    request->send(200, "application/json", "success");
//
//  }
//
// );

  server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");


  // Start server
  Serial.println("Starting Web Server");

  server.begin();
}
