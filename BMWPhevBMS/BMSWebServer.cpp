#include "BMSWebServer.h"
#include "SPIFFS.h"

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

#define Boot 0
#define Ready 1
#define Drive 2
#define Charge 3
#define Precharge 4
#define RapidCharge 5
#define Error 6

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
//  server.on("/config", HTTP_GET, [&] (AsyncWebServerRequest * request) {
//    AsyncResponseStream *response = request->beginResponseStream("application/json");
//    DynamicJsonDocument json(2048);
//
//    config.toJson(settings, json);
//    serializeJson(json, *response);
//    request->send(response);
//  });

//  server.on(
//    "/config",
//    HTTP_POST,
//  [](AsyncWebServerRequest * request) {},
//  NULL,
//  [&](AsyncWebServerRequest * request, uint8_t *data, size_t len, size_t index, size_t total) {
//    Serial.println("Config POST");
//    const size_t JSON_DOC_SIZE = 1024U;
//    DynamicJsonDocument jsonDoc(JSON_DOC_SIZE);
//
//    if (DeserializationError::Ok == deserializeJson(jsonDoc, (const char*)data))
//    {
//      JsonObject obj = jsonDoc.as<JsonObject>();
//      config.fromJson(settings, obj);
//      Serial.print("Settings: ");
//      Serial.println(settings.acDetectionMethod);
//      config.save(settings);
//      request->send(200, "application/json", "success");
//
//    } else {
//      request->send(200, "application/json", "DeserializationError");
//    }
//  });


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


  server.on("/edit",
            HTTP_POST,
  [](AsyncWebServerRequest * request) {},
  [&](AsyncWebServerRequest * request, const String & filename, size_t index, uint8_t *data, size_t len, bool final) {
    if (!index) {
      // open the file on first call and store the file handle in the request object
      request->_tempFile = SPIFFS.open("/" + filename, "w");

    }

    if (len) {
      // stream the incoming chunk to the opened file
      request->_tempFile.write(data, len);
    }

    if (final) {
      // close the file handle as the upload is now done
      request->_tempFile.close();

      if (filename.substring(filename.lastIndexOf(".")).equals("bin")) {
        Serial.println("Firmware uploaded, restarting");
        request->send(200, "application/json", "restarting");
        ESP.restart();
      }
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
