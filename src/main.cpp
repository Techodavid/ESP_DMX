#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SPIFFS.h>

#include "dmx_receiver.h"

// ================= CONFIG =================
#define AP_SSID "ESP32-DMX-DEBUG"
#define AP_PASS "12345678"
// ==========================================

class SystemMonitor {
public:
    float getTemperature() {
        return temperatureRead();
    }
};

class WebInterface {

private:
    WebServer server{80};
    DmxReceiver* dmx;
    SystemMonitor* monitor;

public:

    void begin(DmxReceiver* d, SystemMonitor* m) {

        dmx = d;
        monitor = m;

        server.on("/", [this]() { handleRoot(); });
        server.on("/data", [this]() { handleData(); });

        server.begin();
    }

    void loop() {
        server.handleClient();
    }

private:

    void handleRoot() {

        File file = SPIFFS.open("/index.html", "r");

        if (!file) {
            server.send(500, "text/plain", "index.html not found");
            return;
        }

        server.streamFile(file, "text/html");
        file.close();
    }

    void handleData() {

        String json = "{";

        json += "\"frames\":" + String(dmx->frameCount) + ",";
        json += "\"interval\":" + String(dmx->frameInterval) + ",";
        json += "\"signal\":" + String(dmx->signalPresent ? "true" : "false") + ",";
        json += "\"temp\":" + String(monitor->getTemperature()) + ",";

        json += "\"ch\":[";
        for (int i = 2; i <= 512; i++) {
            json += String(dmx->universe[i]);
            if (i < 512) json += ",";
        }
        json += "]";

        json += "}";

        server.send(200, "application/json", json);
    }
};

DmxReceiver dmx;
SystemMonitor monitor;
WebInterface web;

void setup() {

    Serial.begin(115200);

    WiFi.softAP(AP_SSID, AP_PASS);

    if (!SPIFFS.begin(true)) {
        Serial.println("SPIFFS Mount Failed");
    }

    dmx.begin();
    web.begin(&dmx, &monitor);
}

void loop() {

    dmx.loop();
    web.loop();
}
