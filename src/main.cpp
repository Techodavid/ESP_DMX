#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SPIFFS.h>

#include "driver/uart.h"
#include "freertos/queue.h"

// ================= CONFIG =================
#define AP_SSID "ESP32-DMX-DEBUG"
#define AP_PASS "12345678"

#define DMX_UART UART_NUM_1
#define DMX_RX_PIN 5
#define DMX_BUF_SIZE 600
// ==========================================


// =====================================================
// ================= DMX RECEIVER ======================
// =====================================================

class DmxReceiver {

public:
    uint8_t universe[513] = {0};

    uint32_t lastFrameMicros = 0;
    uint32_t frameInterval = 0;
    uint32_t breakCount = 0;
    uint32_t frameCount = 0;

    bool signalPresent = false;

private:
    QueueHandle_t uart_queue;
    uint16_t slotIndex = 0;
    bool receiving = false;
    uint32_t lastFrameMillis = 0;

public:

    void begin() {

        uart_config_t uart_config = {};
        uart_config.baud_rate = 250000;
        uart_config.data_bits = UART_DATA_8_BITS;
        uart_config.parity = UART_PARITY_DISABLE;
        uart_config.stop_bits = UART_STOP_BITS_2;
        uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;

        uart_driver_install(DMX_UART, DMX_BUF_SIZE, 0, 20, &uart_queue, 0);
        uart_param_config(DMX_UART, &uart_config);
        uart_set_pin(DMX_UART, UART_PIN_NO_CHANGE, DMX_RX_PIN,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

        uart_enable_rx_intr(DMX_UART);
    }

    void loop() {

        uart_event_t event;

        while (xQueueReceive(uart_queue, &event, 0)) {

            if (event.type == UART_BREAK) {

                breakCount++;

                uint32_t now = micros();
                frameInterval = now - lastFrameMicros;
                lastFrameMicros = now;

                lastFrameMillis = millis();
                signalPresent = true;

                slotIndex = 0;
                receiving = true;
            }

            if (event.type == UART_DATA && receiving) {

                uint8_t buf[128];
                int len = uart_read_bytes(DMX_UART, buf, event.size, 0);

                for (int i = 0; i < len; i++) {
                    if (slotIndex < 513) {
                        universe[slotIndex++] = buf[i];
                    }
                }

                if (slotIndex >= 513) {
                    frameCount++;
                    receiving = false;
                }
            }
        }

        // Signal lost detection
        if (millis() - lastFrameMillis > 200) {
            signalPresent = false;
        }
    }
};


// =====================================================
// ================= SYSTEM MONITOR ====================
// =====================================================

class SystemMonitor {
public:
    float getTemperature() {
        return temperatureRead();
    }
};


// =====================================================
// ================= WEB INTERFACE =====================
// =====================================================

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
        for (int i = 1; i <= 16; i++) {
            json += String(dmx->universe[i]);
            if (i < 16) json += ",";
        }
        json += "]";

        json += "}";

        server.send(200, "application/json", json);
    }
};


// =====================================================
// ================= GLOBALS ===========================
// =====================================================

DmxReceiver dmx;
SystemMonitor monitor;
WebInterface web;


// =====================================================
// ================= SETUP / LOOP ======================
// =====================================================

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
