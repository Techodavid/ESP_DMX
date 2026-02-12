#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

#include "driver/uart.h"
#include "freertos/queue.h"

// ================= CONFIG =================
#define AP_SSID "ESP32-DMX-DEBUG"
#define AP_PASS "12345678"

#define DMX_UART UART_NUM_1
#define DMX_RX_PIN 5
#define DMX_BUF_SIZE 600

uint16_t slotIndex = 0;
bool receiving = false;


// ==========================================

class DmxReceiver {
public:
    uint8_t universe[513]; // [0]=startcode, [1..512]=channels
    uint32_t lastFrameMicros = 0;
    uint32_t frameInterval = 0;
    uint32_t breakCount = 0;
    uint32_t frameCount = 0;

    QueueHandle_t uart_queue;

    void begin() {
        uart_config_t uart_config = {
            .baud_rate = 250000,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_2,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
        };

        uart_driver_install(DMX_UART, DMX_BUF_SIZE, 0, 20, &uart_queue, 0);
        uart_param_config(DMX_UART, &uart_config);
        uart_set_pin(DMX_UART, UART_PIN_NO_CHANGE, DMX_RX_PIN,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

      //  uart_set_line_inverse(DMX_UART, UART_SIGNAL_RXD_INV); // wichtig bei RS485
        uart_enable_rx_intr(DMX_UART);
    }

    void loop() {

    uart_event_t event;

    while (xQueueReceive(uart_queue, &event, 0)) {

        if (event.type == UART_BREAK) {

            breakCount++;

            // Neues Frame beginnt
            slotIndex = 0;
            receiving = true;

            uint32_t now = micros();
            frameInterval = now - lastFrameMicros;
            lastFrameMicros = now;
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
}


private:
    void readFrame() {

    uart_flush_input(DMX_UART);
    delayMicroseconds(120);

    uint8_t temp[513];
    int total = 0;

    uint32_t start = millis();

    while ((millis() - start) < 30) {

        int len = uart_read_bytes(
            DMX_UART,
            temp + total,
            sizeof(temp) - total,
            2 / portTICK_PERIOD_MS
        );

        if (len > 0) {
            total += len;
        } else {
            // nichts mehr da → Frame vermutlich komplett
            break;
        }
    }

    if (total >= 2 && temp[0] == 0) {

        memset(universe, 0, sizeof(universe));   // vorher alles nullen
        memcpy(universe, temp, total);           // nur empfangene Slots kopieren

        uint32_t now = micros();
        frameInterval = now - lastFrameMicros;
        lastFrameMicros = now;
        frameCount++;
    }
}



};

class SystemMonitor {
public:
    float getTemperature() {
        return temperatureRead();
    }
};

class WebInterface {
public:
    WebServer server{80};
    DmxReceiver* dmx;
    SystemMonitor* monitor;

    void begin(DmxReceiver* d, SystemMonitor* m) {
        dmx = d;
        monitor = m;

        server.on("/", [&]() { handleRoot(); });
        server.begin();
    }

    void loop() {
        server.handleClient();
    }

private:
    void handleRoot() {
        String html = "<html><head><meta http-equiv='refresh' content='1'></head><body>";

        html += "<h2>DMX Debug</h2>";
        html += "Frames: " + String(dmx->frameCount) + "<br>";
        html += "Breaks: " + String(dmx->breakCount) + "<br>";
        html += "Frame Interval (us): " + String(dmx->frameInterval) + "<br>";
        html += "ESP Temp: " + String(monitor->getTemperature()) + " C<br>";

        html += "<h3>Channels 1-16</h3><pre>";
        for (int i = 1; i <= 16; i++) {
            html += String(i) + ": " + String(dmx->universe[i]) + "\n";
        }
        html += "</pre>";

        html += "<h3>Raw First 32 Bytes</h3><pre>";
        for (int i = 0; i < 32; i++) {
            html += String(dmx->universe[i]) + " ";
        }
        html += "</pre>";

        html += "</body></html>";

        server.send(200, "text/html", html);
    }
};

// ================= GLOBALS =================
DmxReceiver dmx;
SystemMonitor monitor;
WebInterface web;

// ==========================================

void setup() {
    Serial.begin(115200);

    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.println("AP started");

    dmx.begin();
    web.begin(&dmx, &monitor);
}

void loop() {
    dmx.loop();
    web.loop();
}
