#ifndef DMX_RECEIVER_H
#define DMX_RECEIVER_H

#include <Arduino.h>
#include "driver/uart.h"
#include "freertos/queue.h"

#define DMX_UART UART_NUM_1
#define DMX_RX_PIN 5
#define DMX_BUF_SIZE 600

class DmxReceiver {

public:
    uint8_t universe[513] = {0};

    uint32_t lastFrameMicros = 0;
    uint32_t frameInterval = 0;
    uint32_t breakCount = 0;
    uint32_t frameCount = 0;

    bool signalPresent = false;

    void begin();
    void loop();

private:
    QueueHandle_t uart_queue;
    uint16_t slotIndex = 0;
    bool receiving = false;
    uint32_t lastFrameMillis = 0;
};

#endif
