#include "dmx_receiver.h"

void DmxReceiver::begin() {

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

void DmxReceiver::loop() {

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

    if (millis() - lastFrameMillis > 200) {
        signalPresent = false;
        memset(universe, 0, sizeof(universe));
    }
}
