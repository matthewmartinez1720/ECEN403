/*
 * ESP32 Slave — Byte-Level Protocol Responder
 * COM5
 *
 * SPI slave runs in a dedicated FreeRTOS task using blocking
 * spi_slave_transmit() so it's always ready for the next transaction.
 */

#include <Arduino.h>
#include <Wire.h>
#include <driver/spi_slave.h>

constexpr uint8_t  I2C_SLAVE_ADDR = 0x08;
constexpr uint32_t UART2_BAUD     = 9600;
constexpr uint8_t  LED_PIN        = 2;

constexpr int SPI_MOSI_PIN = 23;
constexpr int SPI_MISO_PIN = 19;
constexpr int SPI_SCLK_PIN = 18;
constexpr int SPI_CS_PIN   = 5;

// ===================== I2C Slave =====================
volatile uint8_t i2cRxBuf[64];
volatile uint8_t i2cRxLen = 0;
volatile bool    i2cNewData = false;

void onI2CReceive(int numBytes) {
    i2cRxLen = 0;
    while (Wire.available() && i2cRxLen < 64) {
        i2cRxBuf[i2cRxLen++] = Wire.read();
    }
    i2cNewData = true;
}

void onI2CRequest() {
    if (i2cRxLen > 0) {
        Wire.write((const uint8_t*)i2cRxBuf, i2cRxLen);
    }
}

// ===================== SPI Slave =====================
const uint8_t misoPattern[] = {
    0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80,
    0x90, 0xA0, 0xB0, 0xC0, 0xD0, 0xE0, 0xF0, 0xFF
};

void spiSlaveTask(void* param) {
    WORD_ALIGNED_ATTR uint8_t rxBuf[64];
    WORD_ALIGNED_ATTR uint8_t txBuf[64];
    int txnCount = 0;

    while (true) {
        memset(rxBuf, 0, sizeof(rxBuf));
        memcpy(txBuf, misoPattern, sizeof(misoPattern));

        spi_slave_transaction_t trans = {};
        trans.length    = 64 * 8;
        trans.tx_buffer = txBuf;
        trans.rx_buffer = rxBuf;

        // Blocks here until master clocks a full transaction
        esp_err_t ret = spi_slave_transmit(VSPI_HOST, &trans, portMAX_DELAY);

        if (ret == ESP_OK && trans.trans_len > 0) {
            int bytes = trans.trans_len / 8;
            txnCount++;

            Serial.printf("[SPI  RX] Txn %d (%d bytes)\n", txnCount, bytes);
            Serial.print("  MOSI: ");
            for (int i = 0; i < bytes; i++) Serial.printf("%02X ", rxBuf[i]);
            Serial.println();
            Serial.print("  MISO: ");
            for (int i = 0; i < bytes; i++) Serial.printf("%02X ", misoPattern[i]);
            Serial.println();
        }
    }
}

void setupSPISlave() {
    spi_bus_config_t busConfig = {};
    busConfig.mosi_io_num   = SPI_MOSI_PIN;
    busConfig.miso_io_num   = SPI_MISO_PIN;
    busConfig.sclk_io_num   = SPI_SCLK_PIN;
    busConfig.quadwp_io_num = -1;
    busConfig.quadhd_io_num = -1;

    spi_slave_interface_config_t slaveConfig = {};
    slaveConfig.mode          = 0;
    slaveConfig.spics_io_num  = SPI_CS_PIN;
    slaveConfig.queue_size    = 1;
    slaveConfig.post_trans_cb = NULL;

    esp_err_t ret = spi_slave_initialize(VSPI_HOST, &busConfig, &slaveConfig, SPI_DMA_DISABLED);
    if (ret != ESP_OK) {
        Serial.printf("[SPI] Init failed: %s\n", esp_err_to_name(ret));
        return;
    }

    // Launch SPI listener on core 1
    xTaskCreatePinnedToCore(spiSlaveTask, "spi_slave", 4096, NULL, 5, NULL, 1);
    Serial.println("[SPI] Slave task started");
}

// ===================== Main =====================
void setup() {
    Serial.begin(115200);
    delay(1000);

    pinMode(LED_PIN, OUTPUT);

    Serial2.begin(UART2_BAUD, SERIAL_8N1, 16, 17);

    Wire.begin(I2C_SLAVE_ADDR);
    Wire.onReceive(onI2CReceive);
    Wire.onRequest(onI2CRequest);

    setupSPISlave();

    Serial.println();
    Serial.println("===================================");
    Serial.println("  ESP32 SLAVE - Byte-Level Monitor");
    Serial.println("===================================");
    Serial.println("  UART @ 9600 baud (RX=GPIO16)");
    Serial.println("  SPI  slave Mode 0 (CS=GPIO5)");
    Serial.println("    MISO pattern: 10 20 30 40 50...");
    Serial.println("  I2C  slave addr 0x08");
    Serial.println("===================================");
    Serial.println("Listening...\n");
}

void loop() {
    // ---- UART ----
    if (Serial2.available()) {
        digitalWrite(LED_PIN, HIGH);
        Serial.print("[UART RX] ");
        while (Serial2.available()) {
            uint8_t b = Serial2.read();
            Serial.printf("%02X ", b);
        }
        Serial.println();
        digitalWrite(LED_PIN, LOW);
    }

    // ---- I2C ----
    if (i2cNewData) {
        i2cNewData = false;
        digitalWrite(LED_PIN, HIGH);
        Serial.printf("[I2C  RX] %d bytes: ", i2cRxLen);
        for (int i = 0; i < i2cRxLen; i++) {
            Serial.printf("%02X ", i2cRxBuf[i]);
        }
        Serial.println();
        digitalWrite(LED_PIN, LOW);
    }

    delay(10);
}
