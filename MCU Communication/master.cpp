/*
 * ESP32 Master — Byte-Level Protocol Generator (Manual Mode)
 * COM3
 *
 * Commands:
 *   s = Send 3 SPI transactions (1, 3, 5 bytes) — shows MOSI and MISO
 *   i = Send I2C write then read
 *   u = Send UART bytes
 *   c = Send CAN frame (0x123, 4 bytes: DE AD BE EF)
 *   a = Send all
 *   h = Help
 */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "driver/twai.h"
#include "soc/gpio_sig_map.h"

constexpr uint8_t  I2C_SLAVE_ADDR = 0x08;
constexpr uint32_t UART2_BAUD     = 9600;
constexpr uint8_t  SPI_CS_PIN     = 5;
constexpr uint8_t  LED_PIN        = 2;

constexpr gpio_num_t CAN_TX_PIN      = GPIO_NUM_25;
constexpr gpio_num_t CAN_TX_DUMMY_RX = GPIO_NUM_26;

// ===================== SPI =====================
void spiTransaction(const uint8_t* txData, int len, int txnNum) {
    uint8_t rxData[16] = {0};

    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(SPI_CS_PIN, LOW);
    delay(1);

    for (int i = 0; i < len; i++) {
        rxData[i] = SPI.transfer(txData[i]);
    }

    digitalWrite(SPI_CS_PIN, HIGH);
    SPI.endTransaction();

    Serial.printf("  TX%d MOSI (%d bytes): ", txnNum, len);
    for (int i = 0; i < len; i++) Serial.printf("%02X ", txData[i]);
    Serial.println();

    Serial.printf("  TX%d MISO (%d bytes): ", txnNum, len);
    for (int i = 0; i < len; i++) Serial.printf("%02X ", rxData[i]);
    Serial.println();
}

void sendSPI() {
    Serial.println("[SPI] Starting 3 transactions...");

    uint8_t tx1[] = {0xA5};
    spiTransaction(tx1, 1, 1);
    delay(100);

    uint8_t tx2[] = {0xDE, 0xAD, 0x01};
    spiTransaction(tx2, 3, 2);
    delay(100);

    uint8_t tx3[] = {0xCA, 0xFE, 0xBA, 0xBE, 0x55};
    spiTransaction(tx3, 5, 3);

    Serial.println("[SPI] Done.\n");
}

void sendSPIoverflow() {
    Serial.println("[SPI Overflow] Starting  transactions...");
    uint8_t tx[] = {0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5};
    for (int i = 1; i < 34; i++) {
        spiTransaction(tx, 8, i);
        delay(100);
    }

    Serial.println("[SPI Overflow] Done.\n");
}

// ===================== I2C =====================
void sendI2C() {
    Serial.println("[I2C] Write 4 bytes to slave 0x08...");

    uint8_t txData[] = {0x01, 0x02, 0x03, 0x04};
    Wire.beginTransmission(I2C_SLAVE_ADDR);
    Wire.write(txData, 4);
    uint8_t err = Wire.endTransmission();

    if (err == 0) {
        Serial.println("  W: [08+W] 01 02 03 04  ACK");
    } else {
        Serial.printf("  W: ERROR %d\n", err);
    }

    delay(50);

    Serial.println("[I2C] Read 4 bytes from slave 0x08...");
    uint8_t count = Wire.requestFrom(I2C_SLAVE_ADDR, (uint8_t)4);
    if (count > 0) {
        Serial.print("  R: [08+R] ");
        while (Wire.available()) {
            Serial.printf("%02X ", Wire.read());
        }
        Serial.println();
    } else {
        Serial.println("  R: No response");
    }

    Serial.println("[I2C] Done.\n");
}


// ===================== UART =====================
void sendUART() {
    Serial.println("[UART] Sending 6 bytes...");

    uint8_t txData[] = {0x55, 0xAA, 0x01, 0x02, 0x03, 0xFF};
    Serial2.write(txData, 6);

    Serial.println("  TX: 55 AA 01 02 03 FF");
    Serial.println("[UART] Done.\n");
}

// ===================== CAN =====================
void sendCAN() {
    Serial.println("[CAN] Initialising TWAI driver...");

    // NO_ACK mode: transmit without requiring an ACK from another node.
    // The FPGA sniffer does not send ACK, so normal mode would stall forever.
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        CAN_TX_PIN, CAN_TX_DUMMY_RX, TWAI_MODE_NO_ACK);
    twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        Serial.println("[CAN] Driver install failed.\n");
        return;
    }

    // The inverting level-shifter flips the polarity on the wire.
    // Compensate by inverting the TWAI TX signal in the ESP32 GPIO matrix
    // so that the physical pin matches what the HVD251 D pin expects.
    gpio_matrix_out(CAN_TX_PIN, TWAI_TX_IDX, /*invert=*/true, false);

    if (twai_start() != ESP_OK) {
        Serial.println("[CAN] Start failed.\n");
        twai_driver_uninstall();
        return;
    }

    // Build a standard 11-bit CAN frame
    twai_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.identifier       = 0x123;
    msg.extd             = 0;       // standard frame (11-bit ID)
    msg.rtr              = 0;
    msg.data_length_code = 4;
    msg.data[0]          = 0xDE;
    msg.data[1]          = 0xAD;
    msg.data[2]          = 0xBE;
    msg.data[3]          = 0xEF;

    Serial.println("[CAN] Sending frame...");
    esp_err_t result = twai_transmit(&msg, pdMS_TO_TICKS(1000));
    if (result == ESP_OK) {
        Serial.printf("  ID:  0x%03X\n", msg.identifier);
        Serial.printf("  DLC: %d\n", msg.data_length_code);
        Serial.print( "  Data: ");
        for (int i = 0; i < msg.data_length_code; i++) {
            Serial.printf("%02X ", msg.data[i]);
        }
        Serial.println();
    } else {
        Serial.printf("  TX failed (esp_err=0x%X)\n", result);
    }

    twai_stop();
    twai_driver_uninstall();
    Serial.println("[CAN] Done.\n");
}

// ===================== Menu =====================
void printMenu() {
    Serial.println();
    Serial.println("=== Byte-Level Protocol Tester ===");
    Serial.println("  u = UART  (6 bytes)");
    Serial.println("  s = SPI   (3 txns: 1, 3, 5 bytes)");
    Serial.println("  o = SPI Overflow  (34 txns: 8 bytes each)");
    Serial.println("  i = I2C   (write 4, read 4)");
    Serial.println("  c = CAN   (ID 0x123, 4 bytes: DE AD BE EF)");
    Serial.println("  a = All of the above");
    Serial.println("  h = This menu");
    Serial.println("==================================");
}

// ===================== Main =====================
void setup() {
    Serial.begin(115200);
    delay(1000);

    pinMode(LED_PIN, OUTPUT);
    pinMode(SPI_CS_PIN, OUTPUT);
    digitalWrite(SPI_CS_PIN, HIGH);

    Serial2.begin(UART2_BAUD, SERIAL_8N1, 16, 17);
    SPI.begin();
    Wire.begin();
    Wire.setClock(100000);

    Serial.println();
    Serial.println("===================================");
    Serial.println("  ESP32 MASTER - Byte-Level Test");
    Serial.println("===================================");
    printMenu();
}

void loop() {
    if (Serial.available()) {
        char cmd = Serial.read();
        if (cmd == '\n' || cmd == '\r') return;

        digitalWrite(LED_PIN, HIGH);

        switch (cmd) {
            case 'u': case 'U': sendUART(); break;
            case 's': case 'S': sendSPI();  break;
            case 'o': case 'O': sendSPIoverflow();  break;
            case 'i': case 'I': sendI2C();  break;
            case 'c': case 'C': sendCAN();  break;
            case 'a': case 'A':
                sendUART(); delay(200);
                sendSPI();  delay(200);
                sendI2C();  delay(200);
                sendCAN();
                break;
            case 'h': case 'H': printMenu(); break;
            default:
                Serial.printf("> Unknown '%c'. Press 'h' for help.\n", cmd);
                break;
        }

        digitalWrite(LED_PIN, LOW);
    }
}
