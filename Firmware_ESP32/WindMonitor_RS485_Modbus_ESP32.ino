#include <ModbusMaster.h>

// ====== Object and Pin Configurations ======
ModbusMaster node;

#define RE_DE 4      // RS485 module DE/RE pin
#define TXD2 17      // ESP32 UART2 TX
#define RXD2 16      // ESP32 UART2 RX

// Direction control before/after RS485 transmission
void preTransmission()  { digitalWrite(RE_DE, HIGH); }
void postTransmission() { digitalWrite(RE_DE, LOW); }

void setup() {
  Serial.begin(115200);  // Communication with Raspberry Pi
  Serial2.begin(9600, SERIAL_8E1, RXD2, TXD2);  // Communication with Anemometer: 9600, 8E1
  node.begin(1, Serial2); // Modbus ID = 1

  pinMode(RE_DE, OUTPUT);
  digitalWrite(RE_DE, LOW); // Default to receive mode

  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  delay(1000);
  Serial.println("✅ ESP32 Anemometer initialized, starting data transmission...");
}

void loop() {
  float spd = NAN;
  float dir = NAN;

  // ====== Read Wind Direction (Address 0x0001, 1 Register) ======
  uint8_t result_dir = node.readHoldingRegisters(0x0001, 1);
  if (result_dir == node.ku8MBSuccess) {
    dir = node.getResponseBuffer(0);
  }

  // ====== Read Wind Speed (Address 0x0002, 2 Registers) ======
  uint8_t result_spd = node.readHoldingRegisters(0x0002, 2);
  if (result_spd == node.ku8MBSuccess) {
    uint16_t high_word = node.getResponseBuffer(0);
    uint16_t low_word  = node.getResponseBuffer(1);

    // Correct order: Low word first
    union {
      uint32_t i;
      float f;
    } u;
    u.i = ((uint32_t)low_word << 16) | high_word;
    spd = u.f;
  }

  // ====== Error Handling and JSON Output ======
  if (!isnan(spd) && !isnan(dir)) {
    Serial.printf("{\"spd\":%.2f,\"dir\":%.1f}\n", spd, dir);  // Standard JSON format
  } else {
    Serial.println("{\"spd\":0,\"dir\":0}");  // Return 0 if reading fails
  }

  delay(1000);  // Output once per second
}