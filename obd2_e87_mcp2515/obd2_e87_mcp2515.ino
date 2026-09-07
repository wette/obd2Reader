#include <SPI.h>
#include <mcp2515.h>

// Typical MCP2515 wiring for Uno:
// CS  -> D10
// SCK -> D13
// MOSI-> D11
// MISO-> D12
// INT -> D2 (optional, not required by this sketch)

static const uint8_t MCP2515_CS_PIN = 10;
static const uint32_t CAN_REQUEST_ID = 0x7DF;
static const uint16_t RESPONSE_TIMEOUT_MS = 1500;

MCP2515 mcp2515(MCP2515_CS_PIN);

bool sendObdRequest(uint8_t mode);
bool receiveObdPayload(uint8_t expectedMode, uint8_t *buffer, size_t &payloadLen, size_t maxLen);
void readAndPrintDTCs();
void clearDTCs();
void flushCanRx();

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  SPI.begin();
  mcp2515.reset();

  // Most MCP2515 breakout boards use 8 MHz crystal.
  // If your board is 16 MHz use: CAN_500KBPS, MCP_16MHZ
  if (mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ) != MCP2515::ERROR_OK) {
    Serial.println("ERR MCP2515 bitrate setup failed");
    return;
  }

  if (mcp2515.setNormalMode() != MCP2515::ERROR_OK) {
    Serial.println("ERR MCP2515 normal mode failed");
    return;
  }

  Serial.println("READY");
  Serial.println("INFO Commands: READ, CLEAR");

  // Requirement: report current errors via serial.
  readAndPrintDTCs();
}

void loop() {
  if (!Serial.available()) {
    return;
  }

  String command = Serial.readStringUntil('\n');
  command.trim();
  command.toUpperCase();

  if (command == "READ") {
    readAndPrintDTCs();
  } else if (command == "CLEAR") {
    clearDTCs();
  } else if (command.length() > 0) {
    Serial.print("ERR Unknown command: ");
    Serial.println(command);
  }
}

void flushCanRx() {
  struct can_frame frame;
  while (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
    // drain
  }
}

bool sendObdRequest(uint8_t mode) {
  struct can_frame txFrame;
  txFrame.can_id = CAN_REQUEST_ID;
  txFrame.can_dlc = 8;
  txFrame.data[0] = 0x01;
  txFrame.data[1] = mode;
  txFrame.data[2] = 0x00;
  txFrame.data[3] = 0x00;
  txFrame.data[4] = 0x00;
  txFrame.data[5] = 0x00;
  txFrame.data[6] = 0x00;
  txFrame.data[7] = 0x00;

  return mcp2515.sendMessage(&txFrame) == MCP2515::ERROR_OK;
}

bool receiveObdPayload(uint8_t expectedMode, uint8_t *buffer, size_t &payloadLen, size_t maxLen) {
  payloadLen = 0;
  unsigned long started = millis();

  while (millis() - started < RESPONSE_TIMEOUT_MS) {
    struct can_frame frame;
    if (mcp2515.readMessage(&frame) != MCP2515::ERROR_OK) {
      delay(2);
      continue;
    }

    if (frame.can_id < 0x7E8 || frame.can_id > 0x7EF || frame.can_dlc < 3) {
      continue;
    }

    uint8_t pciType = frame.data[0] >> 4;

    // Single frame (fits in one CAN frame)
    if (pciType == 0x0) {
      uint8_t sfLen = frame.data[0] & 0x0F;
      if (sfLen < 2 || frame.data[1] != (uint8_t)(0x40 + expectedMode)) {
        continue;
      }

      size_t copyLen = sfLen;
      if (copyLen > frame.can_dlc - 1) {
        copyLen = frame.can_dlc - 1;
      }
      if (copyLen > maxLen) {
        copyLen = maxLen;
      }

      for (size_t i = 0; i < copyLen; ++i) {
        buffer[i] = frame.data[1 + i];
      }
      payloadLen = copyLen;
      return true;
    }

    // First frame (ISO-TP multi-frame)
    if (pciType == 0x1) {
      uint16_t totalLen = ((frame.data[0] & 0x0F) << 8) | frame.data[1];
      if (totalLen < 2 || frame.data[2] != (uint8_t)(0x40 + expectedMode)) {
        continue;
      }

      size_t copied = 0;
      for (uint8_t i = 2; i < frame.can_dlc && copied < totalLen && copied < maxLen; ++i) {
        buffer[copied++] = frame.data[i];
      }

      // Send Flow Control (Continue To Send)
      struct can_frame flow;
      flow.can_id = CAN_REQUEST_ID;
      flow.can_dlc = 8;
      flow.data[0] = 0x30;
      flow.data[1] = 0x00;
      flow.data[2] = 0x00;
      flow.data[3] = 0x00;
      flow.data[4] = 0x00;
      flow.data[5] = 0x00;
      flow.data[6] = 0x00;
      flow.data[7] = 0x00;
      mcp2515.sendMessage(&flow);

      uint8_t expectedSeq = 1;
      unsigned long segmentStart = millis();
      while (copied < totalLen && millis() - segmentStart < RESPONSE_TIMEOUT_MS) {
        struct can_frame cf;
        if (mcp2515.readMessage(&cf) != MCP2515::ERROR_OK) {
          delay(1);
          continue;
        }

        if (cf.can_id < 0x7E8 || cf.can_id > 0x7EF || cf.can_dlc < 2) {
          continue;
        }

        uint8_t cfType = cf.data[0] >> 4;
        uint8_t seq = cf.data[0] & 0x0F;
        if (cfType != 0x2 || seq != (expectedSeq & 0x0F)) {
          continue;
        }

        for (uint8_t i = 1; i < cf.can_dlc && copied < totalLen && copied < maxLen; ++i) {
          buffer[copied++] = cf.data[i];
        }
        expectedSeq++;
      }

      payloadLen = copied;
      return copied >= 2;
    }
  }

  return false;
}

void readAndPrintDTCs() {
  flushCanRx();

  if (!sendObdRequest(0x03)) {
    Serial.println("ERR CAN send failure (mode 03)");
    return;
  }

  uint8_t payload[96];
  size_t payloadLen = 0;
  if (!receiveObdPayload(0x03, payload, payloadLen, sizeof(payload))) {
    Serial.println("ERR No OBD response for mode 03");
    return;
  }

  if (payloadLen < 2 || payload[0] != 0x43) {
    Serial.println("ERR Unexpected OBD payload for mode 03");
    return;
  }

  size_t dtcCount = 0;
  for (size_t i = 1; i + 1 < payloadLen; i += 2) {
    uint8_t a = payload[i];
    uint8_t b = payload[i + 1];

    if (a == 0x00 && b == 0x00) {
      continue;
    }

    char first = 'P';
    switch ((a & 0xC0) >> 6) {
      case 0:
        first = 'P';
        break;
      case 1:
        first = 'C';
        break;
      case 2:
        first = 'B';
        break;
      case 3:
        first = 'U';
        break;
    }

    uint8_t second = (a & 0x30) >> 4;
    uint8_t third = a & 0x0F;
    uint8_t fourth = (b & 0xF0) >> 4;
    uint8_t fifth = b & 0x0F;

    Serial.print("DTC ");
    Serial.print(first);
    Serial.print(second);
    Serial.print(third, HEX);
    Serial.print(fourth, HEX);
    Serial.println(fifth, HEX);

    dtcCount++;
  }

  Serial.print("END ");
  Serial.println(dtcCount);
}

void clearDTCs() {
  flushCanRx();

  if (!sendObdRequest(0x04)) {
    Serial.println("CLEAR FAIL CAN send failure (mode 04)");
    return;
  }

  uint8_t payload[16];
  size_t payloadLen = 0;
  if (!receiveObdPayload(0x04, payload, payloadLen, sizeof(payload))) {
    Serial.println("CLEAR FAIL No OBD response for mode 04");
    return;
  }

  if (payloadLen >= 1 && payload[0] == 0x44) {
    Serial.println("CLEAR OK");
  } else {
    Serial.println("CLEAR FAIL Unexpected OBD payload for mode 04");
  }
}
