#include <Wire.h>

static const int SDA_PIN = 21;
static const int SCL_PIN = 47;
static const uint8_t PCA_ADDR = 0x40;

static const uint8_t REG_MODE1 = 0x00;
static const uint8_t REG_MODE2 = 0x01;
static const uint8_t REG_PRESCALE = 0xFE;
static const uint8_t REG_LED0_ON_L = 0x06;

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)PCA_ADDR, 1);
  return Wire.available() ? Wire.read() : 0xFF;
}

void pcaInit50Hz() {
  writeReg(REG_MODE1, 0x10);
  delay(5);
  writeReg(REG_PRESCALE, 121);
  writeReg(REG_MODE1, 0x20);
  delay(5);
  writeReg(REG_MODE2, 0x04);
}

void ensureAwake() {
  uint8_t mode1 = readReg(REG_MODE1);
  if (mode1 == 0xFF || (mode1 & 0x10) != 0) {
    if (mode1 == 0xFF) {
      Wire.end();
      Wire.begin(SDA_PIN, SCL_PIN);
      Wire.setClock(400000);
      Wire.setTimeOut(50);
    }
    pcaInit50Hz();
  }
}

uint8_t setServoAngle(uint8_t ch, int angle) {
  if (ch > 15) return 5;
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;

  ensureAwake();
  long pulseUs = map(angle, 0, 180, 500, 2500);
  uint16_t ticks = (uint16_t)((pulseUs * 4096L) / 20000L);

  uint8_t reg = REG_LED0_ON_L + 4 * ch;
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(reg);
  Wire.write(0);
  Wire.write(0);
  Wire.write(ticks & 0xFF);
  Wire.write((ticks >> 8) & 0xFF);
  return Wire.endTransmission();
}

uint8_t setServoOff(uint8_t ch) {
  if (ch > 15) return 5;
  ensureAwake();
  uint8_t reg = REG_LED0_ON_L + 4 * ch;
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(reg);
  Wire.write(0);
  Wire.write(0);
  Wire.write(0);
  Wire.write(0x10); // Full OFF bit
  return Wire.endTransmission();
}

void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line == "PING") {
    ensureAwake();
    Serial.printf("OK PONG MODE1=0x%02X\n", readReg(REG_MODE1));
    return;
  }

  if (line.startsWith("S ")) {
    int ch = -1, angle = -1;
    if (sscanf(line.c_str(), "S %d %d", &ch, &angle) == 2 && ch >= 0 && ch <= 15) {
      uint8_t err = setServoAngle((uint8_t)ch, angle);
      Serial.printf("OK S %d %d i2c=%u\n", ch, constrain(angle, 0, 180), err);
    } else {
      Serial.println("ERR BAD_S");
    }
    return;
  }

  if (line.startsWith("A ")) {
    int angle = -1;
    if (sscanf(line.c_str(), "A %d", &angle) == 1) {
      uint8_t lastErr = 0;
      for (uint8_t ch = 0; ch < 16; ch++) {
        uint8_t err = setServoAngle(ch, angle);
        if (err != 0) lastErr = err;
      }
      Serial.printf("OK A %d i2c=%u\n", constrain(angle, 0, 180), lastErr);
    } else {
      Serial.println("ERR BAD_A");
    }
    return;
  }

  if (line == "OFF") {
    uint8_t lastErr = 0;
    for (uint8_t ch = 0; ch < 16; ch++) {
      uint8_t err = setServoOff(ch);
      if (err != 0) lastErr = err;
    }
    Serial.printf("OK OFF ALL i2c=%u\n", lastErr);
    return;
  }

  if (line.startsWith("OFF ")) {
    int ch = -1;
    if (sscanf(line.c_str(), "OFF %d", &ch) == 1 && ch >= 0 && ch <= 15) {
      uint8_t err = setServoOff((uint8_t)ch);
      Serial.printf("OK OFF %d i2c=%u\n", ch, err);
    } else {
      Serial.println("ERR BAD_OFF");
    }
    return;
  }

  Serial.println("ERR UNKNOWN");
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  Wire.setTimeOut(50);
  pcaInit50Hz();
  Serial.printf("READY MODE1=0x%02X PRESCALE=%u\n", readReg(REG_MODE1), readReg(REG_PRESCALE));
}

void loop() {
  static String buf;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (buf.length() > 0) {
        handleLine(buf);
        buf = "";
      }
    } else if (buf.length() < 64) {
      buf += c;
    }
  }
}
