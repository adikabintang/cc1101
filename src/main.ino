#include <Arduino.h>
#include <cc1101.h>
#include <AmbroSimpleTCP.h>

CC1101 cc1101;
bool signalFlag = false;
uint8_t data[1] = {0};
uint8_t i = 0;
AmbroSimpleTCP ambroTCP(&cc1101);
AmbroTCPPacket ambroPacket;

void ISRSignalFlag() {
  signalFlag  = true;
}

struct CCPacket packet;

void setup() {
  Serial.begin(9600);
  delay(500);
  cc1101.init();
  cc1101.setCCregs();
  uint8_t addr[3] = {2, 1, 0};
  cc1101.setDeviceAddress(addr);

  attachInterrupt(digitalPinToInterrupt(2), ISRSignalFlag, FALLING);

  char s[16];
  for (int i = 0; i <= 46; i++) {
    sprintf(s, "0x%02X ", cc1101.readReg(i, CC1101_CONFIG_REGISTER));
    Serial.print(s);
    if ((i + 1) % 10 == 0 && i != 0)
      Serial.println();
    delay(10);
  }
  Serial.println();
}

void loop() {
  if (signalFlag) {
    if (ambroTCP.ambroReceiveData(&ambroPacket)) {
      Serial.print("data: ");
      Serial.println(ambroPacket.payload);
    }
    else {
      Serial.println("no data");
    }
    signalFlag = false;
  }
}
