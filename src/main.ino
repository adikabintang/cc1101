#include <Arduino.h>
#include <cc1101.h>

CC1101 cc1101;
bool signalFlag = false;

void ISRSignalFlag() {
  signalFlag  = true;
}

void setup() {
  Serial.begin(9600);
  delay(3000);
  Serial.println("eek");
  cc1101.init();
  cc1101.setCCregs();
  Serial.print("CC1101_PARTNUM "); //cc1101=0
  Serial.println(cc1101.readReg(CC1101_PARTNUM, CC1101_STATUS_REGISTER));
  Serial.print("CC1101_VERSION "); //cc1101=4
  Serial.println(cc1101.readReg(CC1101_VERSION, CC1101_STATUS_REGISTER));
  Serial.print("CC1101_MARCSTATE ");
  Serial.println(cc1101.readReg(CC1101_MARCSTATE, CC1101_STATUS_REGISTER) & 0x1f);

  attachInterrupt(digitalPinToInterrupt(2), ISRSignalFlag, FALLING);

  char s[16];
  for (int i = 0; i <= 46; i++) {
    sprintf(s, "0x%02X ", cc1101.readReg(i, CC1101_CONFIG_REGISTER));
    Serial.print(s);
    if ((i + 1) % 10 == 0 && i != 0)
      Serial.println();
  }
  Serial.println();
}

uint8_t data[1] = {0};

void loop() {
  //char data[2] = {8, 9};

  cc1101.sendData(0, data, 1);
  data[0]++;
  delay(2000);
}
