#include <Arduino.h>
#include <cc1101.h>

CC1101 cc1101;
bool signalFlag = false;

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
    delay(10);
  }
  Serial.println();
}

uint8_t data[1] = {0};
uint8_t i = 0;

void loop() {
  //char data[2] = {8, 9};
  //packet.destinationAddress = BROADCAST_ADDRESS;
  //packet.destinationAddress = 0x03;
  Serial.println("Sending...");
  Serial.print("destination: ");
  for (uint8_t i = 0; i < 3; i++) {
    //packet.destinationAddress[i] = i;
    packet.destinationAddress[i] = 0;
    Serial.print(packet.destinationAddress[i]);
  }
  
  Serial.println();
  packet.payload[0] = i++;
  //packet.payloadLength = sizeof(packet.payload) / sizeof(packet.payload[0]);
  packet.payloadLength = 1;
  packet.type = DATA;
  Serial.print("from: ");
  uint8_t addr[3] = {2, 1, 0};
  for (uint8_t i = 0; i < 3; i++) {
    packet.sourceAddress[i] = addr[i];
    Serial.print(packet.sourceAddress[i]);
  }
  Serial.println();
  //packet.sourceAddress = cc1101.readReg(CC1101_ADDR, CC1101_CONFIG_REGISTER);
  //cc1101.sendData(0, data, 1);
  if (cc1101.sendData(&packet)) {
    Serial.println("Data sent successfully");
  }
  delay(1500);
}
