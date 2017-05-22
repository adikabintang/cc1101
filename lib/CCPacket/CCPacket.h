#ifndef CCPACKET_H
#define CCPACKET_H

#include <Arduino.h>

enum messageType {
  ACK = 0,
  DATA,
  SYN,
  RST,
  FIN
};

class CCPacket_ {
public:
  uint8_t destinationAddress[3];
  uint8_t sourceAddress[3];
  uint8_t payloadLength;
  uint8_t payload[63];
  uint8_t lqi;
  uint8_t rssi;
  messageType type;
  uint8_t seq;
};

#endif
