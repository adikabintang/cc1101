#ifndef AMBROSIMPLETCP_H
#define AMBROSIMPLETCP_H

#include <Arduino.h>
#include <cc1101.h>

#ifdef DEBUGGING_AMBROTCP
  #define DEBUG_AMBROTCP(...) Serial.println( __VA_ARGS__ )
#endif

#ifndef DEBUGGING_AMBROTCP
  #define DEBUG_AMBROTCP(...)
#endif

struct AmbroTCPPacket {
  uint8_t destinationAddress[3];
  uint8_t sourceAddress[3];
  String payload;
  unsigned int payloadLength;
};

class AmbroSimpleTCP {
private:
  uint16_t ambroTimeoutMs = 5000;
  CC1101 *phyCC1101;
  uint8_t ambroSendACK(uint8_t sender[], uint8_t destination[]);
  uint8_t ambroSend(uint8_t sender[], uint8_t destination[],
    messageType type, uint8_t seq = 0, uint8_t data[53] = NULL, uint8_t dataLen = 0);
public:
  AmbroSimpleTCP(CC1101 *cc1101) { this->phyCC1101 = cc1101; }
  bool ambroSendData(AmbroTCPPacket *packet);
  bool ambroSendData(uint8_t data, uint16_t length);
  unsigned int ambroReceiveData(AmbroTCPPacket *packet);
};

#endif
