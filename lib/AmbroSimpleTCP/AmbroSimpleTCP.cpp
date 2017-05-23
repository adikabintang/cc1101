#include "AmbroSimpleTCP.h"

/**
SYN: return N (from ACK)
*/
uint8_t AmbroSimpleTCP::ambroSend(uint8_t sender[], uint8_t destination[],
  messageType type, uint8_t seq, uint8_t data[53], uint8_t dataLen) {
  CCPacket packet;
  for (uint8_t i = 0; i < 3; i++) {
    packet.destinationAddress[i] = destination[i];
    packet.sourceAddress[i] = sender[i];
  }

  packet.type = type;
  packet.seq = seq;

  if (type != DATA) {
    packet.payloadLength = 1;
    packet.payload[0] = data[0]; // send the last 8 bit
    if (type == FIN) {
      packet.payload[0] = 0;
    }
  }
  else {
    packet.payloadLength = dataLen;
    for (uint8_t i = 0; i < dataLen; i++) {
      packet.payload[i] = data[i];
    }
  }

  if (phyCC1101->sendData(&packet)) {
    if (type != ACK) { //send, wait for ack
      unsigned long nowMs = millis();
      while (millis() - nowMs <= ambroTimeoutMs) {
        if (digitalRead(2)) {
          while (digitalRead(2));
          if (phyCC1101->receiveData(&packet)) {
            if (packet.type == ACK) {
              return packet.payload[0]; //return seq number N
            }
          }
        }
      }
      DEBUG_AMBROTCP("Timeout ACK waiting");
      return 0;
    }
    else { // send ack, wait for nothing
      DEBUG_AMBROTCP("ACK sent successfully");
      return 1;
    }

    //return packet.payload[0]; //return seq number N
  }
  else {
    #ifdef DEBUGGING_AMBROTCP
    Serial.print("Sent failed: ");
    Serial.println(type);
    #endif
    return 0;
  }
}

bool AmbroSimpleTCP::ambroSendData(AmbroTCPPacket *packet) {
  packet->payloadLength = (packet->payload).length();

  uint8_t seqNumber = 0;
  while (seqNumber == 0) {
    seqNumber = ambroSend(packet->sourceAddress,
      packet->destinationAddress, SYN);
  }

  unsigned long numberOfChunck = ((packet->payloadLength % 2 == 0
    && packet->payloadLength > 53) ?
    (packet->payloadLength / 53) : (packet->payloadLength / 53 + 1));

  uint8_t payloadChunk[53];
  uint8_t j;
  for (unsigned long i = 0; i < numberOfChunck; i++) {
    for (j = 0; (j < 53 && j < (packet->payloadLength % 53)); j++) {
      payloadChunk[j] = (uint8_t)packet->payload[j];
    }

    while (ambroSend(packet->sourceAddress, packet->destinationAddress,
      DATA, seqNumber++, payloadChunk, j) == 0);

  }
  //fin
  ambroSend(packet->sourceAddress,
    packet->destinationAddress, FIN,
    seqNumber);
  return true;
}

unsigned int AmbroSimpleTCP::ambroReceiveData(AmbroTCPPacket *packet) {
  CCPacket packetPhy; //kalo bisa satu aja, irit stack
  if (phyCC1101->receiveData(&packetPhy) == 0) {
    return 0;
  }
  else {
    /*
      TODO: oper yang ada di CCPAcket ke ambro packet. Pake class aja jangan struct,
      operator overloading
    */
    packet->payload = "";
    for (uint8_t i = 0; i < packetPhy.payloadLength; i++) {
      packet->payload += (char)packetPhy.payload[i];
    }
    //end of TODO
#ifdef DEBUGGING_AMBROTCP
    Serial.print("Packet phy type: ");
    Serial.println(packetPhy.type);
#endif
    if (packetPhy.type != ACK &&
      !(packetPhy.destinationAddress[0] == 0 && packetPhy.destinationAddress[1] == 0
      && packetPhy.destinationAddress[2] == 0)) { //sementara tang penting jalan dulu, send ack

      uint8_t NMillis[1] = {23};
      Serial.println("Sending ack...");
      delay(10); //fuck
      ambroSend(packetPhy.destinationAddress, packetPhy.sourceAddress,
        ACK, 1, NMillis, 1);
    }
    return 1;
  }
}
