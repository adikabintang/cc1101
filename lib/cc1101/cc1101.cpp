#include "cc1101.h"

#define getGDO0state()  digitalRead(GDO0)
/*
*for EsP8266 based.
* see ~/.platformio/packages/framework-arduinoespressif8266
* add ARDUINO_
*/
#if defined (ARDUINO_ESP8266_NODEMCU) || defined (ARDUINO_ESP8266_ESP12) || defined (ESP8266)
	// Select (SPI) CC1101
	#define cc1101_Select()  SPI.begin(); digitalWrite(SS, LOW)
	// Deselect (SPI) CC1101
	#define   cc1101_Deselect()  SPI.end(); digitalWrite(SS, HIGH)
	// Wait until SPI MISO line goes low
	//#define wait_Miso()  delay(10)
	#define wait_Miso() while (digitalRead(MISO) > 0) delay(0) //to avoid wdt reset
	//while(digitalRead(PORT_SPI_MISO))
	// Wait until GDO0 line goes high
	#define wait_GDO0_high()  while(!getGDO0state()) delay(0)
	// Wait until GDO0 line goes low
	#define wait_GDO0_low()  while(getGDO0state()) delay(0)

#else //for other arduino boards
	// Select (SPI) CC1101
	#define cc1101_Select()  digitalWrite(SS, LOW)
	// Deselect (SPI) CC1101
	#define   cc1101_Deselect()  digitalWrite(SS, HIGH)
	// Wait until SPI MISO line goes low
	//#define wait_Miso()  delay(10)
	#define wait_Miso() while (digitalRead(MISO) > 0)
	//while(digitalRead(PORT_SPI_MISO))
	// Wait until GDO0 line goes high
	#define wait_GDO0_high()  while(!getGDO0state())
	// Wait until GDO0 line goes low
	#define wait_GDO0_low()  while(getGDO0state())
#endif

CC1101::CC1101() {

}

void CC1101::init() {
	DEBUG_CC1101("Init");
	pinMode(GDO0, INPUT);
	pinMode(GDO2, INPUT);

	//#if !defined (ARDUINO_ESP8266_NODEMCU) || !defined (ARDUINO_ESP8266_ESP12) || !defined (ESP8266)
		SPI.begin();
	//#endif

	reset();

	//uint8_t patable[8] = {0x6C,0x1C,0x06,0x3A,0x51,0x85,0xC8,0xC0};
	//uint8_t patable[8] = {0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0};
	DEBUG_CC1101("Setting Patable");
	//writeBurstReg(CC1101_PATABLE, patable, 8);
	writeReg(CC1101_PATABLE, 0x80); //5,8 dbm; 0x82
	DEBUG_CC1101("Init done");
}

void CC1101::cmdStrobe(uint8_t cmd) {
	cc1101_Select();
	wait_Miso(); //datasheet page 29
	SPI.transfer(cmd);
	cc1101_Deselect();
}

void CC1101::wakeUp() {
	cc1101_Select();
	wait_Miso();
	cc1101_Deselect();
}

uint8_t CC1101::readReg(uint8_t regAddr, uint8_t regType) {
	uint8_t readValue;

	cc1101_Select();
	wait_Miso();
	readValue = SPI.transfer(regAddr | regType);
	readValue =  SPI.transfer(0);
	cc1101_Deselect();

	return readValue;
}

void CC1101::writeReg(uint8_t regAddr, uint8_t value) {
	cc1101_Select();
	wait_Miso();
 	SPI.transfer(regAddr);
  SPI.transfer(value);
  cc1101_Deselect();
}

void CC1101::writeBurstReg(uint8_t regAddr, uint8_t* buffer, uint8_t len) {
	cc1101_Select();                      // Select CC1101
  wait_Miso();                          // Wait until MISO goes low
  SPI.transfer(regAddr | WRITE_BURST);                       // Send register address

  for(uint8_t i = 0; i < len; i++) {
    SPI.transfer(buffer[i]);
	}

  cc1101_Deselect();
}

void CC1101::setCCregs() {

	uint8_t panStamp[47] = {
		0x2E,         // GDO2 Output Pin Configuration
		0x2E,         // GDO1 Output Pin Configuration
		0x06,         // GDO0 Output Pin Configuration
		0x07,         // RX FIFO and TX FIFO Thresholds
		0xB5,         // Synchronization word, high byte
		0x47,         // Synchronization word, low byte
		0x3D,         // Packet Length
		0x04, //0x04: disble address check; //0x06,         // Packet Automation Control
		0x05,         // Packet Automation Control
		0xFF,         // Device Address
		0x00,         // Channel Number
		0x08,         // Frequency Synthesizer Control
		0x00,         // Frequency Synthesizer Control
		0x10,         // Frequency Control Word, High Byte
		0xA7,         // Frequency Control Word, Middle Byte
		0x62,         // Frequency Control Word, Low Byte
		0xC7,    			// Modem configuration. Speed = 4800 bps
		0x83,         // Modem Configuration
		0x93,         // Modem Configuration
		0x12, //gfsk //0x22,         // Modem Configuration
		0xF8,         // Modem Configuration
		0x35,         // Modem Deviation Setting
		0x07,         // Main Radio Control State Machine Configuration
		0x20,         // Main Radio Control State Machine Configuration
		0x18,         // Main Radio Control State Machine Configuration
		0x16,         // Frequency Offset Compensation Configuration
		0x6C,         // Bit Synchronization Configuration
		0x43,         // AGC Control
		0x40,         // AGC Control
		0x91,         // AGC Control
		0x87,         // High Byte Event0 Timeout
		0x6B,         // Low Byte Event0 Timeout
		0xFB,         // Wake On Radio Control
		0x56,         // Front End RX Configuration
		0x10,         // Front End TX Configuration
		0xE9,         // Frequency Synthesizer Calibration
		0x2A,         // Frequency Synthesizer Calibration
		0x00,         // Frequency Synthesizer Calibration
		0x1F,         // Frequency Synthesizer Calibration
		0x41,         // RC Oscillator Configuration
		0x00,         // RC Oscillator Configuration
		0x59,         // Frequency Synthesizer Calibration Control
		0x7F,         // Production Test
		0x3F,         // AGC Test
		0x81,         // Various Test Settings
		0x35,         // Various Test Settings
		0x09         // Various Test Settings
	};

	for (uint8_t i = 0; i < 47; i++) {
		//writeReg(i, registerSetting[i]);
		writeReg(i, panStamp[i]);
	}

	setRxState();

  //sendData(packet);
	//sendData(, 0)
}

void CC1101::reset() {
	DEBUG_CC1101("Reset");
	cc1101_Deselect();
  delayMicroseconds(5);
  cc1101_Select();
  delayMicroseconds(10);
  cc1101_Deselect();
	delayMicroseconds(41);
  cc1101_Select();

  wait_Miso();
	DEBUG_CC1101("...");
  SPI.transfer(CC1101_SRES);
	DEBUG_CC1101("udah transfer");
  wait_Miso();

  cc1101_Deselect();

	setCCregs();
	DEBUG_CC1101("Reset Done");
}

void CC1101::setDeviceAddress(uint32_t addr) {
	//writeReg(CC1101_ADDR, addr);
	myAddress = addr;
}

void CC1101::setChannel(uint8_t channel) {
	writeReg(CC1101_CHANNR, channel);
}

void CC1101::setCarrierFreq(uint8_t freq) {

}

void CC1101::setPowerDownState() {
	// Disable LNA on LD-board if any

	// Comming from RX state, we need to enter the IDLE state first
	cmdStrobe(CC1101_SIDLE);
  // Enter Power-down state
  cmdStrobe(CC1101_SPWD);
}

void CC1101::setTxState() {
	// Enable PA on LD-board if any
	uint8_t marcState = 0xFF;

	cmdStrobe(CC1101_STX);
	while (marcState != 0x01) {
		marcState = (readReg(CC1101_MARCSTATE, CC1101_STATUS_REGISTER) & 0x1F);
	}
}

void CC1101::setRxState() {
	uint8_t marcState = 0xFF;

	cmdStrobe(CC1101_SRX);
	while (marcState != 0x0D) {
		marcState = (readReg(CC1101_MARCSTATE, CC1101_STATUS_REGISTER) & 0x1F);
		if (marcState == 0x11) { //RX fifo overflow
			flushRxFifo();
		}
	}

	// Enable LNA on LD-board if any
}

void CC1101::seteTxPowerAmp() {

}

//datasheet halaman 33
void CC1101::flushTxFifo() {
	//A SFTX or SFRX command strobe can only be
	//issued in the IDLE, TXFIFO_UNDERFLOW, or
	//RXFIFO_OVERFLOW states
	cmdStrobe(CC1101_SFTX);
}

//datasheet halaman 33
void CC1101::flushRxFifo() {
	cmdStrobe(CC1101_SFRX);
}

void CC1101::setIdle() {
	uint8_t marcState = 0xFF;

	cmdStrobe(CC1101_SIDLE);
	while (marcState != 0x01) {
		marcState = (readReg(CC1101_MARCSTATE, CC1101_STATUS_REGISTER) & 0x1F);
	}
}

uint8_t CC1101::receiveData(struct CCPacket *packet) {
	//read RX fifo buffer
	Serial.print("read RX fifo buffer: ");
	uint8_t filledRXFifo = readReg(CC1101_RXBYTES, CC1101_STATUS_REGISTER);
	DEBUG_CC1101(filledRXFifo);

	if ((filledRXFifo & 0x7F) && !(filledRXFifo & 0x80)) {
		packet->payloadLength = readReg(CC1101_RXFIFO, READ_SINGLE) - 3; // -3 for dest source and message type
		packet->destinationAddress = readReg(CC1101_RXFIFO, READ_SINGLE);
		packet->sourceAddress = readReg(CC1101_RXFIFO, READ_SINGLE);
		packet->type = (messageType)readReg(CC1101_RXFIFO, READ_SINGLE);

		for (uint8_t i = 0; i < packet->payloadLength; i++) {
			packet->payload[i] = readReg(CC1101_RXFIFO, READ_SINGLE);
		}

		packet->lqi = readReg(CC1101_RXFIFO, READ_SINGLE);
		packet->rssi = readReg(CC1101_RXFIFO, READ_SINGLE);
	}
	else {
		packet->payloadLength = 0;
	}

	setIdle();
	flushRxFifo(); //baru
	setRxState();

	if (packet->payloadLength > 0) {
		Serial.print("packet type: ");
		Serial.println(packet->type);
		//if !broadcast, send ACK
		if (packet->destinationAddress != BROADCAST_ADDRESS && packet->type == DATA_PACKET) {
			Serial.println("Sending ack...");
			struct CCPacket ackPacket;
			ackPacket.destinationAddress = packet->sourceAddress;
			ackPacket.sourceAddress = readReg(CC1101_ADDR, CC1101_CONFIG_REGISTER);
			ackPacket.payload[0] = ACK_RESPONSE;
			ackPacket.payloadLength = 1;
			ackPacket.type = ACK_PACKET;
			sendData(&ackPacket);
		}
	}

	return packet->payloadLength;
}

//datasheet: 10.5 FIFO Access
bool CC1101::sendData(struct CCPacket *packet) {
	DEBUG_CC1101("------send awal--------");

	setRxState();
	//delayMicroseconds(500);

	if (packet->payloadLength > 0) {
		DEBUG_CC1101("Write to tx fifo buffer");
		writeReg(CC1101_TXFIFO, packet->payloadLength + 3); // + 2 for destination, ender address, message type
		writeReg(CC1101_TXFIFO, packet->destinationAddress);
		writeReg(CC1101_TXFIFO, packet->sourceAddress); //sender address
		writeReg(CC1101_TXFIFO, (uint8_t)packet->type);
		for (uint8_t i = 0; i < packet->payloadLength; i++) {
			writeReg(CC1101_TXFIFO, packet->payload[i]);
		}

		DEBUG_CC1101("Set to TX state");
		setTxState();

		uint8_t x = readReg(CC1101_TXBYTES, CC1101_STATUS_REGISTER) & 0x7F;
		while (x != 0) {
			x = readReg(CC1101_TXBYTES, CC1101_STATUS_REGISTER) & 0x7F;
		}

		DEBUG_CC1101("Set Idle");
		setIdle();
		DEBUG_CC1101("Flush TX Fifo");
		flushTxFifo();
		flushRxFifo();
		DEBUG_CC1101("Set to RX state");
		setRxState();

		if (packet->destinationAddress == BROADCAST_ADDRESS
			|| packet->type == ACK_PACKET) { // broadcast, then fire and forget
			DEBUG_CC1101("Broadcast/ack done");
			return true;
		}
		else { //wait for ACK
			unsigned long nowMs = millis();
			while (millis() - nowMs <= timeoutMs) {
				if (digitalRead(GDO0)) {
					wait_GDO0_low();
					struct CCPacket ackPacket;
					receiveData(&ackPacket);
					//setRxState();
					if (ackPacket.type == ACK_PACKET) {
						if (ackPacket.payload[0] == ACK_RESPONSE) {
							DEBUG_CC1101("ACK received");
							return true;
						}
					}

				}
			}
			DEBUG_CC1101("Timeout in waiting for ACK");
			return false;
		}
	}
	else {
		DEBUG_CC1101("packet length must not be <= 0");
		return false;
	}
}
