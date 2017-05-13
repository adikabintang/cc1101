#include "cc1101.h"

/*
*for EsP8266 based.
* see ~/.platformio/packages/framework-arduinoespressif8266
* add ARDUINO_
*/
#if defined (ARDUINO_ESP8266_NODEMCU) || defined (ARDUINO_ESP8266_ESP12) || defined (ESP8266)
	// Select (SPI) CC1101
	#define cc1101_Select()  SPI.begin()
	// Deselect (SPI) CC1101
	#define   cc1101_Deselect()  SPI.end()
	// Wait until SPI MISO line goes low
	//#define wait_Miso()  delay(10)
	#define wait_Miso() while (digitalRead(MISO) > 0) delay(0) //to avoid wdt reset
	//while(digitalRead(PORT_SPI_MISO))
	// Get GDO0 pin state
	#define getGDO0state()  digitalRead(PORT_GDO0)
	// Wait until GDO0 line goes high
	#define wait_GDO0_high()  while(!getGDO0state()) {delay(0);}
	// Wait until GDO0 line goes low
	#define wait_GDO0_low()  while(getGDO0state()) {delay(0);}

#else //for other arduino boards
	// Select (SPI) CC1101
	#define cc1101_Select()  digitalWrite(SS, LOW)
	// Deselect (SPI) CC1101
	#define   cc1101_Deselect()  digitalWrite(SS, HIGH)
	// Wait until SPI MISO line goes low
	//#define wait_Miso()  delay(10)
	#define wait_Miso() while (digitalRead(MISO) > 0)
	//while(digitalRead(PORT_SPI_MISO))
	// Get GDO0 pin state
	#define getGDO0state()  digitalRead(PORT_GDO0)
	// Wait until GDO0 line goes high
	#define wait_GDO0_high()  while(!getGDO0state())
	// Wait until GDO0 line goes low
	#define wait_GDO0_low()  while(getGDO0state())
#endif

CC1101::CC1101() {

}

CC1101::CC1101(uint8_t GDO0, uint8_t GDO2) {
	pinMode(GDO0, INPUT);
	pinMode(GDO2, INPUT);
}

void CC1101::cmdStrobe(uint8_t cmd) {
	cc1101_Select();
	wait_Miso();
	spi.transfer(cmd);
	cc1101_Deselect();  
}

void CC1101::wakeUp(void) {
	cc1101_Select();
	wait_Miso();
	cc1101_Deselect();  
}

uint8_t CC1101::readReg(uint8_t regAddr, uint8_t regType) {
	uint8_t readValue;

	cc1101_Select();
	wait_Miso();
	spi.transfer(regAddr | regType);
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

void CC1101::setCCregs(void) {
	uint8_t registerSetting[47]; //ntar isi nih

	for (uint8_t i = 0; i < 47; i++) {
		writeReg(i, registerSetting[i]);
	}

	//send empty packet
	CCPACKET packet;
  	packet.length = 0;
  	sendData(packet);
}

void CC1101::reset(void) {
	cc1101_Deselect();                    // Deselect CC1101
  	delayMicroseconds(5);
  	cc1101_Select();                      // Select CC1101
  	delayMicroseconds(10);
  	cc1101_Deselect();                    // Deselect CC1101
	delayMicroseconds(41);
  	cc1101_Select();                      // Select CC1101

  	wait_Miso();                          // Wait until MISO goes low
  	SPI.transfer(CC1101_SRES);                // Send reset command strobe
  	wait_Miso();                          // Wait until MISO goes low

  	cc1101_Deselect();                    // Deselect CC1101

	setCCregs();                          // Reconfigure CC1101
}