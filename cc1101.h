/**
 * @brief [brief description]
 * @details [long description]
 * 
 * @param l [description]
 * @return [description]
 */
#ifndef CC1101_H
#define CC1101_H

#ifdef DEBUGGING_CC1101
  #define DEBUG_CC1101(...) Serial.println( __VA_ARGS__ )
#endif

#ifndef DEBUGGING_CC1101
  #define DEBUG_CC1101(...)
#endif

class CC1101 {
private:
	void begin();
public:
	CC1101();
	CC1101(uint8_t GDO0, uint8_t GDO2);
	void cmdStrobe(uint8_t cmd);
	void wakeUp(void);
	uint8_t readReg(uint8_t regAddr, uint8_t regType);
	void writeReg(uint8_t regAddr, uint8_t value);
	void setCCregs(void);
	void reset(void);
}

#endif