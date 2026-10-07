#include "Stm32SerialCompat.h"
/*
 * LivoCommunication.h
 *
 *  Created on: Mar 28, 2024
 *      Author: Faisal
 */

#ifndef LIVOCOMMUNICATION_H_
#define LIVOCOMMUNICATION_H_

#include "ByteArrayHandler.h"
#include "Constants.h"
#include "CommandInbox.h"

class LivoCommunication {

private:
    struct RxLine {
        byte data[ARRAY_MAXSIZE];
        size_t used = 0;
        bool overflow = false;
    } usbLine, uartLine;
    CommandInbox<16, ARRAY_MAXSIZE> inbox;
    uint32_t oversizedLines = 0;
    uint32_t deferredQueries = 0;
    void receiveAscii(byte value, RxLine& line);
    bool inputBytesAvailable;
	long startTimeCommunication = 0;

public:
	
    ByteArrayHandler inputFramePacket;
	LivoCommunication();

	void init();
	byte calculateCheckSum8bit(const byte dataArray[], int length);

	void serialEvent_();
	void serialEventStream(LivoHardwareSerial& serial);
	bool getinputBytesAvailable();
	void setinputBytesAvailable(bool input);
	void process2024Byte(byte inByte);
	void process2019Byte(byte inByte);
	bool isProtocol2024();

	virtual ~LivoCommunication();
	long getStartTimeCommunication();
	void setStartTimeCommunication(long value);
};

extern LivoCommunication livoCommunication;

#endif /* LIVOCOMMUNICATION_H_ */
