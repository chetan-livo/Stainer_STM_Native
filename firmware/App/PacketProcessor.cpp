/*
* PacketProcessor.cpp
*
* Created on: Apr 10, 2024
* Author: Faisal
*/

#include "PacketProcessor.h"

// #define SERIALPRINTPacketProcessor

long startTimePacketProcessor = micros();

PacketProcessor::PacketProcessor(LivoCommunication &comm) : livoCommunication(comm) {
}

PacketProcessor::~PacketProcessor() {
	// Destructor
}

void PacketProcessor::processInputFramePacket() {
	// Exit if livoCommunication.getinputBytesAvailable() is false
	if (!livoCommunication.getinputBytesAvailable()){
		return;
	}

	// Check Protocol
	if (livoCommunication.isProtocol2024()) {

	} else {
		if (protocol2019Handler.parsePacket(livoCommunication.inputFramePacket)) {
			protocol2019Handler.process2019Packet();
		} else {
			// Serial.println("PacketProcessor: Invalid Packet");
		}
	}
	
	livoCommunication.setinputBytesAvailable(false);
}

void PacketProcessor::setup() {
	execution2019Handler.setup();
}

void PacketProcessor::loop() {
	execution2019Handler.loop();
	processInputFramePacket();
}
