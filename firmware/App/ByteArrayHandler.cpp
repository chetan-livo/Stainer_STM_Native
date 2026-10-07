/*
 * ByteArrayHandler.cpp
 *
 *  Created on: Aug 9, 2024
 *      Author: Faisal
 */

#include "ByteArrayHandler.h"

#include <string.h> // for memcpy

void ByteArrayHandler::setBytes(const unsigned char* newBytes, int size) {
    int copySize = (size < ARRAY_MAXSIZE) ? size : ARRAY_MAXSIZE;
    memcpy(inputBytes, newBytes, copySize);
}

const unsigned char* ByteArrayHandler::getBytes() const {
    return inputBytes;
}

unsigned char ByteArrayHandler::getByte(int index) const {
    if (index >= 0 && index < ARRAY_MAXSIZE) {
        return inputBytes[index];
    }
    return 0; // Return 0 for out-of-bounds access
}

void ByteArrayHandler::setByte(int index, unsigned char value) {
    if (index >= 0 && index < ARRAY_MAXSIZE) {
        inputBytes[index] = value;
    }
}

int ByteArrayHandler::getSize() {
    return array_size;
}


void ByteArrayHandler::setSize(int size) {
    array_size = size;
}

int ByteArrayHandler::getMaxSize() const {
    return ARRAY_MAXSIZE;
}


ByteArrayHandler::ByteArrayHandler() {
	array_size = ARRAY_MAXSIZE;
	// TODO Auto-generated constructor stub

}

ByteArrayHandler::~ByteArrayHandler() {
	// TODO Auto-generated destructor stub
}

