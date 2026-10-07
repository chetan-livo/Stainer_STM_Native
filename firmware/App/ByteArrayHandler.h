/*
 * ByteArrayHandler.h
 *
 *  Created on: Aug 9, 2024
 *      Author: Faisal
 */

#ifndef BYTEARRAYHANDLER_H_
#define BYTEARRAYHANDLER_H_

#include "Constants.h"

class ByteArrayHandler {

private:

	int array_size;

public:

	unsigned char inputBytes[ARRAY_MAXSIZE];

	void setBytes(const unsigned char *newBytes, int size);
	const unsigned char* getBytes() const;
	unsigned char getByte(int index) const;
	void setByte(int index, unsigned char value);
	int getMaxSize() const;

	int getSize();
	void setSize(int size);

	ByteArrayHandler();
	virtual ~ByteArrayHandler();
};

#endif /* BYTEARRAYHANDLER_H_ */
