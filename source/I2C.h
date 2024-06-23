/*
 * I2C.h
 *
 *  Created on: Apr 21, 2023
 *      Author: seanm
 */

#ifndef I2C_H_
#define I2C_H_

static bool readDone;

void initI2C();

void clearStatusFlag();

void TCFWait();

void IICIFWait();

void SendStart();

void RepeatStart();

void SendStop();

void clearIICIF();

int RxAK();

void WriteByte(unsigned char addr, unsigned char data);

int ReadBlock(unsigned char addr, unsigned char data[], int length);

int readColorSensor(unsigned int colors[]);

#endif /* I2C_H_ */
