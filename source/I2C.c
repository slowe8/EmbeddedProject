/*
 * I2C.c
 *
 *  Created on: Apr 21, 2023
 *      Author: seanm
 */

#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL46Z4.h"
#include "fsl_debug_console.h"
#include "Utilities.h"
#include "I2C.h"

unsigned char device_address = 0x29;


void initI2C() {

    // Enable Clock Gating for I2C module and Port
	SIM->SCGC4 |= (1<<6) | (1<<7);
	SIM->SCGC5 |= (1<<11);

    // Setup Pin Mode for I2C
	PORTC->PCR[9] &= ~(0x700); // Clear MUX bits
	PORTC->PCR[9] |= 0x200; // Set MUX for ALT2 (I2C0_SDA)

	PORTC->PCR[8] &= ~(0x700); // Clear MUX bits
	PORTC->PCR[8] |= 0x200; // Set MUX for ALT2 (I2C0_SCL)

    // Write 0 to all I2C registers
	I2C0->A1 = 0;
	I2C0->F = 0;
	I2C0->C1 = 0;
	I2C0->S = 0;
	I2C0->D = 0;
	I2C0->C2 = 0;
	I2C0->FLT = 0;
	I2C0->RA = 0;
	I2C0->SMB = 0;
	I2C0->A2 = 0;
	I2C0->SLTH = 0;
	I2C0->SLTL = 0;

    // Write 0x50 to FLT register (clears all bits)
	I2C0->FLT = 0x50;

    // Clear status flags
	I2C0->S = 0x12;

    // Set I2C Divider Register (Choose a clock frequency less than 100 KHz)
	I2C0->F |= 0x27;
	I2C0->F &= ~(0x3<<6);

    // Set bit to enable I2C Module
	I2C0->C1 |= (1<<7);
	return;
}

void clearStatusFlags() {

   // Clear STOPF and Undocumented STARTF bit 4 in Filter Register
	I2C0->FLT |= (1<<6); // Clear STOPF and STARTF
	I2C0->FLT |= (1<<4);

    // Clear ARBL and IICIF bits in Status Register
	I2C0->S |= (1<<4) | (1<<1);
	return;
}

void TCFWait() {

    // Wait for TCF bit to Set in Status Register
	while(!(I2C0->S & (1<<7))) {}
	return;
}

void IICIFWait() {

    // Wait for IICIF bit to Set in Status Register
	while(!(I2C0->S & (1<<1))) {}
	return;
}

void SendStart() {

    // Set MST and TX bits in Control 1 Register
	I2C0->C1 |= (1<<5) | (1<<4);
	return;
}

void RepeatStart() {

    // Set MST, TX and RSTA bits in Control 1 Register
	I2C0->C1 |= (1<<5) | (1<<4) | (1<<2);
    // Wait 6 cycles
	// try nops
	for(int i=0; i<6; i++) {
		__asm volatile ("nop");
	}
	return;
}

void SendStop() {

    // Clear MST, TX and TXAK bits in Control 1 Register
	I2C0->C1 &= ~(1<<5 | 1<<4 | 1<<3);
    // Wait for BUSY bit to go low in Status Register
	while(I2C0->S & (1<<5)) {}
	return;
}

void clearIICIF() {

    // Clear IICIF bit in Status Register
	I2C0->S |= (1<<1);
	return;
}

int RxAK() {

    // Return 1 if byte has been ACK'd. (See RXAK in Status Register)
	return ~(I2C0->S & (1<<0));
}

void WriteByte(unsigned char addr, unsigned char data) {

    clearStatusFlags();

    TCFWait();

    SendStart();


    // TODO: Write Device Address, R/W = 0 to Data Register
    I2C0->D = (device_address<<1);

    IICIFWait();


    if (!RxAK()){

        PRINTF("NO RESPONSE - Address");

        SendStop();

        return;

    }


    clearIICIF();


    // TODO: Write Register address to Data Register
    I2C0->D = addr | 1<<7;

    IICIFWait();

    if (!RxAK()){

        PRINTF("NO RESPONSE - Register");

        SendStop();

        return;

    }


    TCFWait();

    clearIICIF();


    // Write Data byte to Data Register
    I2C0->D = data;

    IICIFWait();

    if (!RxAK()){

        PRINTF("Incorrect ACK - Data");

    }


    clearIICIF();

    SendStop();

}

int ReadBlock(unsigned char addr, unsigned char data[], int len) {

    unsigned char dummy = 0;

    clearStatusFlags();

    TCFWait();

    SendStart();


    dummy++;  // Do something to suppress the warning.


    //TODO: Write Device Address, R/W = 0 to Data Register
    I2C0->D = (device_address<<1);

    IICIFWait();


    if (!RxAK()){

        PRINTF("NO RESPONSE - Address");

        SendStop();

        return 0;

    }


    clearIICIF();


    // Write Register address to Data Register
    I2C0->D = addr | 1<<7;

    IICIFWait();

    if (!RxAK()){

        PRINTF("NO RESPONSE - Register");

        SendStop();

        return 0;

    }


    clearIICIF();

    RepeatStart();


    // Write device address again, R/W = 1 to Data Register
    I2C0->D = (device_address<<1) | (1<<0);

    IICIFWait();

    if (!RxAK()){

        PRINTF("NO RESPONSE - Restart");

        SendStop();

        return 0;

    }


    TCFWait();

    clearIICIF();


    // Switch to RX by clearing TX and TXAK bits in Control 1 register
    I2C0->C1 &= ~(1<<4 | 1<<3);

    if(len==1){

        // Set TXAK to NACK in Control 1 - No more data!
    	I2C0->C1 |= (1<<3);

    }


   dummy = I2C0->D; // Dummy Read


    for(int index=0; index<len; index++){

        IICIFWait();

        clearIICIF();


        if(index == len-2){

            // Set TXAK to NACK in Control 1 - No more data!
        	I2C0->C1 |= (1<<3);
        }


       if(index == len-1){

            SendStop();

        }

        // Read Byte from Data Register into Array
       data[index] = I2C0->D;
    }
    return 1;
}

int readColorSensor(unsigned int colors[]) {
	initI2C();
	unsigned char readIDAddr = 0x16;
	unsigned char data[6] = {0};
	WriteByte(0x00, 3);
	delay_ms(3);
	int success = ReadBlock(readIDAddr, data, 6);
	if(success == 1) {
		colors[0] = (data[1]<<8) | data[0];
		//PRINTF("RED: %d\n", red);
		colors[1] = (data[3]<<8) | data[2];
		//PRINTF("GREEN: %d\n", green);
		colors[2] = (data[5]<<8) | data[4];
		//PRINTF("BLUE: %d\n", blue);
		return 1;
	}
	readDone = true;
	return 0;
}
