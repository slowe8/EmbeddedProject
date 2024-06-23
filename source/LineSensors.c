/*
 * LineSensors.c
 *
 *  Created on: Apr 26, 2023
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

unsigned short cal_v = 0;
unsigned char light_val = 0;

void init_LineSensors() {
	SIM->SCGC5 |= (1<<13); // Enable PORTE
	SIM->SCGC6 |= (1<<27); // Enable ADC

	// Setup ADC Clock ( < 4 MHz)
	ADC0->CFG1 = 0; // Default everything.

	// Analog Calibrate
	ADC0->SC3 = 0x07; // Enable Maximum Hardware Averaging
	ADC0->SC3 |= 0x80; // Start Calibration

	// Wait for Calibration to Complete (either COCO or CALF)
	while(!(ADC0->SC1[0] & 0x80)){ }

	// Calibration Complete, write calibration registers.
	cal_v = ADC0->CLP0 + ADC0->CLP1 + ADC0->CLP2 + ADC0->CLP3 + ADC0->CLP4 +
	ADC0->CLPS;
	cal_v = cal_v >> 1 | 0x8000;
	ADC0->PG = cal_v;
	cal_v = 0;
	cal_v = ADC0->CLM0 + ADC0->CLM1 + ADC0->CLM2 + ADC0->CLM3 + ADC0->CLM4 +
	ADC0->CLMS;
	cal_v = cal_v >> 1 | 0x8000;
	ADC0->MG = cal_v;
	ADC0->SC3 = 0; // Turn off Hardware Averaging
}

unsigned char readLeftSensor() {
	ADC0->SC1[0] = 0x01; // Set Channel, starts conversion.
	while(!(ADC0->SC1[0] & 0x80)){ }
	//delay(1000);
	light_val = ADC0->R[0];
	return light_val;
}

unsigned char readRightSensor() {
	ADC0->SC1[0] = 1<<5 | 1<<0; // Set Channel, starts conversion.
	while(!(ADC0->SC1[0] & 0x80)){ }
	//delay(1000);
	light_val = ADC0->R[0];
	return light_val;
}
