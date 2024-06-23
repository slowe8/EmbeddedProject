/*
 * Copyright 2016-2023 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    Project 4.c
 * @brief   Application entry point.
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
#include "LineSensors.h"
/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */
#define BASE 0
#define AVOID 1
#define ORIENT_RIGHT 2
#define ORIENT_LEFT 3
#define WHITE 0
#define RED 1
#define GREEN 2
#define BLUE 3
#define YELLOW 4
/* TODO:
 * 1. debug servo issues
 * 2. implement sonar functions and readings
 * 3. add setup into each individual function to ensure proper setups (motor control and timer management)
 * 4. reorganize source files
 * 5. play around with using static over returns
 * 6. test size of long and timer overflow
 */

/*
 * @brief   Application entry point.
 */
int main(void) {

    /* Init board hardware. */
	BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif

    unsigned int curr_red = 0;
    unsigned int curr_green = 0;
    unsigned int curr_blue = 0;

    unsigned char RIGHT_LINE = 0;
    unsigned char LEFT_LINE = 0;

    bool finished = false;
    char state = BASE;
    char COLOR = 0;
    char TARGET = 0;

    setup_motors();
    init_LineSensors();
    delay_s(5);

    // Determine initial position
    readDone = false;
	unsigned int colorData[3] = {0};
	int success = readColorSensor(colorData);
	if(success == 1) {
		curr_red = colorData[0];
		curr_green = colorData[1];
		curr_blue = colorData[2];
	}
	// Determine Color
	if(curr_red > 10000 && curr_green > 10000 && curr_blue > 10000) {
		COLOR = WHITE;
		PRINTF("WHITE\n");
	} else if(curr_red > 10000 && curr_green > 10000 && curr_blue < 10000){
		COLOR = YELLOW;
		PRINTF("YELLOW\n");
	} else if(curr_red > curr_green && curr_red > curr_blue) {
		COLOR = RED;
		PRINTF("RED\n");
	} else if(curr_blue > curr_green && curr_blue > curr_green) {
		COLOR = BLUE;
		PRINTF("BLUE\n");
	} else if(curr_green > curr_red && curr_green > curr_blue) {
		COLOR = GREEN;
		PRINTF("GREEN\n");
	}

	// Determine Target Color
	if(COLOR == YELLOW) {
		TARGET = RED;
	} else if(COLOR == RED) {
		TARGET = YELLOW;
	} else if(COLOR == BLUE) {
		TARGET = GREEN;
	} else if(COLOR == GREEN) {
		TARGET = BLUE;
	}

	straight();
	while(1) {
		readDone = false;
		success = readColorSensor(colorData);
		if(success == 1) {
			curr_red = colorData[0];
			curr_green = colorData[1];
			curr_blue = colorData[2];
		}
		// Determine Color
		// Determine Color
		if(curr_red > 10000 && curr_green > 10000 && curr_blue > 10000) {
			COLOR = WHITE;
		} else if(curr_red > 10000 && curr_green > 10000 && curr_blue < 10000){
			COLOR = YELLOW;
		} else if(curr_red > curr_green && curr_red > curr_blue) {
			COLOR = RED;
		} else if(curr_blue > curr_green && curr_blue > curr_green) {
			COLOR = BLUE;
		} else if(curr_green > curr_red && curr_green > curr_blue) {
			COLOR = GREEN;
		}

		if(COLOR == TARGET) {
			brake();
			continue;
		}

		LEFT_LINE = readLeftSensor();
		delay_ms(2);
		RIGHT_LINE = readRightSensor();
		delay_ms(2);

		if(RIGHT_LINE > 100) {
			// Both WHITE
			state = ORIENT_LEFT;
		} else if (LEFT_LINE > 215) {
			state = ORIENT_RIGHT;
		} else {
			state = BASE;
		}

		if(state == BASE) {
			straight();
		} else if(state == ORIENT_LEFT) {
			turn(LEFT);
			delay_ms(70);
			brake();
		} else if(state == ORIENT_RIGHT) {
			turn(RIGHT);
			delay_ms(70);
			brake();
		}
	}

/*
    while(!finished) {
    	// Read in data
    	success = readColorSensor(colorData);
    	if(success == 1) {
    		curr_red = colorData[0];
    		curr_green = colorData[1];
    		curr_blue = colorData[2];
    	}
    	LEFT_LINE = readLeftSensor();
    	delay_ms(5);
    	RIGHT_LINE = readRightSensor();
    	delay_ms(5);

    	if(RIGHT_LINE < LEFT_LINE && LEFT_LINE < 215) {
    		// Both WHITE
    		straight();
    	} else if(RIGHT_LINE < LEFT_LINE && LEFT_LINE > 215) {
    		//delay_ms(2);
    		arc(RIGHT);
    	} else if(RIGHT_LINE > LEFT_LINE) {
    		//delay_ms(2);
    		arc(LEFT);
    	}

    	// Determine Color
    	if(curr_red > 10000 && curr_green > 10000 && curr_blue > 10000) {
    		COLOR = WHITE;
    	} else if(curr_red > 10000 && curr_green > 10000 && curr_blue < 10000){
    		COLOR = YELLOW;
    	} else if(curr_red > curr_green && curr_red > curr_blue) {
    		COLOR = RED;
    	} else if(curr_blue > curr_green && curr_blue > curr_green) {
    		COLOR = BLUE;
    	} else if(curr_green > curr_red && curr_green > curr_blue) {
    		COLOR = GREEN;
    	}

    	// Check color
    	if(COLOR != TARGET) {
    		continue;
    	} else {
    		delay_ms(50);
    		brake();
    		finished = true;
    	}

    }
*/
/*
    init_LineSensors();
    delay_ms(5);
    while(1) {
    	unsigned char left = readLeftSensor();
    	delay_ms(5);
    	unsigned char right = readRightSensor();
    	PRINTF("LEFT: %d\n", left);
    	PRINTF("RIGHT: %d\n", right);
    	delay_s(1);
    }
    // BOTH WHITE = RIGHT < LEFT && LEFT < 215
    // RIGHT BLACK = RIGHT > LEFT
    // LEFT BLACK = RIGHT < LEFT && LEFT > 215
*/
/*
    initI2C();
    unsigned char readIDAddr = 0x16;
    unsigned char data[6] = {0};
    WriteByte(0x00, 3);
    delay_ms(3);
    int success = ReadBlock(readIDAddr, data, 6);
    if(success == 1) {
    	PRINTF("Success\n");
    	for(int i=0; i<8; i++) {
    		PRINTF("Read: %d\n", data[i]);
    	}
    	int red = (data[1]<<8) | data[0];
    	PRINTF("RED: %d\n", red);
    	int green = (data[3]<<8) | data[2];
    	PRINTF("GREEN: %d\n", green);
    	int blue = (data[5]<<8) | data[4];
    	PRINTF("BLUE: %d\n", blue);
    }
    // Yellow
    // RED = 16963
    // GREEN = 14045
    // BLUE: 5904

    // Blue
    // RED = 2619
    // GREEN = 4628
    // BLUE = 6209

    // Green
    // RED = 6311
    // GREEN = 9360
    // BLUE = 5018

    // Red
    // RED = 11222
    // GREEN = 3948
    // BLUE = 3155

    // White
    // RED = 13580
    // GREEN = 14739
    // BLUE = 12693
*/
/*
    bool finished = false;

    int navState = 0;

    setup_LED();
    LED_off();
    PRINTF("Start.\n");
    setup_motors();
    setup_sonar();
    set_cnv(187);
    delay_s(1);
    echoReady = true;
    double sonar_dis = 0;
    while(!finished) {
    	if(echoReady) {
    		echoReady = false;
    		sonar_dis = updateSonar();
    	} else {
    		PRINTF("Stuck here\n");
    		continue;
    	}
    	switch(navState) {
    	case 0:
    		if(sonar_dis < 10) {
    			brake();
    			set_cnv(300);
    			delay_s(1);
    			double sonard = updateSonar();
    			if(sonard > 50) {
    				turn(LEFT);
    				set_cnv(50);
    				delay_s(1);
    				echoReady = true;
    				navState = 1;
    				continue;
    			}
    			delay_s(1);
    			set_cnv(50);
    			delay_s(1);
    			sonard = updateSonar();
    			if(sonard > 50) {
    				turn(RIGHT);
    				set_cnv(300);
    				delay_s(1);
    				echoReady = true;
    				navState = 2;
    				continue;
    			}
    		} else {
    			straight();
    		}
    		echoReady = true;
    		break;
    	case 1:
    		if(sonar_dis > 10) {
    			delay_s(1);
    			brake();
    			delay_s(1);
    			turn(RIGHT);
    			navState = 3;
    			echoReady = true;
    			continue;
    		}
    		echoReady = true;
    		break;
    	case 2:
    		if(sonar_dis > 10) {
    			delay_s(1);
    			brake();
    			delay_s(1);
    			turn(LEFT);
    			navState = 3;
    			echoReady = true;
    			continue;
    		}
    		echoReady = true;
    		break;
    	case 3:
    		if(sonar_dis > 100) {
    			delay_s(2);
    			brake();
    			delay_s(1);
    			if(TPM1->CONTROLS[0].CnV == 50) {
    				turn(RIGHT);
    				delay_s(1);
    				straight();
    				delay_s(3);
    				brake();
    				finished = true;
    			} else if (TPM1->CONTROLS[0].CnV == 300) {
    				turn(LEFT);
    				delay_s(1);
    				straight();
    				delay_s(3);
    				finished = true;
    			}
    		} else {
    			straight();
    		}
    		echoReady = true;
    		break;
    	}
    	//PRINTF("Updated. Sonar range: %3.2f\n", sonar_dis);

    	if(sonar_dis < 10) {
    		brake();
    		set_cnv(300);
    		delay_s(1);
    		double sonard = updateSonar();
    		if(sonard > 50) {
    			turn(LEFT);
    			set_cnv(50);
    			delay_s(1);
    			echoReady = true;
    			continue;
    		}
    		delay_s(1);
    		set_cnv(50);
    		delay_s(1);
    		sonard = updateSonar();
    		if(sonard > 50) {
    			turn(RIGHT);
    			set_cnv(300);
    			delay_s(1);
    			echoReady = true;
    			continue;
    		}
    	} else {
    		straight();
    	}
    	echoReady = true;
    }*/

    return 0;
}
