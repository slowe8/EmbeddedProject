/*
 * Utilities.c
 *
 *  Created on: Feb 23, 2023
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

// Motor Controls variables
const float SHORT = 0.5;
const float LONG = 1.0;

// Encoder variables
int ENC_CYCLES = 144;
volatile signed long LEFT_ENC = 0;
volatile signed long RIGHT_ENC = 0;
volatile double total_d = 0;

volatile signed long last_measured;
volatile signed long current;

// Time variables
static volatile unsigned long time_now_ms = 0;
static volatile unsigned long time_now_us = 0;

// Controller variables
float setpoint = 0.70;
float measured_val = 0;
float Kp = 0.2;
float Ki = 0.001;
float Kd = 0.01;
float level = 0;
float last_error = 0;
float sum_error = 0;

// Sonar flags and variables
static double sonarRange = 10000000;
bool sonarUpdated = false;
unsigned long ECHO_start = 0;
unsigned long ECHO_stop = 0;
double ECHO_time = 0;
static volatile int edge_counter = 0;

// Color Sensor flags
static bool waitingCycles = false;

// Interrupt Handlers
void TPM1_IRQHandler(void){
	TPM1->SC |= (1 << 7); // Reset Timer Interrupt Flag
	time_now_us++; // What happens on overflow?

}

void PORTA_IRQHandler(void) {
	/*
	if(PORTA->PCR[6] & (1<<24)) {
		PORTA->PCR[6] |= (1<<24);
		LEFT_ENC++;
	}
	if(PORTA->PCR[7] & (1<<24)) {
		PORTA->PCR[7] |= (1<<24);
		LEFT_ENC--;
	}
	if(PORTA->PCR[14] & (1<<24)) {
		PORTA->PCR[14] |= (1<<24);
		RIGHT_ENC++;
	}
	if(PORTA->PCR[15] & (1<<24)) { // RIGHT FORWARD
		PORTA->PCR[15] |= (1<<24);
		//RIGHT_ENC--;
	}
	*/
	if(PORTA->PCR[13] & (1<<24)) { // ECHO
		/*
			 * PORTA13 interrupt triggers on rising
			 * - clear interrupt
			 * - record rising edge time
			 * PORTA13 interrupt triggers on falling
			 * - clear interrupt
			 * - record falling edge time
			 * - calculate total time
			 * - calculate range
			 */
		PORTA->PCR[13] |= (1<<24); // Clear interrupt
		if(edge_counter % 2 == 0) {
			ECHO_start = time_now_ms;
		} else {
			ECHO_stop = time_now_ms;
			ECHO_time = ECHO_stop - ECHO_start;
			sonarRange = ECHO_time/58; // range in cm, cm = us / 58
			PIT->CHANNEL[0].TCTRL = 0;
			sonarUpdated = true;
		}
		edge_counter++;
	}
	if(RIGHT_ENC % 70 == 0){
		total_d += 0.204;
	}
	/*
	enum Encoder trigger = check_encoder_pin();
	PORTA->PCR[trigger] |= (1<<24); // Clear interrupt flag at the triggered pin
	if(trigger == LEFTA) {
		LEFT_ENC++;
	} else if(trigger == LEFTB) {
		LEFT_ENC--;
	} else if(trigger == RIGHTA) {
		RIGHT_ENC++;
		PRINTF("Right Encoder: %d\n", RIGHT_ENC);
	} else if(trigger == RIGHTB) {
		RIGHT_ENC--;
	}
	*/
}

void PIT_IRQHandler(void) {
	if(PIT->CHANNEL[0].TFLG) {
		PIT->CHANNEL[0].TFLG = 1; // Reset
		time_now_ms++;
		PIT->CHANNEL[0].TCTRL = 0x3;
	} else if(PIT->CHANNEL[1].TFLG) {
		GPIOD->PTOR |= (1 << 2); // Set GPIOD2 low to stop 10us trigger
		PIT->CHANNEL[1].TFLG = 1; // Reset
		PIT->CHANNEL[1].TCTRL = 0; // Disable
	} else if(PIT->CHANNEL[2].TFLG) {
		waitingCycles = false;
		PIT->CHANNEL[2].TFLG = 1;
		PIT->CHANNEL[2].TCTRL = 0;
	}
}

// Timers

void delay_ms(unsigned short delay_t) {
	SIM->SCGC6 |= (1 << 24); // Enable clock for TMP0
	SIM->SOPT2 |= (0x2 << 24); // Set TPMSRC to OSCERCLK
	TPM0->CONF |= (0x1 << 17); // Set clock to stop on overflow
	TPM0->SC = (0x1 << 7) | (0x7); // Reset Timer Overflow Flag, Set Prescaler 128
	TPM0->MOD = delay_t*61 + delay_t/2 - 1; // Set MOD register for ms

	TPM0->SC |= (0x01<<3); // Start clock

	while(!(TPM0->SC & 0x80)){} // Wait until Overflow Flag
	return;
}

void delay_s(float delay_t) {
	/*
    SIM->SCGC6 |= (1 << 24); // Enable clock for TMP0
    SIM->SOPT2 |= (0x3 << 24); // Set TPMSRC to MCGIRCLK
    TPM0->CONF |= (0x1 << 17); // Set clock to stop on overflow
    TPM0->SC = (0x1 << 7) | (0x07); // Reset Timer Overflow Flag, Set Prescaler 128
    TPM0->MOD = delay_t*255 + delay_t/2 - 1; // Set MOD register for s

    TPM0->SC |= (0x01<<3); // Start clock

    while(!(TPM0->SC & 0x80)){} // Wait until Overflow Flag
    */
	unsigned short s = delay_t;
	unsigned short ms = (unsigned short)((delay_t - s)*1000);
	for(int i=0; i<s; i++) {
		delay_ms(1000);
	}
	if(ms != 0) {
		delay_ms(ms);
	}
    return;
}

void init_TPM1() {
	SIM->SCGC6 |= (1 << 25); // Clock enable TPM1
	SIM->SOPT2 |= (0x2 << 24); // TPM Clock Source OSCERCLK
	TPM1->MOD = 7; // Reload every microsecond.
	NVIC_EnableIRQ(18);
	// Reset TOF, Enable Interrupt, Prescaler = 0, Start Timer
	TPM1->SC = (1 << 7) | (1 << 6) | (1 << 3);
}

int get_time() {
	return time_now_ms;
}

// Switches
void setup_SW1() {
	SIM->SCGC5 |= (1 << 11); // Enable clocking for SW1
	PORTC->PCR[3] &= ~(0x703); // Clear MUX bits
	PORTC->PCR[3] |= 0x703 & ((1 << 8) | 0x03); // Set MUX bits to GPIO and pullup enable
	GPIOC->PDDR &= ~(1 << 3);
	return;
}

bool SW1isPressed() {
	if(GPIOC->PDIR & 0x8) {
		return 0;
	} else {
		return 1;
	}
}

// LED
void setup_LED() {
	SIM->SCGC5 |= (1<<13);
	PORTE->PCR[29] &= ~(0x700);
	PORTE->PCR[29] |= (1 << 8);
	GPIOE->PDDR |= (1 << 29);
	return;
}

void LED_on() {
	setup_LED();
	GPIOE->PDOR &= ~(1 << 29);
	return;
}

void LED_off() {
	setup_LED();
	GPIOE->PDOR |= (1 << 29);
}

/*
 * Motor control functions
 */
void setup_motors() {
	SIM->SCGC5 |= (1 << 10) | (1 << 11); // Enable clock for PORTB and PORTC
	SIM->SCGC6 |= (1 << 26); // Clock Enable TPM2
	SIM->SOPT2 |= (0x2 << 24); // Set TPMSRC to OSCERCLK

	PORTB->PCR[0] &= ~(0x700); // Clear MUX bits
	PORTB->PCR[0] |= (1 << 8); // Set MUX to GPIO
	PORTB->PCR[1] &= ~(0x700); // Clear MUX bits
	PORTB->PCR[1] |= (1 << 8); // Set MUX to GPIO

	// PWM pins !! change later
	/*
	PORTB->PCR[2] &= ~(0x700); // Clear MUX bits
	PORTB->PCR[2] |= (1 << 8); // Set MUX to GPIO
	PORTB->PCR[3] &= ~(0x700); // Clear MUX bits
	PORTB->PCR[3] |= (1 << 8); // Set MUX to GPIO
	*/
	GPIOB->PDDR |= (1 << 0) | (1 << 1); // Set all GPIOB bits as output

	PORTB->PCR[2] &= ~(0x700); // Clear MUX bits
	PORTB->PCR[2] |= (0x300); // Set to TMP2_CH0
	PORTB->PCR[3] &= ~(0x700); // Clear MUX bits
	PORTB->PCR[3] |= (0x300); // Set to TPM2_CH1

	TPM2->CONF |= (0x3 << 6);
	TPM2->CONTROLS[0].CnSC |= (0x2 << 2) | (0x2 << 4);
	TPM2->CONTROLS[1].CnSC |= (0x2 << 2) | (0x2 << 4);
	TPM2->MOD = 10000;

	PORTC->PCR[1] &= ~(0x700);
	PORTC->PCR[1] |= (1 << 8);
	PORTC->PCR[2] &= ~(0x700);
	PORTC->PCR[2] |= (1 << 8);
	GPIOC->PDDR |= (1 << 1) | (1 << 1); // Set all GPIOC bits as output
	return;
}

void brake() {
	GPIOC->PDOR &= ~(1 << 1);
	GPIOC->PDOR &= ~(1 << 2);
	GPIOB->PDOR &= ~(1 << 1);
	GPIOB->PDOR &= ~(1 << 0);
}

void drive_motor(enum Motor motor, enum Direction direction, int speed) {
	if(motor == R) {
		if(direction == FORWARD) {
			GPIOC->PDOR |= (1 << 1);
			GPIOC->PDOR &= ~(1 << 2);
			TPM2->CONTROLS[1].CnV = speed;
		} else if(direction == REVERSE) {
			GPIOC->PDOR &= ~(1 << 1);
			GPIOC->PDOR |= (1 << 2);
			GPIOB->PDOR |= (1 << 3);
		}
	} else {
		if(direction == FORWARD) {
			GPIOB->PDOR &= ~(1 << 1);
			GPIOB->PDOR |= (1 << 0);
			TPM2->CONTROLS[0].CnV = speed;
		} else if(direction == REVERSE) {
			GPIOB->PDOR |= (1 << 1);
			GPIOB->PDOR &= ~(1 << 0);
			GPIOB->PDOR |= (1 << 2);
		}
	}
	TPM2->SC |= (0x01 << 3);
	return;
}

void straight() {
	drive_motor(L, FORWARD, 8000);
	drive_motor(R, FORWARD, 8000);
	return;
}

void turn(enum Direction dir) {
	if(dir == LEFT) {
		brake();
		drive_motor(R, FORWARD, 8000);
	} else if(dir == RIGHT) {
		brake();
		drive_motor(L, FORWARD, 8000);
	}
	return;
}

void arc(enum Direction dir) {
	if(dir == LEFT) {
		drive_motor(R, FORWARD, 10000);
		drive_motor(L, FORWARD, 8000);
	} else if(dir == RIGHT) {
		drive_motor(R, FORWARD, 8000);
		drive_motor(L, FORWARD, 10000);
	}
	return;
}

/*
 * Speed Control functions
 */
enum Encoder check_encoder_pin() {
	if(PORTA->PCR[6] & (1<<24)) {
		return LEFTA;
	} else if(PORTA->PCR[7] & (1<<24)) {
		return LEFTB;
	} else if(PORTA->PCR[14] & (1<<24)) {
		return RIGHTA;
	} else if(PORTA->PCR[15] & (1<<24)) {
		return RIGHTB;
	}
	return NOINTR;
}

float GetControlLevel(float measured) {
    float error = setpoint - measured;
    float change = error - last_error;
    sum_error += error;
    float level = Kp*error + Ki*sum_error + Kd*change;

    last_error = error;
    return level;
}

void init_encoders() {
	SIM->SCGC5 |= 1<<9; // Enable clock for PORTA
	PORTA->PCR[6] &= ~(0xF0700); // Clear and set GPIO for encoders
	PORTA->PCR[6] |= (1<<8) | (9<<16);

	PORTA->PCR[7] &= ~(0xF0700);
	PORTA->PCR[7] |= (1<<8) | (9<<16);

	PORTA->PCR[14] &= ~(0xF0700);
	PORTA->PCR[14] |= (1<<8) | (9<<16);

	PORTA->PCR[15] &= ~(0xF0700);
	PORTA->PCR[15] |= (1<<8) | (9<<16);

	GPIOA->PDDR &= ~(1<<6);
	GPIOA->PDDR &= ~(1<<7);
	GPIOA->PDDR &= ~(1<<14);
	GPIOA->PDDR &= ~(1<<15); // Set direction to input

	PORTA->PCR[6] |= (1<<24);
	PORTA->PCR[7] |= (1<<24);
	PORTA->PCR[14] |= (1<<24);
	PORTA->PCR[15] |= (1<<24);

	NVIC_EnableIRQ(30);
}

void init_PIT() {
	SIM->SCGC6 |= (1<<23);
	PIT->MCR = 0x00;
	PIT->CHANNEL[0].LDVAL = 239999;
	PIT->CHANNEL[0].LDVAL = 239999;
	NVIC_EnableIRQ(22);
	PIT->CHANNEL[0].TCTRL = 0x3;

	return;
}

/*
 * Servo functions
 */
void setup_servo() {

	SIM->SCGC5 |= 1<<9; // Enable clock for PORTA
	SIM->SCGC6 |= 1<<25; // Enable clock for TPM1
	SIM->SOPT2 |= (0x2 << 24); // Set clock for TPM
	PORTA->PCR[12] &= ~(0x700); // Clear MUX bits
	PORTA->PCR[12] |= (0x300); // Set to TMP1_CH0

	TPM1->CONF |= (0x3 << 6);
	TPM1->CONTROLS[0].CnSC |= (0x2 << 2) | (0x2 << 4);
	TPM1->MOD = 1250;

	TPM1->SC |= (0x01 << 3) | (0x6 << 0);

}

void set_cnv(int cnv) {
	setup_servo();
	TPM1->CONTROLS[0].CnV = cnv;
	delay_ms(200);
}

/*
 * Sonar functions
 */
void setup_sonar() {
	SIM->SCGC5 |= (1 << 12) | (1 << 9); // Enable clock for PORTD and PORTA
	SIM->SCGC6 |= (1 << 23); // Enable clock for PIT

	PIT->MCR = 0x00; // PIT CLOCK IS 24MHz
	PIT->CHANNEL[0].LDVAL = 24; // 1ms timer for whole communication
	PIT->CHANNEL[1].LDVAL = 240; // 10us trigger timer
	NVIC_EnableIRQ(22);

	PORTD->PCR[2] &= ~(0x700);
	PORTD->PCR[2] |= (1 << 8); // Clear and set MUX to GPIO
	GPIOD->PDDR |= (1 << 2); // Set GPIOD2 as input
	GPIOD->PDOR &= ~(1<<2);

	PORTA->PCR[13] &= ~(0xF0700); // Clear and set GPIO for encoders
	PORTA->PCR[13] |= (1<<8) | (0xB<<16);
	GPIOA->PDDR &= ~(1<<13);
	PORTA->PCR[13] |= (1<<24);
	NVIC_EnableIRQ(30);
}

double updateSonar() {
	/*
	 * SETUP!!!
	 * Send 10us pulse on PTD2
	 * Implementation: 10us 2-channel PIT setup
	 */
	GPIOD->PTOR |= (1 << 2); // Write high to GPIOD2
	PIT->CHANNEL[1].TCTRL = 0x3; // Start trigger timer
	PIT->CHANNEL[0].TCTRL = 0x3;
	sonarUpdated = false;
	//PIT interrupt stops timer and sets GPIOD2 low
	/*
	 * PORTA13 interrupt triggers on rising
	 * - clear interrupt
	 * - record rising edge time
	 * PORTA13 interrupt triggers on falling
	 * - clear interrupt
	 * - record falling edge time
	 * - calculate total time
	 * - calculate range
	 */
	// May have to disable TPM1
	while(!sonarUpdated){}
	echoReady = true;
	return sonarRange;

}
