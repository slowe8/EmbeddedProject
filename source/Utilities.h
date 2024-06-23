/*
 * Utilities.h
 *
 *  Created on: Feb 23, 2023
 *      Author: seanm
 */

#ifndef UTILITIES_H_
#define UTILITIES_H_

//static double sonarRange;
static bool echoReady;

enum Motor {L, R};
enum Direction {FORWARD, REVERSE, RIGHT, LEFT};
enum Speed {NORMAL=10000, FAST=15000, SLOW=5000};
enum Encoder {LEFTA=6, LEFTB=7, RIGHTA=14, RIGHTB=15, NOINTR=-1};
enum Position {PRIGHT=90, CENTER=187, PLEFT=300};

void ColorSensor_waitSixCycles();

void delay_ms(unsigned short delay_t);

void delay_s(float delay_t);

void setup_SW1();

bool SW1isPressed();

void setup_LED();

void LED_on();

void LED_off();

// Motor functions
void setup_motors();

void brake();

void drive_motor(enum Motor motor, enum Direction direction, int speed);

void straight();

void turn(enum Direction dir);

void arc(enum Direction dir);

// Speed Control functions
enum Encoder check_encoder_pin();

float GetControlLevel(float measured);

void init_TPM1();

int get_time();

void init_encoders();

void init_PIT();

void change_setpoint(float sp);

// Servo functions
void setup_servo();

void set_cnv(int cnv);

// Sonar functions
void setup_sonar();

double updateSonar();

#endif /* UTILITIES_H_ */
