#define F_CPU 16000000UL
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "uart.h"

//PWM variables
volatile unsigned int right_duty_cycle;
volatile unsigned int left_duty_cycle;
#define PWML PORTD5
#define PWMR PORTD6

//Motor variables
#define L_CTRL_1 PORTD2
#define L_CTRL_2 PORTD4
#define R_CTRL_1 PORTD7
#define R_CTRL_2 PORTB0

//PWM and sensor variables
volatile unsigned int Ain;
volatile unsigned int left_Ain;
volatile unsigned int middle_Ain;
volatile unsigned int right_Ain;
#define sensor_threshold 600 //Ain threshold at which motors should change behavior

#define left_sensor (1<<REFS0)
#define middle_sensor ((1<<REFS0) | (1<<MUX0))
#define right_sensor ((1<<REFS0) | (1<<MUX1))

volatile unsigned int last_dir = 0; //variable to save last direction of redbot

volatile unsigned long delay_counter = 0; //counter for delay ISR

volatile int diameter;

void initialize_all(void){
	//set outputs
	DDRD |= (1<<PWML) | (1<<PWMR) | (1<<L_CTRL_1) | (1<<R_CTRL_1) | (1<<L_CTRL_2);
	DDRB |= (1<<R_CTRL_2);

	//enable PWM
	PORTD |= (1<<PWML) | (1<<PWMR);
		
	//ADC setup
	ADCSRA |= (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);		// Set ADC prescaler to 128	and enable								

	//set up timer 0 for PWM
	TCCR0A |= (1<<WGM01) | (1<<WGM00) | (1<<COM0A1) | (1<<COM0B1); //Fast PWM Mode, 255 top
	OCR0B = left_duty_cycle;
	OCR0A = right_duty_cycle;
	TIMSK0 |= (1<<OCIE0B); //enable TIMER0 COMPB ISR
	TCCR0B |= (1<<CS00); //prescalar of 1
	
	//set up timer 1 for delay
	OCR1A = 249; // Set the compare reg to 249 time ticks (every 1ms)
	TIMSK1 |= (1<<OCIE1A); // turn on Timer0 Compare match ISR
	TCCR1B = (1<<WGM12) | (1<<CS11) | (1<<CS10); // turn on clear-on-match, CTC mode and set pre-scalar to divide by 64
	
	sei();		
}

//update left/right motor duty cycles
ISR (TIMER0_COMPB_vect)
{
	OCR0B = left_duty_cycle;
	OCR0A = right_duty_cycle;
}

ISR (TIMER1_COMPA_vect)
{
	if (delay_counter > 0)
		delay_counter--;
}

//function to read Ain from left, middle, and right sensors
void read_sensors() {
	ADMUX = left_sensor;
	ADCSRA |= (1<<ADSC);
	while((ADCSRA & (1<<ADSC)));
	Ain = ADCL;
	Ain |= (ADCH<<8);
	left_Ain = Ain;
	
	ADMUX = middle_sensor;
	ADCSRA |= (1<<ADSC);
	while((ADCSRA & (1<<ADSC)));
	Ain = ADCL;
	Ain |= (ADCH<<8);
	middle_Ain = Ain;
	
	ADMUX = right_sensor;
	ADCSRA |= (1<<ADSC);
	while((ADCSRA & (1<<ADSC)));
	Ain = ADCL;
	Ain |= (ADCH<<8);
	right_Ain = Ain;
}

//functions to control behavior and speed/duty cycles of motors
void stop() {
	PORTD &= ~((1<<L_CTRL_1) | (1<<L_CTRL_2) | (1<<R_CTRL_1));
	PORTB &= ~(1<<R_CTRL_2);
	left_duty_cycle = 0;
	right_duty_cycle = 0;
}

void go() {
	PORTD |= (1<<L_CTRL_2) | (1<<R_CTRL_1);
	PORTD &= ~(1<<L_CTRL_1);
	PORTB &= ~(1<<R_CTRL_2);
}

void reverse() {
	PORTD &= ~((1<<L_CTRL_2) | (1<<R_CTRL_1));
	PORTD |= (1<<L_CTRL_1);
	PORTB |= (1<<R_CTRL_2);
	left_duty_cycle = 60;
	right_duty_cycle = 67;
}

//left motor is faster than right, balancing ratio L/R: 130/143
void straight() {
	go();					//straight line //S curve
	left_duty_cycle = (int)(130*0.1685*log(2*diameter)+.1547); //140 //130
	right_duty_cycle = 130; //155 //143
}

void slowright() {
	go();
	left_duty_cycle = 32;
	right_duty_cycle = 105; //95 //105
}

void slowleft() {
	go();
	left_duty_cycle = 99; //90 //99
	right_duty_cycle = 35;
}

void right() {
	go();
	left_duty_cycle = 19;
	right_duty_cycle = 130;
}

void left() {
	go();
	left_duty_cycle = 119;
	right_duty_cycle = 20;
}

//function to choose direction based on sensor values
void choose_direction() {
	if ((left_Ain < sensor_threshold) && (middle_Ain > sensor_threshold) && (right_Ain < sensor_threshold)) {
		straight();
	}
	else if ((left_Ain > sensor_threshold) && (middle_Ain > sensor_threshold) && (right_Ain < sensor_threshold)) {
		slowright();
		last_dir = 1;
	}
	else if ((left_Ain > sensor_threshold) && (middle_Ain < sensor_threshold) && (right_Ain < sensor_threshold)) {
		right();
		last_dir = 1;
	}
	else if ((left_Ain < sensor_threshold) && (middle_Ain > sensor_threshold) && (right_Ain > sensor_threshold)) {
		slowleft();
		last_dir = 2;
	}
	else if ((left_Ain < sensor_threshold) && (middle_Ain < sensor_threshold) && (right_Ain > sensor_threshold)) {
		left();
		last_dir = 2;
	}
	else if ((left_Ain < sensor_threshold) && (middle_Ain < sensor_threshold) && (right_Ain < sensor_threshold)) {
		if (last_dir == 1)
			right();
		else if (last_dir == 2) 
			left();
	}
	else {
		straight();
	}
}

int main(void){
	initialize_all();
	
	uart_init();
	
	go();
	diameter = 25;
	straight();

	while(1) {
		read_sensors();
		
		choose_direction();
		
		//print sensor values to uart console
 		fprintf(stdout, "%d    ", left_Ain);
 		fprintf(stdout, "%d    ", middle_Ain);
 		fprintf(stdout, "%d\n", right_Ain);
	}
	return 0;
}