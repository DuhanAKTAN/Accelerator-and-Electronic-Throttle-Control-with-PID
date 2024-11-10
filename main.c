#ifndef F_CPU
#define F_CPU 16000000UL		// define it now as 16 MHz unsigned long
#endif

#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "millis.h"

#define ENA_PIN PB5   // PWM sinyali gönderilecek pin
#define IN1_PIN PE4   // Motor sürücü IN1 pini
#define IN2_PIN PE5   // Motor sürücü IN2 pini

// PID Define
//
#define AUTOMATIC	1
#define MANUAL	0
#define DIRECT  0
#define REVERSE  1
#define P_ON_M 0
#define P_ON_E 1
//

#define BIT_IS_SET(byte, bit) (byte & (1 << bit))
#define BIT_IS_CLEAR(byte, bit) (!(byte & (1 << bit)))

volatile uint16_t adc_result = 0;

long map(long x, long in_min, long in_max, long out_min, long out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void adc_init(void) {
	// AVCC'yi referans voltaj olarak seç (ADMUX REFS0 bitini set et)
	ADMUX = (1 << REFS0);
	// ADC'yi etkinle?tir, önbölme faktörünü 128 olarak ayarla
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

// ADC'den analog de?er okuyan fonksiyon
uint16_t adc_read(uint8_t channel) {
	// Kanal? seç (ADMUX'un alt 4 bitini ayarla)
	ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
	// ADC dönü?ümünü ba?lat
	ADCSRA |= (1 << ADSC);
	// Dönü?ümün bitmesini bekle
	while (ADCSRA & (1 << ADSC));
	// ADC sonucunu döndür
	return ADC;
}


void pwm_init(void) {
	// 11. pin (PB5, OC1A) ç?k?? olarak ayarlan?yor
	DDRB |= (1 << PB5);

	// Timer/Counter1 Control Register A
	// WGM11:0 = 1: Fast PWM mode
	// COM1A1 = 1: Non-inverted PWM
	TCCR1A |= (1 << WGM10) | (1 << WGM12) | (1 << COM1A1);

	// Timer/Counter1 Control Register B
	// WGM12 = 1: Fast PWM mode
	// CS10 = 1: No prescaler
	TCCR1B |= (1 << CS10) | (1 << WGM12);

	// Ba?lang?çta duty cycle'? %0 yapal?m
	OCR1A = 0;
}

void pwm_set(uint8_t duty_cycle) {
	// Duty cycle de?erini ayarla (0-255 aras?)
	OCR1A = duty_cycle;
}

ISR(ADC_vect) {
	adc_result = ADC; // Read ADC result
}

// PID Control Variables
//
double dispKp;				//  With temporary variable for display purpose,
double dispKi;				//  we will keep setting parameters in user entered format
double dispKd;

double kp;                  //  Proportional Tuning Parameter
double ki;                  //  Integral Tuning Parameter
double kd;                  //  Derivative Tuning Parameter

int controllerDirection;
int pOn;


unsigned long lastTime;
double outputSum, lastInput;

unsigned long SampleTime;
double outMin, outMax;
int inAuto, pOnE;
//

// Input and output values for the driver
//
double Setpoint, Input, Output, myOutput;
//

// Configuration values for the motor driver
//
double consKp = 9, consKi = 0.1, consKd = 2.5;
//

// Function definitions for PID
//
void PID(double Kp, double Ki, double Kd, int POn, int ControllerDirection);
int Compute();
void SetTunings(double Kp, double Ki, double Kd, int POn);
void SetSampleTime(int NewSampleTime);
void SetOutputLimits(double Min, double Max);
void SetMode(int Mode);
void Initalize();
void SetControllerDirection(int Direction);
//

int tpsMax,tpsMin,TPS;

int main(void) {
	// PWM'i ba?lat
	DDRB |= (1<<ENA_PIN);
	sei();
	adc_init();
	pwm_init();
	// IN1 ve IN2 pinlerini ç?k?? olarak ayarla
	DDRE |= (1 << IN1_PIN) | (1 << IN2_PIN);
	PORTE |= (1 << IN2_PIN);  // HIGH
	PORTE &= ~(1 << IN1_PIN); // LOW
	
	tpsMax = adc_read(2);
	tpsMin = adc_read(1);
	 
	
	
	while (1) {
		Setpoint = map(adc_read(0), 137, 449, 0, 255);
		Input = map(adc_read(1),tpsMin,tpsMax,0,255);

		PID(consKp, consKi, consKd, P_ON_E, DIRECT);

		SetMode(AUTOMATIC);

		SetTunings(consKp, consKi, consKd, pOn);

		Compute();


		pwm_set(Output);		
	}

	return 0;
}

void PID(double Kp, double Ki, double Kd, int POn, int ControllerDirection)
{
	
	SetOutputLimits(0,255);
	
	SampleTime = 100;
	
	SetControllerDirection(ControllerDirection);
	SetTunings(Kp, Ki, Kd, POn);
	
	lastTime = millis()-SampleTime;
	
}

int Compute()
{
	if(!inAuto) return 0;
	unsigned long now = millis();
	unsigned long timeChange = (now - lastTime);
	if(timeChange>=SampleTime)
	{
		/*Compute all the working error variables*/
		double input = Input;
		double error = Setpoint - input;
		double dInput = (input - lastInput);
		outputSum+= (ki * error);

		/*Add Proportional on Measurement, if P_ON_M is specified*/
		if(!pOnE) outputSum-= kp * dInput;

		if(outputSum > outMax) outputSum= outMax;
		else if(outputSum < outMin) outputSum= outMin;

		/*Add Proportional on Error, if P_ON_E is specified*/
		double output;
		if(pOnE) output = kp * error;
		else output = 0;

		/*Compute Rest of PID Output*/
		output += outputSum - kd * dInput;

		if(output > outMax) output = outMax;
		else if(output < outMin) output = outMin;
		Output = output;

		/*Remember some variables for next time*/
		lastInput = input;
		lastTime = now;
		return 1;
	}
	else return 0;
}

void SetTunings(double Kp, double Ki, double Kd, int POn)
{
	pOn = POn;
	pOnE = POn == P_ON_E;

	dispKp = Kp; dispKi = Ki; dispKd = Kd;

	double SampleTimeInSec = ((double)SampleTime)/1000;
	kp = Kp;
	ki = Ki * SampleTimeInSec;
	kd = Kd / SampleTimeInSec;

	if(controllerDirection ==REVERSE)
	{
		kp = (0 - kp);
		ki = (0 - ki);
		kd = (0 - kd);
	}
}

void SetSampleTime(int NewSampleTime)
{
	if (NewSampleTime > 0)
	{
		double ratio  = (double)NewSampleTime
		/ (double)SampleTime;
		ki *= ratio;
		kd /= ratio;
		SampleTime = (unsigned long)NewSampleTime;
	}
}

void SetOutputLimits(double Min, double Max)
{
	if(Min >= Max)return;
	outMin = Min;
	outMax = Max;
	
	if(inAuto)
	{
		if(Output > outMax) Output = outMax;
		else if(Output < outMin) Output = outMin;

		if(outputSum > outMax) outputSum= outMax;
		else if(outputSum < outMin) outputSum= outMin;
	}
}

void SetMode(int Mode)
{
	int newAuto = (Mode == AUTOMATIC);
	if(newAuto && !inAuto)
	{  /*we just went from manual to auto*/
		Initalize();
	}
	inAuto = newAuto;
}

void Initalize()
{
	outputSum = Output;
	lastInput = Input;
	if(outputSum > outMax) outputSum = outMax;
	else if(outputSum < outMin) outputSum = outMin;
}

void SetControllerDirection(int Direction)
{
	if(inAuto && Direction !=controllerDirection)
	{
		kp = (0 - kp);
		ki = (0 - ki);
		kd = (0 - kd);
	}
	controllerDirection = Direction;
}
//