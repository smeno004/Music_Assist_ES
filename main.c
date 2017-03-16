/*
 * Final-Proto-Code.c
 *
 * Created: 3/16/2017 10:20:58 AM
 * Author : Siddharth Menon
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <math.h>

#include <util/delay.h>
#define N 32 //Number of samples for Fourier Transform
#include "lookup.h" //Lookup table for Fourier Transform
#include <limits.h> //For the frequency display

//Additional libraries included
#include <bit.h>
#include <timer.h>
#include <io.c>
#include <pwm.h>
#include <lcd5110.h>
#include <fixedFloatConvert.h>

//Some defined values required for some computations
//#define VREF 3.3
//#define VRMSPa 0.00631
//#define Gain 45

unsigned char toggle = 0x00;

//Some shared variables
unsigned short peakToPeak = 0;
//double rmsVolt = 0;
//unsigned long temp = 0;
//double dB_First = 0;
//double dB_Second = 0;

//More Shared variables 
//unsigned char digitArray[7];
unsigned char col = 0;
unsigned char printNow = 0;
unsigned char printed = 0;
unsigned char count = 0;

//Default Radius for the Loudness Viewer
short currRadius = 12;

//ADC initialization and Timer init for Frequency Viewer
void adc_init();
unsigned short adc_read();
void TRANSFORM();
void timer1_init();

//Arrays for Fourier Transform
long FourierInput[N];
long FourierOut[N/2][2];

//Attempt to calculate Frequency
unsigned short frequency;

//Arrays for Frequency Viewer
unsigned char digitArray[7];
unsigned char bottomArray[] = {219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219};
unsigned char Array[N/2];
unsigned char totalArray[32];

//Fill Array for Frequency Spectrum
void fillArraySpec(short freqInd) {
	for (unsigned char k = 0; k < N/2; k++) {
		if (k == freqInd) {
			Array[k] = 219;
		}
		else {
			Array[k] = ' ';
		}
		//digitArray[k-1] = 219;
	}
}

//Radius Lookup for Loudness viewer
//(Possible Improvement): automatic calibration?
const double radius_lookup_table[] = {
	22.662000,
	23.062000,
	23.462000,
	23.862000,
	24.262000,
	24.662000,
	25.062000,
	25.462000,
	25.862000,
	26.262000,
	26.662000,
	27.062000,
	27.462000,
	27.862000,
	28.262000,
	28.662000,
	29.062000,
	29.462000,
	29.862000,
	30.262000,
	30.662000,
	31.062000,
	31.462000
};

//ADC initialization for Loudness Viewer
void ADC_init() {
	ADCSRA |= (1 << ADEN) | (1 << ADSC) | (1 << ADATE);
	// ADEN: setting this bit enables analog-to-digital conversion.
	// ADSC: setting this bit starts the first conversion.
	// ADATE: setting this bit enables auto-triggering. Since we are
	//        in Free Running Mode, a new conversion will trigger whenever
	//        the previous conversion completes.
}

//Currently used to parse each digit of a number passed in
//Used for testing: to print out values taken in by ADC
void fillArrayVolt(short signalVal) {
	unsigned char digit = 0x00;
	double decVal = fixed_to_float(signalVal);
	double noDecValue = decVal * 1000;
	for (unsigned char k = 6; k > 0; k--) {
		digit = (int)noDecValue % 10;
		noDecValue = noDecValue / 10;

		if (k == 3) {
			digitArray[k - 1] = '.';
			k--;
		}

		digitArray[k-1] = digit + '0';
	}
	digitArray[7] = '\0';
	if (!printed) {
		printNow = 1;
	}
}

//dBV calculation
short fixedPoint_12_4_dBV_Calc(short signalMax) {
	double VREF = 3.3;
	double VRMSPa = 0.0631;
	short Gain = 45;

	double rmsVolt = ((signalMax * VREF) / 1024) * 0.707; //Calculates RMS Voltage
	double dB_First = log10(rmsVolt/VRMSPa) * 20; //First Calculation
	double dB_Second = dB_First + 94 - 44 - Gain; //Final Conversion to dBV

	//double rmsVolt = fixed_to_float(mult_fixed(div_fixed((mult_fixed(signalMax, float_to_fixed(VREF))), 1024), float_to_fixed(0.707)));
	//double dB_First = fixed_to_float(mult_fixed(float_to_fixed(log10(fixed_to_float(div_fixed(float_to_fixed(rmsVolt),float_to_fixed(VRMSPa))))) , 20));
	//double dB_Second = fixed_to_float(sub_fixed(sub_fixed(add_fixed(float_to_fixed(dB_First), 94), 44), Gain));

	//Convert to fixed point
	return float_to_fixed(dB_Second);
}

//Printing onto the LCD 5110
void LCD_5110_pulse_print(short dBVal) {
	//double threshold = 27.062;
	//double dB_Gap = 0.4;
	short radius = 0;
	double currdB = fixed_to_float(dBVal);
	//char radiusAltered = 0;

	short i = 0;
	for(i = 0; i < 23; i++) {
		if (currdB >= radius_lookup_table[i]) {
			radius++;
		}
		//if ((currdB > threshold) && (currdB <= ((i * dB_Gap) + threshold))) {
			//radius += i;
			//radiusAltered = 1;
			//break;
		//}
		//else if ((currdB < threshold) && (currdB >= (threshold - (i * dB_Gap)))) {
			//radius -= i;
			//radiusAltered = 1;
			//break;
		//}
	}

	if (radius == 0) {
		radius = 1;
	}

	//LCD_5110_clear();
	LCD_5110_DrawCirc(42,23,radius,Black,Filled_bool);
	//if (currRadius == radius) {
		//LCD_5110_DrawCirc(42,23,currRadius,Black,Filled_bool);
	//}
	//else {
		//LCD_5110_DrawCirc(42,23,radius,Black,Filled_bool);
	//}
	LCD_5110_update();

	currRadius = radius;

}

//--------Find GCD function --------------------------------------------------
unsigned long int findGCD(unsigned long int a, unsigned long int b)
{
	unsigned long int c;
	while(1){
		c = a%b;
		if(c==0){return b;}
		a = b;
		b = c;
	}
	return 0;
}
//--------End find GCD function ----------------------------------------------

//--------Task scheduler data structure---------------------------------------
// Struct for Tasks represent a running process in our simple real-time operating system.
typedef struct _task {
	/*Tasks should have members that include: state, period,
		a measurement of elapsed time, and a function pointer.*/
	signed char state; //Task's current state
	unsigned long int period; //Task period
	unsigned long int elapsedTime; //Time elapsed since last task tick
	int (*TickFct)(int); //Task tick function
} task;

//--------End Task scheduler data structure-----------------------------------

enum Sample_States {Sample_Start, Sampling};
unsigned char numSamples = 0;
unsigned short adc_value = 0x0000;
unsigned short signalMax = 0;
unsigned short signalMin = 1024;
unsigned short sampleWindow = 50;
short dBVval = 0;

//--------SAMPLING STATE MACHINE---------------------------------------
// State machine to sample ADC values and calculate peak value
int Sample_SM_TckFct(int state) {
	switch(state) {
		case Sample_Start:
			signalMax = 0;
			signalMin = 1024;
			peakToPeak = 0;
			numSamples = 0;
			state = Sampling;
			break;
		case Sampling:
			state = Sampling;
			//if (numSamples >= 50) {
				//state = Sample_Start;
			//}
			//else {
				//state = Sampling;
			//}
		default:
			state = Sample_Start;
			break;
	}

	switch(state) {
		case Sample_Start:
			peakToPeak = signalMax - signalMin;
			break;
		case Sampling:
			if (numSamples < sampleWindow) {
				numSamples++;
				adc_value = ADC;

				if (adc_value < 1024) {
					if (adc_value > signalMax) {
						signalMax = adc_value;
					}
					
					else if (adc_value < signalMin) {
						signalMin = adc_value;
					}
				}
			}
			else {
				peakToPeak = signalMax;
				signalMax = 0;
				signalMin = 1024;
			}
			break;
		default:
			break;
	}

	return state;
}
//--------END SAMPLING STATE MACHINE---------------------------------------

enum calc_dB_States {calc_Start,calc_dB};

//--------Decibel Calculation STATE MACHINE---------------------------------------
// State machine to calculate dBV values based on peak voltage values
int calc_dB_SM_TckFct(int state) {
	switch(state) {
		case calc_Start:
			state = calc_dB;
			break;
		case calc_dB:
			state = calc_dB;
			break;
		default:
			state = calc_Start;
			break;
	}

	switch(state) {
		case calc_dB:
			dBVval = fixedPoint_12_4_dBV_Calc(signalMax);
			//rmsVolt = ((float_to_fixed(signalMax * VREF)) / 1024) * 0.707;
			//dB_First = log10(rmsVolt/VRMSPa) * 20;
			//dB_Second = dB_First + 94 - 44 - Gain;
			//temp = dB_Second * 1000;
			fillArrayVolt(dBVval);
			break;
		default:
			break;
	}

	return state;
}
//--------END Decibel Calculation STATE MACHINE---------------------------------------

//enum print_States {print_Start, print_Now};
//
//int print_SM_TckFct(int state) {
	//switch(state) {
		//case print_Start:
			//state = print_Now;
			//break;
		//case print_Now:
			//state = print_Now;
			//break;
		//default:
			//state = print_Start;
			//break;
	//}
//
	//switch(state) {
		//case print_Now:
			//LCD_ClearScreen();
			//LCD_DisplayString(1,digitArray);
			//break;
		//default:
			//break;
	//}
//
	//return state;
//}

enum pulse_States {pulse_Start, pulse_Now};

//--------LCD Pulse Generation STATE MACHINE---------------------------------------
// State machine to generate circular pulse on the LCD 5110
int pulse_SM_TckFct(int state) {
	switch(state) {
		case pulse_Start:
			state = pulse_Now;
			break;
		case pulse_Now:
			state = pulse_Now;
			break;
		default:
			state = pulse_Start;
			break;
	}

	switch(state) {
		case pulse_Now:
			LCD_5110_pulse_print(dBVval);
			break;
		default:
			break;
	}

	return state;
}
//--------END LCD Pulse Generation STATE MACHINE---------------------------------------

//--------LOUDNESS VIEWER --------------------------------------------------
// State Machine controller to calculate decibel values and generate pulses
int decibelShow(void)
{
	DDRA = 0x00; PORTA = 0xFF;
	DDRD = 0xFF; PORTD = 0x00; // LCD data lines
	DDRC = 0xFF; PORTC = 0x00; // LCD control lines

	DDRB = 0xFF; PORTB = 0x00; //Test outputs

    // Period for the tasks
    unsigned long int sampling_period = 1;
	unsigned long int calc_dB_period = 50;
	//unsigned long int print_period = 500;
	unsigned long int pulse_period = 200;

    static task task1;
	static task task2;
	//static task task3;
	static task task4;

    task *tasks[] = {&task1,&task2,&task4};
    const unsigned short numTasks = sizeof(tasks)/sizeof(task*);

	//Calculating GCD
	unsigned long int tmpGCD = 1;
	tmpGCD = findGCD(sampling_period, calc_dB_period);
	//tmpGCD = findGCD(tmpGCD, print_period);
	tmpGCD = findGCD(tmpGCD, pulse_period);

	//Greatest common divisor for all tasks or smallest time unit for tasks.
	unsigned long int GCD = tmpGCD;

    // Task 1
    task1.state = -1;//Task initial state.
    task1.period = sampling_period;//Task Period.
    task1.elapsedTime = sampling_period;//Task current elapsed time.
    task1.TickFct = &Sample_SM_TckFct;//Function pointer for the tick.

	// Task 2
	task2.state = -1;//Task initial state.
	task2.period = calc_dB_period;//Task Period.
	task2.elapsedTime = calc_dB_period;//Task current elapsed time.
	task2.TickFct = &calc_dB_SM_TckFct;//Function pointer for the tick.

	//// Task 3
	//task3.state = -1;//Task initial state.
	//task3.period = print_period;//Task Period.
	//task3.elapsedTime = print_period;//Task current elapsed time.
	//task3.TickFct = &print_SM_TckFct;//Function pointer for the tick.

	// Task 4
	task4.state = -1;//Task initial state.
	task4.period = pulse_period;//Task Period.
	task4.elapsedTime = pulse_period;//Task current elapsed time.
	task4.TickFct = &pulse_SM_TckFct;//Function pointer for the tick.

    TimerSet(GCD);
    TimerOn();

	ADC_init();

	//// Initializes the 16x2 LCD display
	//LCD_init();
	//LCD_ClearScreen();

	//Initialize the Nokia 5110 LCD Display
	LCD_5110_init();

	LCD_5110_DrawCirc(41,23,currRadius,Black,Filled_bool);
	LCD_5110_update();

	unsigned short i; // Scheduler for-loop iterator
    while(!toggle) {
	    // Scheduler code
	    for ( i = 0; i < numTasks; i++ ) {
		    // Task is ready to tick
		    if ( tasks[i]->elapsedTime == tasks[i]->period ) {
			    // Setting next state for task
			    tasks[i]->state = tasks[i]->TickFct(tasks[i]->state);
			    // Reset the elapsed time for next tick.
			    tasks[i]->elapsedTime = 0;
		    }
		    tasks[i]->elapsedTime += 1;
	    }

		toggle = (~PINA & 0x02);

	    while(!TimerFlag);
	    TimerFlag = 0;
    }
	TimerOff();
	TimerSet(100);
	TimerOn();
    return 0;
}
//--------END LOUDNESS VIEWER --------------------------------------------------

//--------Frequency Spectrum Displayer ------------------------------------------
//Controller to display frequency spectrum based on a Fourier Transform
int FreqSpec_Disp(void)
{
	DDRA = 0x00; PORTA = 0xFF;
	DDRD = 0xFF; PORTD = 0x00; // LCD data lines
	DDRC = 0xFF; PORTC = 0x00; // LCD control lines

	short fftarray[N];				// array to hold FFT points
	short i;
	adc_init();
	
	timer1_init();

	LCD_init();

	unsigned char yo = 0;

	LCD_ClearScreen();
	LCD_DisplayString(17,bottomArray);

	while(!toggle) {
		TIMSK1 = 0;
		TIMSK0 = 1;
		TCNT1 = 0;
		TIFR0 |= 1<<OCF1A;
		for(i=0;i<N;i++) {
			while((TIFR1 & (1<<OCF1A)) == 0);
			FourierInput[i] = ((short)adc_read());
			TIFR1 |= 1<<OCF1A;
		}
		TRANSFORM();
		for(i =1; i<N/2; i++) {
			if(FourierOut[i][0]<0) {
				FourierOut[i][0] *= -1;
			}
			if(FourierOut[i][1]<0) { 
				FourierOut[i][1] *= -1;
			}
			
			fftarray[i] = (FourierOut[i][0] * FourierOut[i][0]) + (FourierOut[i][1] * FourierOut[i][1]);
		}

		short max_mag = SHRT_MIN;
		char max_index;

		for (i=1;i<(N/2);i++) {
			if (fftarray[i] > max_mag) {
				max_mag = fftarray[i];
				max_index = i;
			}
		}

		frequency = max_index * 800 / N;

		yo++;

		if (yo == 10) {
			printNow = 1;
		}

		printNow = 1;

		if (printNow) {
			fillArraySpec(max_index);
			//memcpy(totalArray, Array, 16);
			//memcpy(totalArray + 16, bottomArray, 16);
			//LCD_ClearScreen();
			//LCD_DisplayString(1,Array);
			unsigned char c = 0;
			for(c=0;c<(N/2);c++) {
				LCD_Cursor(c);
				LCD_WriteData(Array[c]);
			}
			yo = 0;
			printNow = 0;
		}
		
		toggle = (~PINA & 0x02);
	}

	
	TimerSet(100);
	TimerOn();
	return 0;
}
//--------END Frequency Spectrum Displayer ------------------------------------------

//Fourier Transform Function
//adapted from http://blog.vinu.co.in/2012/05/implementing-discrete-fourier-transform.html
void TRANSFORM()
{
	short count,degree;
	unsigned char u,k;
	count = 0;
	for (u=0; u<N/2; u++) {
		for (k=0; k<N; k++) {
			degree = (unsigned short)pgm_read_byte_near(degree_lookup + count)*2;
			count++;
			FourierOut[u][0] +=  FourierInput[k] * (short)pgm_read_word_near(cos_lookup + degree);
			FourierOut[u][1] += -FourierInput[k] * (short)pgm_read_word_near(sin_lookup + degree);
		}
		FourierOut[u][0] /= N;
		FourierOut[u][0] /= 10000;
		FourierOut[u][1] /= N;
		FourierOut[u][1] /= 10000;
	}
}

//Timer for Fourier Transform
void timer1_init()
{
	TCCR1B = 0x0B;
	OCR1A = 125;
}

//ADC init for Fourier Transform
void adc_init()
{
	ADMUX = 0b11000000;
	ADCSRA =0b10000010;
}

//ADC read for Fourier Transform
unsigned short adc_read()
{
	volatile unsigned short adcLow,adcHi;
	ADCSRA |= 1<<ADSC; //Starting conversion
	while(!ADIF); //While interrupt flag is not set
	ADCSRA |= 1<<ADIF; //Set interrupt Flag
	adcLow = ADCL;  //ADC low
	adcHi = ADCH;  //ADC high
	adcHi <<=8;     //Shift ADC high by 8 bits (Set to upper nibble of short)
	adcHi |= adcLow; //OR with ADC low
	return adcHi;  //Return ORed ADC value
}

unsigned funcNum = 0;

enum ToggleStates {Toggle_Start, Toggle_Wait, Toggle_Pressed, Toggle_Released, Toggle_Released_Debounce} ToggleState;

//State machine to Toggle button presses to toggle between views
void ToggleButton_TckFct() {
	switch(ToggleState) {
		case Toggle_Start:
			ToggleState = Toggle_Wait;
			break;
		case Toggle_Wait:
			if (toggle) {
				ToggleState = Toggle_Pressed;
			}
			else {
				ToggleState = Toggle_Wait;
			}
			break;
		case Toggle_Pressed:
			if (toggle) {
				ToggleState = Toggle_Pressed;
			}
			else {
				ToggleState = Toggle_Released;
			}
			break;
		case Toggle_Released:
			if (toggle) {
				ToggleState = Toggle_Wait;
			}
			else {
				ToggleState = Toggle_Released_Debounce;
			}
			break;
		case Toggle_Released_Debounce:
			if (toggle) {
				ToggleState = Toggle_Wait;
			}
			else {
				ToggleState = Toggle_Released_Debounce;
			}
			break;
		default:
			ToggleState = Toggle_Start;
			break;
	}

	switch(ToggleState) {
		case Toggle_Start:
			funcNum = 0;
			break;
		case Toggle_Released:
			if (funcNum) {
				funcNum = 0;
			}
			else {
				funcNum = 1;
			}
			break;
		default:
			break;
	}
}

//Main Controller to toggle between Loudness Viewer and Frequency Viewer Controllers
int main(void) {
	DDRA = 0x00; PORTA = 0xFF;

	DDRB = 0xFF; PORTB = 0x00; // LCD data lines
	DDRC = 0xFF; PORTC = 0x00; // LCD control lines

	ToggleState = Toggle_Start;

	TimerSet(100);
	TimerOn();

	// Initializes the 16x2 LCD display
	LCD_init();

	funcNum = 0;

	while (1) {
		toggle = (~PINA & 0x02);

		ToggleButton_TckFct();

		if (funcNum == 0) {
			LCD_ClearScreen();
			decibelShow();
		}
		else if (funcNum == 1) {
			TimerOff();
			FreqSpec_Disp();
		}
		else {
			LCD_DisplayString(1,"Error");
		}

		while(!TimerFlag);
		TimerFlag = 0;
	}
}



