//
//  calibration.c
//  PWM
//
//  Created by Jens Willy Johannsen on 15-11-11.
//  Copyright (c) 2011 Greener Pastures. All rights reserved.
//
//	This is based on Rolfe Schmidt's work: http://chionophilous.wordpress.com/
//	Truckloads of street cred and praise go to the snow-happy man.
//
//	Adapted from Arduino sketch to AVR Libc code by Jens W. Johannsen
//	Accelerometer is assumed to be connected as follows:
//		X-axis	ACD0
//		Y-axis	ACD1
//		Z-axis	ACD2
//	And the AREF must be connected to the ADXL335's supply voltage
//	
//	Three LEDs (R, G, B) are assumed to be connected as follows:
//		R	PD6
//		G	PD5
//		B	PB3
//	And they should already be configured as outputs
//

/*
 # copyright 2011 by Rolfe Schmidt
 # This work is licensed under the Creative Commons Attribution-ShareAlike 3.0 Unported License. 
 # To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/3.0/ or send a
 # letter to Creative Commons, 444 Castro Street, Suite 900, Mountain View, California, 94041, USA.
 #
 # Available at http://rolfeschmidt.com/mathtools/skimetrics/adxl_gn_calibration.pde
 #
 # Described at http://wp.me/p1CQVj-1k
 */

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include "calibration.h"

const int NUM_READINGS = 6;
// these constants describe the AD channels
const uint8_t xpin = 0;                  // x-axis of the accelerometer
const uint8_t ypin = 1;                  // y-axis
const uint8_t zpin = 2;                  // z-axis (only on 3-axis models)

//data collection structures
unsigned int *data;   //dynamically allocated array for data storage.  Don't let it get too big!
int samp_capacity = 0;             //the capacity of the sample data array
int n_samp = 0;                    // Number of samples used for calibration
int sample_size=32;                // Number of measurements averaged to produce 1 sample
float beta[6];                        //parameters for model.  beta[0], beta[1], and beta[2] are the 0-G marks (about 512),
// while beta[3], beta[4], and beta[5] are the scaling factors.  So, e.g., if xpin reads
// value x, number of G's in the x direction in beta[3]*(x - beta[0]).


//matrices for Gauss-Newton computations
float JS[6][6];
float dS[6];
float delta[6];


void calibrate_model_matrices();
void find_delta();
void calibrate_model();
void reset_calibration_matrices();

uint16_t analogRead( uint8_t AD_channel )
{
	ADMUX = (ADMUX & 0xF0) | ( 0x0F & AD_channel);	// Mask bits 7:4 so we don't change them. And set bits 3:0 to active channel
	ADCSRA |= (1<<ADSC);							// Start single conversion
	while( ADCSRA & (1<<ADSC) )	;					// Wait until conversion is done

	return ADC;										// Return value
}

void calibrate_setup()
{
	// Set up analog to digital converter
	// By default set to use AREF and single-conversion mode
	ADCSRA |= (1<< ADPS2) | (1<< ADPS1) | (1<< ADEN);	// Set prescaler 64 for 125 kHz @ 8 MHz system clock and enable

	//initialize to handle 20 samples
	samp_capacity = 20;
	data = (unsigned int*)malloc(samp_capacity*3*sizeof(unsigned int));
	
	reset_calibration_matrices();
	
	//initialize beta to something reasonable
	beta[0] = beta[1] = beta[2] = 512.0;
	beta[3] = beta[4] = beta[5] = 0.0095; 

	// Flash RGB pattern to indicate "Calibration mode"
	PORTD |= (1<< PD6);
	_delay_ms( 100 );
	PORTD &= ~(1<< PD6);
	
	PORTD |= (1<< PD5);
	_delay_ms( 100 );
	PORTD &= ~(1<< PD5);

	PORTB |= (1<< PB3);
	_delay_ms( 100 );
	PORTB &= ~(1<< PB3);

	// Wait for the slow human to pay attention
	_delay_ms( 1000 );

	// Ready to calibrate
}

void calibrate()
{
	int i;
	
	// Has 6 readings been taken?
	while( n_samp < 6 )
	{
		// No: take another
		// Flash B LED for four seconds to allow the user to position the sensor
		for( i=0; i<4; i++ )
		{
			PORTB |= (1<< PB3);
			_delay_ms( 500 );
			PORTB &= ~(1<< PB3);
			_delay_ms( 500 );
		}
		
		// The sensor is assumbed to be positioned now so take the sample
		take_sample(data + 3*(n_samp%samp_capacity));
		n_samp++;
	}
	
	// 6 readings have been taken – calculate calibration values
	calibrate_model();
	
	// Done: store values in EEPROM when returning from here
}

//pass in a length-3 array where the data will be written
void take_sample(unsigned int* sample_out) {
	int i=0;
	int first_pass_size = 5;
	int success = 0;
	
	
	while (success == 0) {
		//First, run through 32 samples and accumulate the mean and variance.
		//Make all variables longs because we will do some aritmetic that 
		// will overflow an int.
		unsigned long sum[] = {0,0,0};
		unsigned long sum_squares[] = {0,0,0};
		unsigned long variance[] = {0,0,0};
		unsigned long x,y,z;
		for(i=0;i< (1<<first_pass_size);++i) {
			x= analogRead(xpin);
			_delay_ms(1);
			y= analogRead(ypin);
			_delay_ms(1);
			z= analogRead(zpin);
			_delay_ms(18); //sample at 50 Hz
			
			sum[0] += x;
			sum[1] += y;
			sum[2] += z;
			
			sum_squares[0] += x*x;
			sum_squares[1] += y*y;
			sum_squares[2] += z*z;
		}
		
		//now compute the variance onh each axis. Don't divide by 32 -- keep the extra digits
		//around to reduce quantization errors.  So really computing variance*32 here. Also make sure it is > 0.
		for(i=0;i<3; i++) {
			variance[i] = 1+ sum_squares[i] - (sum[i]*sum[i])/32;
		}
		
		
		//with mean and variance in place, start collecting real samples but filter out outliers.
		//Track the success rate and start over if we get too many fails.
		unsigned int success_count = 0;
		unsigned int fail_count = 0;
		i=0;
		sample_out[0] = sample_out[2] = sample_out[1] = 0;
		
		
		while(i < sample_size) {
			
			//take a reading
			x= analogRead(xpin);
			_delay_ms(1);
			y= analogRead(ypin);
			_delay_ms(1);
			z= analogRead(zpin);
			_delay_ms(18); //sample at 50 Hz
			
			unsigned long dx = x*32 - sum[0];
			unsigned long dy = y*32 - sum[1];
			unsigned long dz = z*32 - sum[2];
			
			//check to see if it is any good (within 3 std deviations)
			if((dx*dx)/32 < 9*variance[0]
			   &&(dy*dy)/32 < 9*variance[1]
			   &&(dz*dz)/32 < 9*variance[2]) {
				success_count++;
				sample_out[0] += x;
				sample_out[1] += y;
				sample_out[2] += z;
				
				++i;
			} else {        
				fail_count++;
			}
			
			if(fail_count > success_count && i > 10) {
				// we're failing too much, start over!
				// Flash R LED twice times to indicate "bad sample"
				PORTD |= (1<< PD6);
				_delay_ms( 100 );
				PORTD &= ~(1<< PD6 );
				_delay_ms( 100 );
				PORTD |= (1<< PD6);
				_delay_ms( 100 );
				PORTD &= ~(1<< PD6 );
				_delay_ms( 100 );
				break;
			} 
			
		}
		
		
		//if we got our samples, mark the success.  Otherwise we'll start over.
		if (i == sample_size) {
			success = 1;
			
			// Flash G LED twice to indicate "good sample"
			PORTD |= (1<< PD5);
			_delay_ms( 200 );
			PORTD &= ~(1<< PD5 );
			_delay_ms( 200 );
			PORTD |= (1<< PD5);
			_delay_ms( 200 );
			PORTD &= ~(1<< PD5 );
			_delay_ms( 500 );
		}
	}
}

//Gauss-Newton functions

void reset_calibration_matrices() {
	int j,k;
    for(j=0;j<6;++j) {
		dS[j] = 0.0;
		for(k=0;k<6;++k) {
			JS[j][k] = 0.0;
		}
    }
}

void update_calibration_matrices(const unsigned int* data) {
    int j, k;
    float dx, b;
    float residual = 1.0;
    float jacobian[6];
    
    for(j=0;j<3;++j) {
		b = beta[3+j];
		dx = ((float)data[j])/sample_size - beta[j];
		residual -= b*b*dx*dx;
		jacobian[j] = 2.0*b*b*dx;
		jacobian[3+j] = -2.0*b*dx*dx;
    }
    
    for(j=0;j<6;++j) {
		dS[j] += jacobian[j]*residual;
		for(k=0;k<6;++k) {
			JS[j][k] += jacobian[j]*jacobian[k];
		}
    }
}

void compute_calibration_matrices() {
    int i; //, j, k;
//    float dx, b;
	
    reset_calibration_matrices();
    int ub = n_samp < samp_capacity ? n_samp : samp_capacity;
    for(i=0;i<ub;i++) {    
		update_calibration_matrices(data+3*i);
    }
}

void find_delta() {
	//Solve 6-d matrix equation JS*x = dS
	//first put in upper triangular form
	int i,j,k;
	float mu;
	
	//make upper triangular
	for(i=0;i<6;++i) {
		//eliminate all nonzero entries below JS[i][i]
		for(j=i+1;j<6;++j) {
			mu = JS[i][j]/JS[i][i];
			if(mu != 0.0) {
				dS[j] -= mu*dS[i];
				for(k=j;k<6;++k) {
					JS[k][j] -= mu*JS[k][i];
				} 
			}
		}
	}
	
	//back-substitute
	for(i=5;i>=0;--i) {
		dS[i] /= JS[i][i];
		JS[i][i] = 1.0;
		for(j=0;j<i;++j) {
			mu = JS[i][j];
			dS[j] -= mu*dS[i];
			JS[i][j] = 0.0;
		}
	}
	
	for(i=0;i<6;++i) {
		delta[i] = dS[i];
	}
}

void calibrate_model() {
	int i;
	float eps = 0.000000001;
	int num_iterations = 20;
	float change = 100.0;
	while (--num_iterations >=0 && change > eps) {
		compute_calibration_matrices();
		find_delta();
		change = delta[0]*delta[0] + delta[0]*delta[0] + delta[1]*delta[1] + delta[2]*delta[2] + delta[3]*delta[3]/(beta[3]*beta[3]) + delta[4]*delta[4]/(beta[4]*beta[4]) + delta[5]*delta[5]/(beta[5]*beta[5]); 
		
		for(i=0;i<6;++i) {
			beta[i] -= delta[i];
		}
		
		reset_calibration_matrices();
	}
}
