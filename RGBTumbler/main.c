/*
 *  main.c
 *
 *  Created by Jens Willy Johannsen on 12-11-11.
 *  Copyright (c) 2011 Jens Willy Johannsen
 *	This work is licensed under the Creative Commons Attribution-ShareAlike 3.0 Unported License. 
 *
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include "calibration.h"
#include <avr/eeprom.h>
#include <util/delay.h>
#include <math.h>

// Macro for finding maximum of three values
#define MAX3(x,y,z)	((y) >= (z) ? ((x) >= (y) ? (x) : (y)) : ((x) >= (z) ? (x) : (z)))

// Macro for clamping values to accepted interval
#define CLAMP(x, l, h)  (((x) > (h)) ? (h) : (((x) < (l)) ? (l) : (x)))

// Smoothing value for low-pass filter
#define ALPHA 0.2

// Minimum OCRxx value. If value is below this, the LED will be switched off
#define LED_MIN 5
#define LED_MINCOUNT 12

/* Snapping macro
 * If the counter (OCRxx) value is below the threshold, the corresponding LED is turned off and PWM output is disabled, 
 * the samples-above-threshold counter is reset and the stateOn is set to 0 (false).
 * Every time we get a value below the threshold, the counter is reset to 0.
 * If the value is above the threshold (and it doesn't matter if the OCRxx is set to a higher value since the PWM output has been disabled),
 * the above-threshold counter is increased by 1.
 * When the above-threshold counter is above the required count (LED_MINCOUNT), PWM output is enabled again and the stateOn is set to 1 (true).
 *
 * In other words: as soon as we get a sample value below the threshold, the LED is turned off. We will then need LED_MINCOUNT *consecutive*
 * samples above the threshold for the LED to be turned on again.
 */
#define SNAP( counter_, stateCounter_, stateOn_, TCCR_, COM_, PORT_, BIT_ ) if( counter_ < LED_MIN ) \
{ \
stateCounter_ = 0; \
if( stateOn_ ) \
{ \
TCCR_ &= ~(1<< COM_); \
PORT_ &= ~(1<< BIT_); \
stateOn_ = 0; \
} \
} \
if( !stateOn_ && counter_ > LED_MIN ) \
{ \
stateCounter_++; \
if( stateCounter_ >= LED_MINCOUNT ) \
{ \
TCCR_ |= (1<< COM_); \
stateOn_ = 1; \
} \
}

uint8_t stateROn=1;
uint8_t stateRCnt=0;
uint8_t stateGOn=1;
uint8_t stateGCnt=0;
uint8_t stateBOn=1;
uint8_t stateBCnt=0;

// State var for calibrate/normal mode
uint8_t EEMEM eeprom_calibrate;

// Sensor calibration values m=midpoint, s=sensitivity
float EEMEM eeprom_mx;
float EEMEM eeprom_my;
float EEMEM eeprom_mz;
float EEMEM eeprom_sx;
float EEMEM eeprom_sy;
float EEMEM eeprom_sz;
extern float beta[6];	// declared in calibration.c

float mx = 399.9272155;
float my = 638.5637817;
float mz = 517.5350341;
float sx = 0.0095106;
float sy = 0.0094931;
float sz = 0.0096375;

// output for X, Y and Z (OCR values 0-255)
float prevX;
float prevY;
float prevZ;

// Low-pass filter
float lowPass( float x, float y, float alpha )
{
	return y + alpha * (x-y);
}

// Watchdog timer interrupt: main analog-to-digital conversion and acceleration-to-intensity calculation takes place here
ISR( WDT_vect )
{
	// Get ADC values
	uint16_t x, y, z;
	float tmp;
	
	// AD channel 0 (PC0)
	ADMUX = (ADMUX & 0xFC) | 0b00;			// Mask bits 7:2 so we don't change them. And set bits 1:0 to active channel
	ADCSRA |= (1<<ADSC);					// Start single conversion
	while( ADCSRA & (1<<ADSC) ) ;			// Wait until conversion is done
	x = ADC;
	
	// AD channel 1 (PC1)
	ADMUX = (ADMUX & 0xFC) | 0b01;
	ADCSRA |= (1<<ADSC);					// Start single conversion
	while( ADCSRA & (1<<ADSC) )	;			// Wait until conversion is done
	y = ADC;
	
	// AD channel 2 (PC2)
	ADMUX = (ADMUX & 0xFC) | 0b10;
	ADCSRA |= (1<<ADSC);					// Start single conversion
	while( ADCSRA & (1<<ADSC) ) ;			// Wait until conversion is done
	z = ADC;
	
	// Set PWM values

	// Green=X – OCR0A/PD6
	tmp = ((float)x - mx) * sx;				// Calculate force in Gs
	tmp = CLAMP( tmp, -1, 1 );				// Clamp to +/- 1G
	prevX = lowPass( tmp, prevX, ALPHA );	// Low-pass
	OCR0A = (uint8_t)(fabs(prevX) * 255);	// Convert to absolute and then to 0-255. I.e. -1 G is 100% and +1 G is 100% and 0 G is 0%
	SNAP( OCR0A, stateGCnt, stateGOn, TCCR0A, COM0A1, PORTD, PD6 );

	// Blue=Y – OCR0B/PD5
	tmp = ((float)y - my) * sy;				// Calculate force in Gs
	tmp = CLAMP( tmp, -1, 1 );				// Clamp to +/- 1G
	prevY = lowPass( tmp, prevY, ALPHA );	// Low-pass
	OCR0B = (uint8_t)(fabs(prevY) * 255);
	SNAP( OCR0B, stateBCnt, stateBOn, TCCR0A, COM0B1, PORTD, PD5 );
	
	// Red=Z – OCR2A/PB3
	tmp = ((float)z - mz) * sz;				// Calculate force in Gs
	tmp = CLAMP( tmp, -1, 1 );				// Clamp to +/- 1G
	prevZ = lowPass( tmp, prevZ, ALPHA );	// Low-pass
	OCR2A = (uint8_t)(fabs(prevZ) * 255);
	SNAP( OCR2A, stateRCnt, stateROn, TCCR2A, COM2A1, PORTB, PB3 );

	// And we're done – go to sleep again
}

int main(void)
{
	// OC0A, OC0B and OC2A as outputs
	DDRD |= (1<< PD6) | (1<< PD5);
	DDRB |= (1<< PB3);
	
	// Go to calibration mode?
	eeprom_busy_wait();
	uint8_t should_calibrate = eeprom_read_byte( &eeprom_calibrate );
	if( should_calibrate == 1 || should_calibrate == 0xFF )	// 0xFF means uninitialized: so force calibration
	{
		/*
		 * CALIBRATION MODE
		 */

		// Do the calibration stuff using Rolfe's magic math stuff
		calibrate_setup();
		calibrate();
		
		// Write calibration values to EEPROM
		eeprom_write_float( &eeprom_mx, beta[0] );
		eeprom_write_float( &eeprom_my, beta[1] );
		eeprom_write_float( &eeprom_mz, beta[2] );
		eeprom_write_float( &eeprom_sx, beta[3] );
		eeprom_write_float( &eeprom_sy, beta[4] );
		eeprom_write_float( &eeprom_sz, beta[5] );
		
		// Clear calibration mode var
		eeprom_busy_wait();
		eeprom_write_byte( &eeprom_calibrate, 0 );
		
		// Calibration done: keep flashing B LED
		for( ;; )
		{
			// Keep flashing B LED
			PORTD |= (1<< PD5);
			_delay_ms( 200 );
			PORTD &= ~(1<< PD5);
			_delay_ms( 200 );
		}
	}

	// No: don't calibrate. 

	// Wait a little bit let the power switch debounce
	_delay_ms( 50 );
	
	// Set calibrate bit to YES
	eeprom_busy_wait();
	eeprom_write_byte( &eeprom_calibrate, 1 );
	// White light to indicate "switch off now to enter calibration mode"
	PORTD |= (1<< PD6) | (1<< PD5);
	PORTB |= (1<< PB3);
	
	// Wait 1 second
	_delay_ms( 1000 );
	
	// Clear calibration var
	eeprom_busy_wait();
	eeprom_write_byte( &eeprom_calibrate, 0 );
	
	// LED off
	PORTD &= ~(1<< PD5) & ~(1<< PD6);
	PORTB &= ~(1<< PB3);
	_delay_ms( 1000 );
	
	
	/*
	 *	NORMAL MODE
	 */
	
	// Read calibration values
	// Wait until the EEPROM is ready before reading
	eeprom_busy_wait();
	mx = eeprom_read_float( &eeprom_mx );
	my = eeprom_read_float( &eeprom_my );
	mz = eeprom_read_float( &eeprom_mz );
	sx = eeprom_read_float( &eeprom_sx );
	sy = eeprom_read_float( &eeprom_sy );
	sz = eeprom_read_float( &eeprom_sz );

	// Power-down unused stuff
	PRR |= (1<< PRTWI) | (1<< PRTIM1) | (1<< PRSPI) | (1<< PRUSART0);	// Power-down TWI, Timer1, SPI and USART
	ACSR |= (1<< ACD);													// Power-down analog comperator		
	
	/// DEBUG: PB0 for CPU active pin -> output
	DDRB |= (1<< PB0);
	
	// Set up Timer0 for PWM
	TCCR0A = (1<< COM0A1) | (1<< COM0B1) | (1<< WGM01) | (1<< WGM00);	// OC0A and OC0B normal mode; fast PWM
	TCCR0B = (1<< CS01);	// ~3906 Hz
	
	// Set up Timer2 for PWM
	TCCR2A = (1<< COM2A1) | (1<< WGM21) | (1<< WGM20);					// OC2A normal mode; fast PWM
	TCCR2B = (1<< CS21);	// ~3906 Hz
	
	// Set up analog to digital converter
	// By default set to use AREF and single-conversion mode
	ADCSRA |= (1<< ADPS2) | (1<< ADPS1);	// Set prescaler 64 for 125 kHz @ 8 MHz system clock	
	
	// Disable digital input buffers for all ADC pins. (We're only using 0-2 but since we aren't using
	// the other pins for anything, we might as well turn them off to save a few nA there as well.
	DIDR0 |= (1<< ADC5D) | (1<< ADC4D) | (1<< ADC3D) | (1<< ADC2D) | (1<< ADC1D) | (1<< ADC0D);
	ADCSRA |= (1<< ADEN);	// Enable ADC
	
	// Set up Watchdog timer for 16 ms timeout. That is 62.5 Hz which is slightly above the
	// the accelerometer sampling frequency so we won't loose many samples
	WDTCSR |= (1<< WDIE);	// Interrupt enabled, 2048 clock cycles (for 16 ms)
	sei();					// Interrupts enabled
	
	// Main loop: do nothing but sleep since everything happens in the Watchdog interrupt
	for( ;; )
	{
		// Enter idle mode. AD conversion and PWM adjust will take place in Watchdog interrupt
		SMCR = (1<< SE);	// Sleep enable, idle mode
		asm("sleep");		// Nighty, night, CPU
	}

    return 0;
}
