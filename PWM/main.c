/*
 *  main.c
 *  PWM
 *
 *  Created by Jens Willy Johannsen on 12-11-11.
 *  Copyright Greener Pastures 2011. All rights reserved.
 *
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include "calibration.h"
#include <avr/eeprom.h>
#include <util/delay.h>

// Macro for finding maximum of three values
#define MAX3(x,y,z)	((y) >= (z) ? ((x) >= (y) ? (x) : (y)) : ((x) >= (z) ? (x) : (z)))

// Macro for clamping values to accepted interval
#define CLAMP(x, l, h)  (((x) > (h)) ? (h) : (((x) < (l)) ? (l) : (x)))

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

// Watchdog timer interrupt
ISR( WDT_vect )
{
	/// DEBUG: CPU active
	PORTB |= (1<< PB0);
	
	// Get ADC values
	uint16_t x, y, z;
	float tmp;
	
	// AD channel 0 (PC0)
	ADMUX &= 0xFC | 0b00;			// Mask bits 7:2 so we don't change them. And set bits 1:0 to active channel
	ADCSRA |= (1<<ADSC);			// Start single conversion
	while( ADCSRA & (1<<ADSC) )		// Wait until conversion is done
		;
	x = ADC;
	
	// AD channel 1 (PC1)
	ADMUX &= 0xFC | 0b01;
	ADCSRA |= (1<<ADSC);			// Start single conversion
	while( ADCSRA & (1<<ADSC) );	// Wait until conversion is done
	y = ADC;
	
	// AD channel 2 (PC2)
	ADMUX &= 0xFC | 0b10;
	ADCSRA |= (1<<ADSC);			// Start single conversion
	while( ADCSRA & (1<<ADSC) );	// Wait until conversion is done
	z = ADC;
	
	// Calc proportions
	uint16_t max_value = MAX3( x, y, z );
	/// TEMP: set max to max 10 bit input
	max_value = 0x3FF;
	
	// Set PWM values

	// R=X
	tmp = ((float)x - mx) * sx;				// Calculate force in Gs
	tmp = CLAMP( tmp, -1, 1 );				// Clamp to +/- 1G
	OCR0A = (uint8_t)(((tmp+1)/2) * 255);	// Convert -1 to 1 -> 0 to 255
	
	// G=Y
	tmp = ((float)y - my) * sy;				// Calculate force in Gs
	tmp = CLAMP( tmp, -1, 1 );				// Clamp to +/- 1G
	OCR0B = (uint8_t)(((tmp+1)/2) * 255);	// Convert -1 to 1 -> 0 to 255
	
	// B=Z
	tmp = ((float)z - mz) * sz;				// Calculate force in Gs
	tmp = CLAMP( tmp, -1, 1 );				// Clamp to +/- 1G
	OCR2B = (uint8_t)(((tmp+1)/2) * 255);	// Convert -1 to 1 -> 0 to 255
	
	// And we're done – go to sleep again
}

int main(void)
{
	// Go to calibration mode?
	uint8_t should_calibrate = eeprom_read_byte( &eeprom_calibrate );
	if( should_calibrate == 1 || should_calibrate == 0xFF )	// 0xFF means uninitialized: so force calibration
	{
		// Yes: calibrate
		calibrate_setup();
		calibrate();
		
		eeprom_write_float( &eeprom_mx, beta[0] );
		eeprom_write_float( &eeprom_my, beta[1] );
		eeprom_write_float( &eeprom_mz, beta[2] );
		eeprom_write_float( &eeprom_sx, beta[3] );
		eeprom_write_float( &eeprom_sy, beta[4] );
		eeprom_write_float( &eeprom_sz, beta[5] );
		
		// Calibration done: keep flashing G LED
		for( ;; )
		{
			// Keep flashing G LED
			PORTD |= (1<< PD5);
			_delay_ms( 200 );
			PORTD &= ~(1<< PD5);
			_delay_ms( 200 );
		}
	}
	
	// No: don't calibrate. 
	// Set calibrate bit to YES
	eeprom_write_byte( &eeprom_calibrate, 1 );
	
	// Wait 1 second
	_delay_ms( 1000 );
	
	// Clear calibration var
	eeprom_write_byte( &eeprom_calibrate, 0 );
	
	// Continue in normal mode:
	
	// Read calibration values
	mx = eeprom_read_float( &eeprom_mx );
	my = eeprom_read_float( &eeprom_my );
	mz = eeprom_read_float( &eeprom_mz );
	sx = eeprom_read_float( &eeprom_sx );
	sy = eeprom_read_float( &eeprom_sy );
	sz = eeprom_read_float( &eeprom_sz );
	
	// Power-down unused stuff
	PRR |= (1<< PRTWI) | (1<< PRTIM1) | (1<< PRSPI) | (1<< PRUSART0);	// Power-down TWI, Timer1, SPI and USART
	ACSR |= (1<< ACD);														// Power-down analog comperator		
	
	// OC0A, OC0B and OC2A as outputs
	DDRD |= (1<< PD6) | (1<< PD5);
	DDRB |= (1<< PB3);
	
	/// DEBUG: PB0 for CPU active pin -> output
	DDRB |= (1<< PB0);
	
	// Set up Timer0 for PWM
	TCCR0A = (1<< COM0A1) | (1<< COM0B1) | (1<< WGM01) | (1<< WGM00);	// OC0A and OC0B normal mode; fast PWM
	TCCR0B = (1<< CS01);	// ~3906 Hz
	
	// Set up Timer2 for PWM
	TCCR2A = (1<< COM2A1) | (1<< WGM21) | (1<< WGM20);		// OC2A normal mode; fast PWM
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
	
	// Main loop
	for( ;; )
	{
		// Enter idle mode. AD conversion and PWM adjust will take place in Watchdog interrupt
		SMCR = (1<< SE);	// Sleep enable, idle mode
		/// DEBUG: CPU off
		PORTB &= ~(1<< PB0);
		asm("sleep");		// Nighty, night, CPU
	}

    return 0;
}
