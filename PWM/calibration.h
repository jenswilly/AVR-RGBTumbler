//
//  calibration.h
//  PWM
//
//  Created by Jens Willy Johannsen on 15-11-11.
//  Copyright (c) 2011 Greener Pastures. All rights reserved.
//

#ifndef PWM_calibration_h
#define PWM_calibration_h

uint16_t analogRead( uint8_t AD_channel );
void calibrate_setup();
void calibrate();
void take_sample(unsigned int* sample_out);

//Gauss-Newton functions
void reset_calibration_matrices();
void update_calibration_matrices(const unsigned int* data);
void compute_calibration_matrices();
void find_delta();
void calibrate_model();

#endif
