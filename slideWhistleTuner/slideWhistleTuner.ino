/*

Original Author: Clyde A. Lettsome, PhD, PE, MEM
Adapted significantly by Isaac Newcomb

Description: This code/sketch makes displays the approximate frequency of the loudest sound detected by a sound detection module. For this project, the analog output from the
sound module detector sends the analog audio signal detected to A0 of the Arduino Uno. The analog signal is sampled and quantized (digitized). A Fast Fourier Transform (FFT) is
then performed on the digitized data. The FFT converts the digital data from the approximate discrete-time domain result. The maximum frequency of the approximate discrete-time
domain result is then determined and displayed via the Arduino IDE Serial Monitor.

Note: The arduinoFFT.h library needs to be added to the Arduino IDE before compiling and uploading this script/sketch to an Arduino.

License: This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License (GPL) version 3, or any later
version of your choice, as published by the Free Software Foundation.

Notes: Copyright (c) 2019 by C. A. Lettsome Services, LLC. Revisions Copyright (c) 2021 Isaac Newcomb.
For more information visit https://clydelettsome.com/blog/2019/12/18/my-weekend-project-audio-frequency-detector-using-an-arduino/

*/

#include "arduinoFFT.h"
#include <Servo.h>

arduinoFFT FFT;

// analog pins
const int MIC_PIN = 0;
const int POT_PIN = 1;
const int SERVO_PIN = 2;

// timekeeping and indexing
long sampleBirthday; // the micros() time when the last sample was recorded
int i = 0; // index of sample to record

// FFT config
const int SAMPLES = 128; // must be a base 2 number; max 128 for Arduino Uno
const int SAMPLING_FREQ = 2048; // must be twice the highest expected frequency
const int SAMPLING_PERIOD = round(1000000*(1.0/SAMPLING_FREQ)); // sampling period in microseconds

// FFT working vars
double vReal[SAMPLES]; // create vector of size SAMPLES to hold real values
double vImag[SAMPLES]; // empty; should initialize as 0s and remain as such
bool snippetIsReady = false; // indicates when to recalculate the FFT

// pitches to correct to - select by (un)commenting
// C Major
const int PITCH_SET_LG = 29;
const int PITCH_SET[PITCH_SET_LG] =
{
	48, 50, 52, 53, 55, 57, 59,
	60, 62, 64, 65, 67, 69, 71,
	72, 74, 76, 77, 79, 81, 83,
	84, 86, 88, 89, 91, 93, 95, 96
};

// C Minor
// const int PITCH_SET_LG = 29;
// const int PITCH_SET[PITCH_SET_LG] =
// {
// 	48, 50, 51, 53, 55, 56, 58,
// 	60, 62, 63, 65, 67, 68, 70,
// 	72, 74, 75, 77, 79, 80, 82,
// 	84, 86, 87, 89, 91, 92, 94, 96
// };

// Chromatic
// const int PITCH_SET_LG = 49;
// const int PITCH_SET[PITCH_SET_LG] =
// {
// 	48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59,
// 	60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71,
// 	72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83,
// 	84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96
// };

void setup()
{
	// TODO configure Arduino ports
	pinMode(MIC_PIN, INPUT);
	pinMode(POT_PIN, INPUT);
	
	Serial.begin(9600); // TODO remove for faster loop speed?
	
	sampleBirthday = micros(); // pretend we just took a sample (to start the loop correctly)
}

void loop()
{
	if (i < SAMPLES) // get samples if we don't have all of them yet
	{
		// wait for a sampling period to have passed
		if (micros() - sampleBirthday >= SAMPLING_PERIOD)
		{
			vReal[i] = analogRead(MIC_PIN); // take a sample in tmp var
			sampleBirthday = micros(); // save sample birthday to the microsecond
			i++; // increment i because we've taken a sample
		}
	}
	else
	{
		i = 0; // run this just once; go back to recording samples on next loop
		
		// TODO: does object construction take longer than the (deprecated) alternative?
		
		// do FFT on snippet of samples
		// use hamming windowing to get narrower pitches with more frequency bleed
		// use hann    windowing to get wider pitches and less frequency bleed
		FFT = arduinoFFT(vReal, vImag, SAMPLES, SAMPLING_FREQ);
		FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
		FFT.Compute(FFT_FORWARD);
		FFT.ComplexToMagnitude();
		
		// find dominant MIDI pitch (formula from https://newt.phys.unsw.edu.au/jw/notes.html)
		double peakPitch = 12.0 * log(FFT.MajorPeak()/440.0) / log(2.0) + 69;
		
		// find correction amount
		double corr = findCorrection(peakPitch, analogRead(POT_PIN));
		
		// convert to servo angle
		
	}
	
}

double findCorrection(double pitchIn, double strength)
{
	// TODO check if pitch is in a reasonable range for better reliability
	int idx = 0;
	
	double distance = abs(pitchIn - PITCH_SET[idx]);
	
	for (int i = 1; i < PITCH_SET_LG; i++)
	{
		double d = abs(pitchIn - PITCH_SET[i]);
		
		if (d < distance)
		{
			idx = i;
			distance = d;
		}
	}
	
	return strength * (pitchIn - PITCH_SET[idx]);
}