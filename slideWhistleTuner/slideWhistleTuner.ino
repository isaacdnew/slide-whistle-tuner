/*

Original Author: Clyde A. Lettsome, PhD, PE, MEM
Adapted by Isaac Newcomb

Description: This code/sketch makes displays the approximate frequency of the loudest sound detected by a sound detection module. For this project, the analog output from the
sound module detector sends the analog audio signal detected to A0 of the Arduino Uno. The analog signal is sampled and quantized (digitized). A Fast Fourier Transform (FFT) is
then performed on the digitized data. The FFT converts the digital data from the approximate discrete-time domain result. The maximum frequency of the approximate discrete-time
domain result is then determined and displayed via the Arduino IDE Serial Monitor.

Note: The arduinoFFT.h library needs to be added to the Arduino IDE before compiling and uploading this script/sketch to an Arduino.

License: This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License (GPL) version 3, or any later
version of your choice, as published by the Free Software Foundation.

Notes: Copyright (c) 2019 by C. A. Lettsome Services, LLC
For more information visit https://clydelettsome.com/blog/2019/12/18/my-weekend-project-audio-frequency-detector-using-an-arduino/

*/

#include "arduinoFFT.h"

arduinoFFT FFT = arduinoFFT();

const unsigned int SAMPLES = 128; // must be a base 2 number; max 128 for Arduino Uno
const unsigned int SAMPLING_FREQ = 2048; // must be twice the highest expected frequency
const unsigned int SAMPLING_PERIOD = round(1000000*(1.0/SAMPLING_FREQ)); // sampling period in microseconds

const unsigned int MIC_PIN = 0; // the analog pin on the arduino that's connected to the mic

unsigned long sampleBirthday; // the micros() time when the last sample was recorded
unsigned int i = 0; // how many sample

double vReal[SAMPLES]; // create vector of size SAMPLES to hold real values
double vRealTmp[SAMPLES]; // temporary one to build things in
double vImag[SAMPLES]; // empty; should initialize as 0s

void setup() {
	Serial.begin(9600); // TODO remove for speed?
}

void loop() {
	
	// Get Samples!
	if (i < SAMPLES) { // if we've collected at most SAMPLES samples so far (i==127 means we have 128 total samples)
		
		// wait for a sampling period to have passed
		if (micros() - sampleBirthday >= SAMPLING_PERIOD) {
			
			vRealTmp[i] = analogRead(MIC_PIN); // take a sample in tmp var
			sampleBirthday = micros(); // save sample birthday to the microsecond
			i++; // increment i because we've taken a sample
			
		}
		
	} else {
		
		vReal = vRealTmp; // copy from tmp var to main var
		i = 0; // go back to recording samples
		
	}
	
	// do FFT on snippet
	FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
	FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
	FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);

	// find dominant frequency and print it
	double peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQ);
	Serial.print("peak frequency: ");
	Serial.println(peak);
}
