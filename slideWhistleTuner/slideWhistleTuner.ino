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

arduinoFFT FFT;

// analog pins
const unsigned int MIC_PIN = 0; // the Arduino analog pin for the mic
const unsigned int POT_PIN = 1; // the Arduino analog pin for the potentiometer

// timekeeping and indexing
unsigned long sampleBirthday; // the micros() time when the last sample was recorded
unsigned int i = 0; // index of sample to record

// FFT config
const unsigned int SAMPLES = 128; // must be a base 2 number; max 128 for Arduino Uno
const unsigned int SAMPLING_FREQ = 2048; // must be twice the highest expected frequency
const unsigned int SAMPLING_PERIOD = round(1000000*(1.0/SAMPLING_FREQ)); // sampling period in microseconds

// FFT working vars
double vReal[SAMPLES]; // create vector of size SAMPLES to hold real values
double vRealTmp[SAMPLES]; // temporary one to build things in
bool vRealIsNew = false; // indicates when to recalculate the FFT
double vImag[SAMPLES]; // empty; should initialize as 0s and remain as such
double peakPitch; // MIDI pitch of highest peak

void setup() {
	// TODO configure Arduino ports
	
	Serial.begin(9600); // TODO remove for faster loop speed?
	
	sampleBirthday = micros(); // pretend we just took a sample (to start the loop correctly)
}

void loop() {
	
	// get samples as needed
	if (i < SAMPLES) { // if we've collected at most SAMPLES samples so far (i==0 means we have 1 sample)
		
		// wait for a sampling period to have passed
		if (micros() - sampleBirthday >= SAMPLING_PERIOD) {
			
			vRealTmp[i] = analogRead(MIC_PIN); // take a sample in tmp var
			sampleBirthday = micros(); // save sample birthday to the microsecond
			i++; // increment i because we've taken a sample
			
		}
		
	} else {
		
		vReal = vRealTmp; // copy from tmp var to main var
		vRealIsNew = true;
		i = 0; // go back to recording samples
		
	}
	
	if (vRealIsNew) {
		vRealIsNew = false;
		// do FFT on snippet
		// use hamming windowing to get narrower pitches with more frequency bleed
		// use hann    windowing to get wider pitches and less frequency bleed
		FFT = arduinoFFT(vReal, vImag, SAMPLES, SAMPLING_FREQ);
		FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
		FFT.Compute(FFT_FORWARD);
		FFT.ComplexToMagnitude();
		
		// find dominant MIDI pitch (formula from https://newt.phys.unsw.edu.au/jw/notes.html)
		peakPitch = 12.0 * log2(FFT.MajorPeak() / 440.0) + 69;
		
		// // print dominant MIDI pitch
		// Serial.print("Dominant MIDI Pitch: ");
		// Serial.println(peakPitch);
		
		// find correction amount
		corr = findCorrection(peakPitch, str)
		
		// convert to servo angle
	}
	
	
}

double findCorrection(double pitchIn, double strength) {
	// TODO check if pitch is in a reasonable range for better reliability
	
}