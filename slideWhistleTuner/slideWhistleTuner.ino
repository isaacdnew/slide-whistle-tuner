/*

Original Author: Clyde A. Lettsome, PhD, PE, MEM
Adapted significantly by Isaac Newcomb

This code uses the arduinoFFT library to identify the pitch produced by a slide
whistle, find the nearest pitch in a specified scale, and adjust a servo to move the
slide whistle's slide - thereby correcting the pitch produced. Three labeled LEDs
indicate the pitch correction's direction, if any.

Description of original: This code/sketch makes displays the approximate frequency of
the loudest sound detected by a sound detection module. For this project, the analog 
output from the sound module detector sends the analog audio signal detected to A0 of
the Arduino Uno. The analog signal is sampled and quantized (digitized). A Fast
Fourier Transform (FFT) is then performed on the digitized data. The FFT converts the
digital data from the approximate discrete-time domain result. The maximum frequency
of the approximate discrete-time domain result is then determined and displayed via
the Arduino IDE Serial Monitor.

Note: The arduinoFFT.h library needs to be added to the Arduino IDE before compiling
and uploading this script/sketch to an Arduino.

License: This program is free software; you can redistribute it and/or modify it
under the terms of the GNU General Public License (GPL) version 3, or any later
version of your choice, as published by the Free Software Foundation.

Notes: Copyright (c) 2019 by C. A. Lettsome Services, LLC.
Adaptations Copyright (c) 2021 Isaac Newcomb.
For more information visit https://clydelettsome.com/blog/2019/12/18/my-weekend-project-audio-frequency-detector-using-an-arduino/

*/

#include "arduinoFFT.h"
#include <Servo.h>

// Arduino pins
const int MIC_PIN = 0; // analog input
const int POT_PIN = 1; // analog input
const int SERVO_PIN = 2; // PWM output
const int LED_UP = 3; // digital output
const int LED_GOOD = 4; // digital output
const int LED_DOWN = 5; // digital output


Servo servo;
const double SERVO_ARM_LG = 15.0; // mm

// FFT config
arduinoFFT fft;
const int SAMPLES = 128; // must be a base 2 number; max 128 for Arduino Uno
const int SAMPLING_FREQ = 2048; // must be twice the highest expected frequency
const int SAMPLING_PERIOD = round(1000000*(1.0/SAMPLING_FREQ)); // sampling period in microseconds

// FFT working vars
double vReal[SAMPLES]; // create vector of size SAMPLES to hold real values
double vImag[SAMPLES]; // empty; should initialize as 0s and remain as such
bool snippetIsReady = false; // indicates when to recalculate the FFT
long sampleBirthday; // the micros() time when the last sample was recorded
int i = 0; // index of sample to record

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

// // C Minor
// const int PITCH_SET_LG = 29;
// const int PITCH_SET[PITCH_SET_LG] =
// {
// 	48, 50, 51, 53, 55, 56, 58,
// 	60, 62, 63, 65, 67, 68, 70,
// 	72, 74, 75, 77, 79, 80, 82,
// 	84, 86, 87, 89, 91, 92, 94, 96
// };

// // Chromatic
// const int PITCH_SET_LG = 49;
// const int PITCH_SET[PITCH_SET_LG] =
// {
// 	48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59,
// 	60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71,
// 	72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83,
// 	84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96
// };

// physical/musical constants
const double SOUND_SPEED = 343.2; // m/s
const double A4_FREQ = 440.0; // Hz

void setup()
{
	// configure Arduino ports
	pinMode(MIC_PIN, INPUT);
	pinMode(POT_PIN, INPUT);
	servo.attach(SERVO_PIN);
	pinMode(LED_UP, OUTPUT);
	pinMode(LED_GOOD, OUTPUT);
	pinMode(LED_DOWN, OUTPUT);
	
	Serial.begin(9600); // TODO remove for faster loop speed?
	
	fft = arduinoFFT();
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
		fft.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
		fft.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
		fft.ComplexToMagnitude(vReal, vImag, SAMPLES);
		
		// find dominant MIDI pitch (formula from https://newt.phys.unsw.edu.au/jw/notes.html)
		double peakFreq = fft.MajorPeak(vReal, SAMPLES, SAMPLING_FREQ);
		double peakPitch = freq2Pitch(peakFreq);
		
		// find pitch correction amount and corrected freqency
		double corr = correction(peakPitch, analogRead(POT_PIN));
		double correctedFreq = pitch2Freq(peakPitch + corr);
		
		updateServo(correctedFreq, peakFreq);
		
		updateLEDs(corr, 0.10); // tolerance units: MIDI pitch
	}
}

// find the (signed) amount of pitch correction to apply,
// given an input pitch and a strength from 0 to 1
double correction(double pitchIn, double strength)
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
	
	return strength * (PITCH_SET[idx] - pitchIn);
}

// update the LEDs based on amount of correction applied
void updateLEDs(double corr, double tol)
{
	// choose which LED to light
	if (corr > tol)
	{
		// light the LED for high
		digitalWrite(LED_UP, HIGH);
		digitalWrite(LED_GOOD, LOW);
		digitalWrite(LED_DOWN, LOW);
	}
	else if (corr < -tol)
	{
		// light the LED for down
		digitalWrite(LED_UP, LOW);
		digitalWrite(LED_GOOD, LOW);
		digitalWrite(LED_DOWN, HIGH);
	}
	else
	{
		// light the LED for good
		digitalWrite(LED_UP, LOW);
		digitalWrite(LED_GOOD, HIGH);
		digitalWrite(LED_DOWN, LOW);
	}
}

// move the servo to the angle that yields a perfect correction;
// otherwise rotate it to center.
void updateServo(double correctedFreq, double realFreq)
{
	// convert to servo angle and move servo
	double realLength = ((SOUND_SPEED / realFreq) * 1000.0) / 4.0; // mm
	double desiredLength = ((SOUND_SPEED / correctedFreq) * 1000.0) / 4.0; // mm
	double corrDist = desiredLength - realLength; // mm
	
	// servo angle is 0-180, so 90 is centered
	if (corrDist > SERVO_ARM_LG)
	{
		// can't make tube long enough; make it as long as possible
		servo.write(180);
	}
	else if (corrDist < -SERVO_ARM_LG)
	{
		// can't make tube short enough; make it as short as possible
		servo.write(0);
	}
	else
	{
		servo.write(90 + asin(corrDist / SERVO_ARM_LG));
	}
}

double freq2Pitch(double freq)
{
	return 12.0 * log(freq/A4_FREQ) / log(2) + 69;
}

double pitch2Freq(double pitch)
{
	return A4_FREQ * pow(2, ((pitch - 69) / 12.0));
}