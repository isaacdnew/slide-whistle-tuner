/*

MAE 3780 Individual Project
Isaac Newcomb (idn6)

This code uses the arduinoFFT library to identify the pitch produced by a slide
whistle, find the nearest pitch in a specified scale, and adjust a servo to move the
slide whistle's slide - thereby correcting the pitch produced. Three labeled LEDs
indicate the pitch correction's direction, if any.

*/

#include "arduinoFFT.h"
#include <Servo.h>

// Arduino pins
const int MIC_PIN = A0; // analog input
const int POT_PIN = A1; // analog input
const int SERVO_PIN = 11; // PWM output
const int LED_SHORTEN = 3; // digital output
const int LED_GOOD = 4; // digital output
const int LED_LENGTHEN = 5; // digital output


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
	pinMode(LED_SHORTEN, OUTPUT);
	pinMode(LED_GOOD, OUTPUT);
	pinMode(LED_LENGTHEN, OUTPUT);
	
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
		// use hamming windowing to get narrower peaks but more frequency bleed
		// use hann    windowing to get wider    peaks but less frequency bleed
		fft.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
		fft.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
		fft.ComplexToMagnitude(vReal, vImag, SAMPLES);
		
		// find dominant MIDI pitch (formula from https://newt.phys.unsw.edu.au/jw/notes.html)
		double peakFreq = fft.MajorPeak(vReal, SAMPLES, SAMPLING_FREQ);
		double peakPitch = freq2Pitch(peakFreq);
		
		// find pitch correction amount and corrected freqency
		double corr = correction(peakPitch, analogRead(POT_PIN));
		double correctedFreq = pitch2Freq(peakPitch + corr);
		
		updateServo(peakFreq, correctedFreq);
		
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
		// indicate that the tube is being shortened (pitch raised)
		digitalWrite(LED_SHORTEN, HIGH);
		digitalWrite(LED_GOOD, LOW);
		digitalWrite(LED_LENGTHEN, LOW);
	}
	else if (corr < -tol)
	{
		// indicate that the tube is being lengthened (pitch lowered)
		digitalWrite(LED_SHORTEN, LOW);
		digitalWrite(LED_GOOD, LOW);
		digitalWrite(LED_LENGTHEN, HIGH);
	}
	else
	{
		// indicate that the tube is being minimally corrected (pitch is good)
		digitalWrite(LED_SHORTEN, LOW);
		digitalWrite(LED_GOOD, HIGH);
		digitalWrite(LED_LENGTHEN, LOW);
	}
}

// move the servo to the angle that yields a perfect correction;
// otherwise rotate it to center.
void updateServo(double realFreq, double correctedFreq)
{
	// convert to servo angle and move servo
	double realLength = ((SOUND_SPEED / realFreq) * 1000.0) / 4.0; // mm
	double desiredLength = ((SOUND_SPEED / correctedFreq) * 1000.0) / 4.0; // mm
	double corrDist = desiredLength - realLength; // mm
	
	// servo angle is 0-180, so 90 is centered
	// compact syntax: (boolean expression) ? (true block) : (false block)
	// write safe angle to servo, doing the trig calculation only if it'll yield a valid answer:
	servo.write(corrDist < -SERVO_ARM_LG ? 0 : corrDist > SERVO_ARM_LG ? 180 : 90 + asin(corrDist / SERVO_ARM_LG));
}


double freq2Pitch(double freq)
{
	return 12.0 * log(freq/A4_FREQ) / log(2) + 69;
}

double pitch2Freq(double pitch)
{
	return A4_FREQ * pow(2, ((pitch - 69) / 12.0));
}