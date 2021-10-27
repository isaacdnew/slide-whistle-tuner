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

// physical/musical constants
const double SOUND_SPEED = 343.2; // [m/s]
const double A4_FREQ = 440.0; // frequency of A4 [Hz]

// arduino pins
const int MIC_PIN = A0; // analog in
const int POT_PIN = A1; // analog in
const int SERVO_PIN = 6; // PWM out
const int LED_SHORTEN = 11; // digital out
const int LED_GOOD = 12; // digital out
const int LED_LENGTHEN = 13; // digital out

// FFT
arduinoFFT fft;
const int SAMPLE_CT = 128; // must be a base 2 number; max 128 for Arduino Uno
const int SAMPLING_FREQ = 4096; // must be twice the highest expected frequency
const int SAMPLING_PERIOD = round(1000000*(1.0/SAMPLING_FREQ)); // sampling period in microseconds

double vReal[SAMPLE_CT]; // create vector of size SAMPLE_CT to hold real values
double vImag[SAMPLE_CT]; // empty; should initialize as 0s and remain as such
bool snippetIsReady = false; // indicates when to recalculate the FFT
long sampleBirthday; // the micros() time when the last sample was recorded
int i = 0; // index of sample to record

// servo and feedback control loop
Servo servo;
const int SERVO_MIN = 470; // minimum pulse length (microseconds)
const int SERVO_MAX = 2620; // maximum pulse length (microseconds)
const double SERVO_ARM_LG = 15.0; // [mm]

double servoX = 0; // position of scotch yoke relative to arduino [mm]
double mmErr = 0; // given current pitch, signed distance to nearest good length [mm]
double p = 0; // proportional term for feedback control (controlled by potentiometer)

/*
	lists of target pitches - select by (un)commenting
*/

// // c major
// const int PITCH_CT = 29;
// const int PITCHES[PITCH_CT] =
// {
// 	48, 50, 52, 53, 55, 57, 59,
// 	60, 62, 64, 65, 67, 69, 71,
// 	72, 74, 76, 77, 79, 81, 83,
// 	84, 86, 88, 89, 91, 93, 95, 96
// };

// // c minor
// const int PITCH_CT = 29;
// const int PITCHES[PITCH_CT] =
// {
// 	48, 50, 51, 53, 55, 56, 58,
// 	60, 62, 63, 65, 67, 68, 70,
// 	72, 74, 75, 77, 79, 80, 82,
// 	84, 86, 87, 89, 91, 92, 94, 96
// };

// // chromatic
// const int PITCH_CT = 49;
// const int PITCHES[PITCH_CT] =
// {
// 	48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59,
// 	60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71,
// 	72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83,
// 	84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96
// };

// just one note: A4
const int PITCH_CT = 1;
const int PITCHES[PITCH_CT] =
{
	69
};

void setup()
{
	// configure Arduino ports
	servo.attach(SERVO_PIN, SERVO_MIN, SERVO_MAX);
	pinMode(MIC_PIN, INPUT);
	pinMode(POT_PIN, INPUT);
	pinMode(LED_SHORTEN, OUTPUT);
	pinMode(LED_GOOD, OUTPUT);
	pinMode(LED_LENGTHEN, OUTPUT);
	
	Serial.begin(9600); // TODO remove for faster loop speed?
	
	fft = arduinoFFT();
	sampleBirthday = micros(); // pretend we just took a sample (to start the loop correctly)
	
	for (int i = 0; i < SAMPLE_CT; i++)
	{
		vImag[i] = 0; // imaginary part is always 0
	}
	
}

void loop()
{
	// get a snippet of samples
	for (int i = 0; i < SAMPLE_CT; i++)
	{
		sampleBirthday = micros(); // save sample birthday to the microsecond
		vReal[i] = analogRead(MIC_PIN); // take a sample
		
		while(micros() < (sampleBirthday + SAMPLING_PERIOD))
		{
			// do nothing - precise timing is essential
		}
	}
	
	// do FFT on the snippet of samples
	// use hamming windowing to get narrower peaks but more frequency bleed
	// use hann    windowing to get wider    peaks but less frequency bleed
	fft.Windowing(vReal, SAMPLE_CT, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
	fft.Compute(vReal, vImag, SAMPLE_CT, FFT_FORWARD);
	fft.ComplexToMagnitude(vReal, vImag, SAMPLE_CT);
	
	// find dominant MIDI pitch (formula from https://newt.phys.unsw.edu.au/jw/notes.html)
	double peakFreq = fft.MajorPeak(vReal, SAMPLE_CT, SAMPLING_FREQ);
	
	// TODO ignore bad peakFreq readings
	
	p = analogRead(POT_PIN) / 1024.0; // use potentiometer value as P gain
	mmErr = mmError(peakFreq); // find error for feedback control
	slideTo(servoX - p * mmErr); // move proportionally to error
	updateLEDs(servoX, 0.10); // indicate servo position with LEDs
	
	// Serial.print("mmErr = "); Serial.println(mmErr);
}

/*
	finds the signed distance (in millimeters) to the nearest valid note
*/
double mmError(double freq)
{
	double pitch = freq2Pitch(freq);
	double desiredFreq = pitch2Freq(pitch - pitchError(pitch)); // the note minus its imperfection
	return freq2mm(desiredFreq) - freq2mm(freq); // the difference between them
}

/*
	finds the signed pitch distance to the nearest valid note,
	comparing `pitchIn` to the set of desirable pitches
*/
double pitchError(double pitchIn)
{
	// if pitch is out of range, skip the algorithm
	if (pitchIn < PITCHES[0])
	{
		return pitchIn - PITCHES[0];
	}
	else if (PITCHES[PITCH_CT - 1] < pitchIn)
	{
		return pitchIn - PITCHES[PITCH_CT - 1];
	}
	
	// start in the middle of the list of valid pitches
	int minIdx = 0;
	int maxIdx = PITCH_CT - 1;
	int idx = maxIdx / 2; // in C++, int division truncates towards 0
	
	// binary search (ish): iteratively shrink the bounds by almost half each time
	while (maxIdx - minIdx > 1) // repeat until the gap between minIdx and maxIdx is 1 or less
	{
		if (PITCHES[idx] > pitchIn) // the pitch at idx is too high
		{
			// update range maximum
			maxIdx = idx;
			// but don't exclude this idx; it might be *just* above pitchIn
		}
		else if (PITCHES[idx] < pitchIn) // the pitch at idx is too low
		{
			// update range minimum
			minIdx = idx;
			// but don't exclude this idx; it might be *just* below pitchIn
		}
		else // pitchIn exactly matches a note in PITCHES
		{
			return 0;
		}
		
		idx = (maxIdx + minIdx) / 2; // int division: go halfway between min and max
	}
	
	// now PITCHES[minIdx] and PITCHES[maxIdx] are the two nearest to pitchIn.
	// find the errors for both and return the smaller-magnitude one:
	double err1 = pitchIn - PITCHES[minIdx];
	double err2 = pitchIn - PITCHES[maxIdx];
	return abs(err1) < abs(err2) ? err1 : err2;
}

/*
	moves the servo such that the scotch yoke is `x` mm from center
*/
void slideTo(double x)
{
	if (x < -SERVO_ARM_LG || SERVO_ARM_LG < x) // if x is too big or too small
	{
		// move home to hopefully find a different note to snap to
		servoX = 0;
		servo.write(90);
	}
	else
	{
		// move to the specified x
		servoX = x;
		servo.write(rad2deg(acos(servoX / SERVO_ARM_LG)));
	}
}

/*
	updates the LEDs based on amount of correction applied
*/
void updateLEDs(double pitchCorr, double tol)
{
	// choose which LED to light
	if (pitchCorr > tol)
	{
		// indicate that the tube is being shortened (pitch raised)
		digitalWrite(LED_SHORTEN, HIGH);
		digitalWrite(LED_GOOD, LOW);
		digitalWrite(LED_LENGTHEN, LOW);
	}
	else if (pitchCorr < -tol)
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

/*
	conversion formulas
*/

// from http://hyperphysics.phy-astr.gsu.edu/hbase/Waves/clocol.html#c1
double freq2mm(double freq)
{ //         ↓ m/s         ↓ Hz    ↓ mm/m    ↓ number of wavelengths that fit in a cylinder with one of its ends closed
	return ((SOUND_SPEED / freq) * 1000.0) * 0.25;
}

// from https://newt.phys.unsw.edu.au/jw/notes.html
double freq2Pitch(double freq)
{ //       ↓ 12 notes per octave                 ↓ MIDI pitch number: 69 corresponds to A4
	return 12.0 * log(freq/A4_FREQ) / log(2.0) + 69.0;
}

// from https://newt.phys.unsw.edu.au/jw/notes.html
double pitch2Freq(double pitch)
{ //                     ↓ a pitch jump of one octave doubles the frequency
	return A4_FREQ * pow(2.0, ((pitch - 69.0) / 12.0));
}

// convert radians to degrees
double rad2deg(double x)
{
	return x * 57.2957795131; // pre-evaluated 180/pi
}