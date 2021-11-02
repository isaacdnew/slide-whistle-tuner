/*

SnapSlide: a pitch-corrected slide whistle
MAE 3780 Individual Project — Isaac Newcomb (idn6@cornell.edu)

This code uses elm-chan's ffft library to identify the pitch produced by a slide
whistle, finds the distance to nearest pitch in a specified scale, and adjusts a
servo to move the slide whistle's slide - thereby correcting the pitch produced.
Three labeled LEDs indicate the pitch correction's direction, if any.

Some of this code (along with the hardware requirements) is adapted from the Piccolo music visualizer by Adafruit:
https://github.com/adafruit/Adafruit_Learning_System_Guides/tree/main/Tiny_Music_Visualizer

*/

#include "ffft.h"
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

// microphone
const int MIN_LOUDNESS = 20; // minimum sample loudness to consider as a signal (0 - 511)

// FFT
const int SAMPLING_FREQ = 4096; // [Hz] - must be twice the highest expected frequency
const int SAMPLING_PERIOD = round(1000000*(1.0/SAMPLING_FREQ)); // sampling period in microseconds
int16_t   capture[FFT_N];    // Audio capture buffer
complex_t bfly_buff[FFT_N];  // FFT "butterfly" buffer
uint16_t  spectrum[FFT_N/2]; // Spectrum output buffer
double peakFreq = 0; // the frequency of the highest peak
// FFT_N is the number of samples per snippet. It's #defined in ffft.h as 128

// servo
Servo servo;
const int SERVO_MIN = 470; // minimum pulse length (microseconds)
const int SERVO_MAX = 2620; // maximum pulse length (microseconds)
const double SERVO_ARM_LG = 15.0; // [mm]
const double LG_TOL = 5; // how close the slide has to be to its ideal position before the green light turns on [mm]
double servoX = 0; // position of scotch yoke relative to arduino [mm]

// PID control loop
double P = 0; // proportional term in PID control - adjusted by potentiometer
double I = 0; // integral term in PID control (not yet implemented - TODO)
double D = 4; // derivative term in PID control

double mmErr = 0; // signed distance to nearest good length [mm]
double integ = 0; // sum of error over time [mm] (not yet implemented - TODO)
double deriv = 0; // rate at which mmErr is changing [mm/s]

double mmErrOld = 0; // the previous mmErr (used to calculate deriv)

/*
	lists of target pitches - select by (un)commenting
*/

// // c pentatonic
// const int PITCH_CT = 21;
// const int PITCHES[PITCH_CT] =
// {
// 	48, 50, 52, 55, 57,
// 	60, 62, 64, 67, 69,
// 	72, 74, 76, 79, 81,
// 	84, 86, 88, 91, 93, 96
// };

// c major
const int PITCH_CT = 29;
const int PITCHES[PITCH_CT] =
{
	48, 50, 52, 53, 55, 57, 59,
	60, 62, 64, 65, 67, 69, 71,
	72, 74, 76, 77, 79, 81, 83,
	84, 86, 88, 89, 91, 93, 95, 96
};

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

// // just one note: A4
// const int PITCH_CT = 1;
// const int PITCHES[PITCH_CT] =
// {
// 	69
// };

void setup()
{
	// configure Arduino ports
	servo.attach(SERVO_PIN, SERVO_MIN, SERVO_MAX);
	pinMode(MIC_PIN, INPUT);
	pinMode(POT_PIN, INPUT);
	pinMode(LED_SHORTEN, OUTPUT);
	pinMode(LED_GOOD, OUTPUT);
	pinMode(LED_LENGTHEN, OUTPUT);
	
	Serial.begin(9600); // open serial connection for debugging
}

void loop()
{
	// get a snippet of samples
	int16_t loudness = 0; // reset loudness reading
	for (int i = 0; i < FFT_N; i++)
	{
		long sampleBirthday = micros(); // save when the sample was taken to the microsecond
		capture[i] = analogRead(MIC_PIN) - 512; // take a sample, shifting to range [-512, 511]
		
		if (capture[i] > loudness)
		{
			loudness = capture[i];
		}
		
		while(micros() < (sampleBirthday + SAMPLING_PERIOD))
		{
			// do nothing - precise timing is essential
		}
	}
	
	// Serial.print("loudness = "); Serial.println(loudness);
	
	// if it's loud enough that there's probably a note being played...
	if (loudness >= MIN_LOUDNESS)
	{
		// do FFT on the snippet of samples
		fft_input(capture, bfly_buff);   // Samples -> complex numbers in bfly_buff
		fft_execute(bfly_buff);          // Process complex data
		fft_output(bfly_buff, spectrum); // Complex -> frequency spectrum
		
		// analyze the FFT output and take action
		peakFreq = majorPeak(spectrum, FFT_N, SAMPLING_FREQ); // find loudest frequency in the spectrum
		P = analogRead(POT_PIN) / 1024.0; // use potentiometer value as P gain
		mmErrOld = mmErr; // save previous value to calculate derivative with it
		mmErr = mmError(peakFreq); // find signed distance to nearest good length [mm]
		deriv = (mmErr - mmErrOld) / SAMPLING_PERIOD; // approximate time derivative of error [mm/s]
		slideTo(servoX - P * mmErr + D * deriv); // move inversely to error but proportionally to the derivative of error
		updateLEDs(mmErr, LG_TOL); // indicate servo position with LEDs
		
		// Serial.print("freq   = "); Serial.println(peakFreq);
		// Serial.print("mmErr  = "); Serial.println(mmErr);
	}
}


/*
	finds the frequency of the highest peak of the FFT output.
	copied from the arduinoFFT library: https://github.com/kosme/arduinoFFT
*/
double majorPeak(int16_t *vD, int16_t samples, double samplingFrequency)
{
	double maxY = 0;
	uint16_t IndexOfMaxY = 0;
	// If sampling_frequency = 2 * max_frequency in signal,
	// value would be stored at position samples/2
	for (uint16_t i = 1; i < ((samples >> 1) + 1); i++)
	{
		if ((vD[i-1] < vD[i]) && (vD[i] > vD[i+1]))
		{
			if (vD[i] > maxY)
			{
				maxY = vD[i];
				IndexOfMaxY = i;
			}
		}
	}
	double delta = 0.5 * ((vD[IndexOfMaxY-1] - vD[IndexOfMaxY+1]) / (vD[IndexOfMaxY-1] - (2.0 * vD[IndexOfMaxY]) + vD[IndexOfMaxY+1]));
	double interpolatedX = ((IndexOfMaxY + delta)  * samplingFrequency) / (samples-1);
	if(IndexOfMaxY==(samples >> 1)) // To improve calculation on edge values
		interpolatedX = ((IndexOfMaxY + delta)  * samplingFrequency) / (samples);
	// returned value: interpolated frequency peak apex
	return(interpolatedX);
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
	conversion formulas / simple things
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

// find max element in array
int16_t findMax(int16_t* array, int array_lg)
{
	int16_t max = array[0];
	
	for (int i = 1; i < array_lg; i++)
	{
		if (array[i] > max)
		{
			max = array[i];
		}
	}
	
	return max;
}