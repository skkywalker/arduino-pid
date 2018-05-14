/*
  pid_c.h - Library for implementing PID controllers.
  Created by Lucas "skkywalker", May 12, 2018.
  Released into the public domain.
*/
#include "Arduino.h"
#include "Pid_c.h"

PID::PID() {
	kp = 1.0;
	kd = 1.0;
	ki = 1.0;
	sample_time = 10;
	debugMode = 1;
	last_error = 0.0;
	last_control = 0.0;
	output_control = 0.0;
	min_val = 0.0;
	max_val = 1023.0;
	_time = 0.0;
}

PID::PID(double kp_i, double kd_i, double ki_i, int sample_time_i, int debugMode_i) {
	kp = kp_i;
	kd = kd_i;
	ki = ki_i;
	sample_time = sample_time_i;
	debugMode = debugMode_i;
	last_error = 0.0;
	last_control = 0.0;
	output_control = 0.0;
	min_val = 0.0;
	max_val = 1023.0;
	_time = 0.0;
}

PID::PID(double kp_i, double kd_i, double ki_i, int sample_time_i) {
	kp = kp_i;
    kd = kd_i;
    ki = ki_i;
    sample_time = sample_time_i;
    debugMode = 0;
    last_error = 0.0;
	last_control = 0.0;
	output_control = 0.0;
	min_val = 0.0;
	max_val = 1023.0;
	_time = 0.0;
}

void PID::setSampleTime(int sample_time_i) {
	sample_time = sample_time_i;
}

void PID::controlRun() {
	int input = 1023*(1-(max_val-analogRead(input_pin))/(max_val-min_val));

	double error = ref - input;
	double proportional_control = kp * error;

	double area = (last_error + error)/2 * (sample_time/1000);
	double integral_control = ki * area;

	double angle = (- error + last_error)/(sample_time/1000);
	double derivative_control = kd * angle;

	last_control = output_control;

	output_control = last_control + proportional_control + derivative_control + integral_control;

	if(output_control < 0.0)
		output_control = 0.0;

	if (output_control > 1023.0)
		output_control = 1023.0;

	analogWrite(output_pin, output_control);

	if(debugMode == 1) {
		Serial.println("#######################################");
		Serial.print("Porportional Control: ");
		Serial.println(proportional_control);
		Serial.print("Derivative Control: ");
		Serial.println(derivative_control);
		Serial.print("Integral Control: ");
		Serial.println(integral_control);
		Serial.println("--------------------------------------");
		Serial.print("Last Error is ");
		Serial.println(last_error);
		Serial.print("Error (reference - input) is ");
		Serial.println(error);
		Serial.print("Sensor reading: ");
		Serial.println(input);
		Serial.print("Last Output value: ");
		Serial.println(last_control);
		Serial.print("Output value: ");
		Serial.println(output_control);
		Serial.print("Reference: ");
		Serial.println(ref);
		Serial.print("Sample Time: ");
		Serial.println(sample_time);
	} else if(debugMode == 2) { // Print reference and sensor info with csv formatting
		Serial.print(ref);
		Serial.print(",");
		Serial.print(input);
		Serial.print(",");
		Serial.println(_time);
	}

	_time += sample_time;
	last_error = error;

	delay(sample_time);
}

void PID::setConstants(double kp_i, double kd_i, double ki_i) {
	kp = kp_i;
    kd = kd_i;
    ki = ki_i;
}

void PID::setValues(double ref_i, int i_pin, int o_pin) {
	ref = ref_i;
	input_pin = i_pin;
	output_pin = o_pin;
}

void PID::setRange(double _min, double _max) {
	min_val = _min;
	max_val = _max;
}