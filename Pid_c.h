/*
  pid_c.h - Library for implementing PID controllers.
  Created by Lucas "skkywalker", May 12, 2018.
  Released into the public domain.
*/
#ifndef Pid_c_h
#define Pid_c_h

#include "Arduino.h"

class PID
{
  public:
    PID(); // Constants = 1.0, debugMode = 1, sample_time = 10ms
    PID(double, double, double, int, int); // Proportional, derivative and integral constants, sampling time, debug mode
    PID(double, double, double, int); // No debug
    void setSampleTime(int); // Set sample_time
    void controlRun(); // Runs the PID controller
    void setConstants(double, double, double); // Kp, Kd, Ki
    void setValues(double, int, int); // Reference, sensor input, output PWN
    void setRange(double, double); // Min value, max value to place sensor value
  
  private:
    double kp; // Kp
    double ki; // Ki
    double kd; // Kd
    double sample_time; // sets the sampling time
    int debugMode; // sends important values to serial
    double ref; // reference value
    double last_error; // keeps track of last error to calculate area and slope
    int input_pin; // reference pin for the input sensor read
    int output_pin; // reference pin for the ouput PWN
    double last_control; // last value of output_control
    double output_control; // output value to PWM output
    double max_val; // maximum value to place sensor input
    double min_val; // minimum value to place sensor input
    double _time; // keeps track of time to export to .csv
};

#endif
