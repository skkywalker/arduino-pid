# arduino-pid

#### Starting the controller

You can define the controller using:

```PID my_controller();``` with Kp, Kd, Ki = 1.0; sampling time = 10 ms; debugMode = 1;

```PID my_controller(kp, kd, ki, sample_time, debugMode);``` where sample_time is given in ms and debugMode is 0 (no debugging), 1 (shows every variable) and 2 (shows reference, sensor value, time to export to .csv). Or using:

`PID my_controller(kp, kd, ki, sample_time); ` to start the PID controller with debugMode set to 0.

#### Functions

```my_controller.setSampleTime(int)```  sets the sampling time in ms.

```my_controller.setConstants(double, double, double)``` sets Kp, Kd and Ki.

```my_controller.setValues(double, int, int)``` to set reference, input (sensor) pin and output (PWM) pin.

```my_controller.controlRun()``` to read the input, compare it to the reference and make  corrections to the PWM output pin. Will print variable values if ```debugMode == 1``` or will print reference, sensor value and time if ```debugMode == 2```.

```my_controller.setRange()``` to place sensor input levels in a 0-1023 scale.