# arduino_l298n_driver_simple
This is a simple ROS package for an arduino using the L298N board to drive DC motors.

Load it onto a sutiable arduino UNO.

I am using PWM pins 9 and 10 to turn on the motors
I am sending polarity through 4 5 6 7

NOTE: I cannot test this code right now. I do not have enough voltage to run any motor. If this works on your system, please tell me.

Through Arduino IDE, upload

then run 

$ rosrun rosserial_python serial_node.py /dev/tty*

in /dev/tty*, find the correct port your arduino is connecting to. It is usually /dev/ttyACM0 or /dev/ttyUSB0
