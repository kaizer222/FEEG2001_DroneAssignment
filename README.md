# FEEG2001_DroneAssignment
GROUP 11 FEEG2001 SEMESTER 2 MULTIROTOR DESIGN PROJECT 

Notes:
1) FIFO Overload
2) Continuous Servo not usable

Solution:
1) Reduce noise in SDA and SCL lines by:
	ensuring <3cm wire length
	using 100 Ohm resistor in series

2) Replace all continuous servo with conventional servos

To do:

1) LED indication lights
2) TX toggle for gimbal
3) Complementary filter (optional):
	96% Gyro
	4% Accel 