# AgOpenGPS Phidgets Steering
This projects tries to implement an AgOpenGPS compatible steering controller using a 
[Phidgets 24V DC Motor](https://www.phidgets.com/?tier=3&catid=19&pcid=16&prodid=993), 
a [Phidgets Motor Controller](https://www.phidgets.com/?tier=3&catid=18&pcid=15&prodid=1089)
and a [Phidgets Rotary Encoder](https://www.phidgets.com/?tier=3&catid=103&pcid=83&prodid=404).
The goal of this project is to eliminate the need of a wheel angle sensor by 
calculating the current wheel angle from the rotary encoder's output. 

The overall architecture consits of a python server (running on the same computer as AgOpenGPS) 
communicating via UDP traffic with AgIO on the one side 
and via an USB connection using the Phidgets SDK with the motor controller. 


# Similar / inspring Projects
- https://github.com/salmiac/pi-steer/
- https://github.com/jkonno/AOG_wasless
