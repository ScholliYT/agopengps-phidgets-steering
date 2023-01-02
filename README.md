# AgOpenGPS Phidgets Steering
This projects tries to implement an AgOpenGPS compatible steering controller using a 
[Phidgets 24V DC Motor](https://www.phidgets.com/?tier=3&catid=19&pcid=16&prodid=993), 
a [Phidgets Motor Controller](https://www.phidgets.com/?tier=3&catid=18&pcid=15&prodid=1089)
and a [Phidgets Rotary Encoder](https://www.phidgets.com/?tier=3&catid=103&pcid=83&prodid=404).
The goal of this project is to eliminate the need of a wheel angle sensor by 
calculating the current wheel angle from the rotary encoder's output. 

The overall architecture consits of a python server (running on the same computer as AgOpenGPS) 
communicating via UDP traffic with AgIO on the one side 
and via an USB connection using the [Python Phidgets SDK](https://www.phidgets.com/docs/Language_-_Python#Quick_Downloads) with the motor controller. 

# Warning
Please note that this project is just a proof of concept and may fail / crash suddenly. 

# Similar / inspring Projects
- https://github.com/salmiac/pi-steer/
- https://github.com/jkonno/AOG_wasless
- https://github.com/v12cat/WAS-Less-AOG
- https://discourse.agopengps.com/t/aog-without-wheel-angle-sensor/2929

# Hardware
![Photo of the required hardware](https://i.ibb.co/d52sqcj/PXL-20230102-032135098-2.jpg)

- Windows Tablet
  - AGOpenGPS (v5.6.2)
  - Python 3.10
- [Phidgets 24V DC Motor](https://www.phidgets.com/?tier=3&catid=19&pcid=16&prodid=993)
- [Phidgets Motor Controller](https://www.phidgets.com/?tier=3&catid=18&pcid=15&prodid=1089)
- [Phidgets Rotary Encoder](https://www.phidgets.com/?tier=3&catid=103&pcid=83&prodid=404)
- 3D Printed gears for the motor and steering wheel
  - I used [these](https://drive.google.com/drive/folders/1oB-xxBiD6sCXIlz-HanKOceyacVLFpN9) but likely any will do
- GPS (+IMU)
  - You will also need a system to get the position. There are many options with AGOpenGPS to choose from (e.g. Single, Single+IMU, Dual ...)
  - I used the [AGOpenGPS Panda Board](https://github.com/farmerbriantee/AgOpenGPS/tree/master/Support/TeensyModules/Board/Panda_Board) with single GPS and BNO085 IMU connected via USB
