Intro
=====

This repositary contains the software part of the DIYRobocarsFr Hat.
This Hat is designed for Raspberry Pi 3/4, Jetson Nano or any other single board computer Host implementing the well known 'rpi' GPIO connector.
This Hat provides various interfaces between the Host and hardware to easely build an autonomous small scale RC-style car.
A typical example of such build is the [Donkey car](https://www.donkeycar.com/)

The Hat features :
- LM2596 or xl4005 DSN5000 based DC/DC converter to power everything from a single battery (7-15V, NiMh or LiPo 2S/3S), through GPIO or through dedicated connector.
- Battery voltage monitoring (battery level and cell level for LiPo)
- PWM output signal to control Throttle (through ESC) and STeering (through Servo)
- PWM input signal acquisition of RC Receiver (4 channels)
- Ultra Sonic acquisition
- RPM sensor acquisition
- 2 pre-wired Aux channels
- High Speed data link (Serial 1Mbps) between Hat and the Host 
- RBG Led to display status, including Alarm on low battery voltage, loss of receiver, loss of link with the Host, loss of active driving 
- RGB Led output to control more WS2812B LEDs 
- Faisafe (Force output PWM to neutral signal in case of alarm)

Communication with Host
=======================

Protocol
--------

Communication between Hat and the Host is based on either ROS/ROSSerial, or on a simple stupid serial protocol.
To select the protocol to use, uncomment accordingly one of the following macro before compiling the sketch (on top of RobocarsHat.ino):
#define Use_ROSSerial // Select ROS communication implementation
#define Use_SimpleSerial // select simple stupid serial protocol

Messages
--------

Hat publishes the following topics to the Host:
- radio_channels @ 100Hz : Throttle and Steering value acquired from RX Receiver
- battery @ 1Hz : Battery and Cells current voltage
- sensors @ 30Hz : Ultrasonic and RPM sensor

Hat consumes the folowing topics from the Host:
- Driving Throttle and Steering

ROS protocol
============

When ROS is used, topics are :
- /radio_channels : std_msgs::Int16MultiArray
    - data[0] : Int16 : Throttle (signal pulse width in us, expecting value betzeen 1000 and 2000)
    - data[1] : Int16 : Steering (signal pulse width in us, expecting value betzeen 1000 and 2000) 
    - data[2] : Int16 : Aux1 (signal pulse width in us, expecting value betzeen 1000 and 2000)
    - data[3] : Int16 : Aux2 (signal pulse width in us, expecting value betzeen 1000 and 2000) 
- /battery : std_msgs::Int16MultiArray
    - data[0] : Int16 : Vbat : Battery total voltage in mv
    - data[1] : Int16 : Cell1 : LiPo Cell 1 voltage in mv (0 is no Cell detected)
    - data[2] : Int16 : Cell2 : LiPo Cell 1 voltage in mv (0 is no Cell detected)
    - data[3] : Int16 : Cell3 : LiPo Cell 1 voltage in mv (0 is no Cell detected)
- /sensors : std_msgs::Int16MultiArray
    - data[0] : Int16 : Ultrasonic sensor : Distance in cm, -1 if not plugged
    - data[1] : Int16 : rpm sensor : from ~100 (high rpm) to 2000 (low rpm), to check depending on sensor used
- /throttle_ctrl/output : std_msgs::Int16
    - data : Int16 : Throttle (signal pulse width in us, expecting value betzeen 1000 and 2000)
- /steering_ctrl/output : std_msgs::Int16
    - data : Int16 : Steering (signal pulse width in us, expecting value betzeen 1000 and 2000)

Note : Code has been optimized to fit into the Arduino pro mini.
When ROS is selected to implement comunication with Host, almost all RAM is consumed by this sketch. The remaining free memory (~180 Bytes) must be preserved since dynamic allocation can occurrd at runtime. So far this is enough to work like a charm.

Simple Stupid Serial protocol
=============================
This protocol exchanges data through serial link as CSV.
Separator between messages is the carriage return/line feed.
Message are formated following the scheme :

``<msg_type>,<param1>,<param2>,<param3,...\r\n>`` 

msg_type, from Hat to Host :

- 0 : radio channels
    - param1 : int : throttle (signal pulse width in us, expecting value betzeen 1000 and 2000)
    - param2 : int : steering (signal pulse width in us, expecting value betzeen 1000 and 2000) 
    - param3 : int : Aux1 (signal pulse width in us, expecting value betzeen 1000 and 2000)
    - param4 : int : Aux2 (signal pulse width in us, expecting value betzeen 1000 and 2000) 
- 1 : battery state
    - param1 : Int : Vbat : Battery total voltage in mv
    - param2 : Int : Cell1 : LiPo Cell 1 voltage in mv (0 is no Cell detected)
    - param3 : Int : Cell2 : LiPo Cell 1 voltage in mv (0 is no Cell detected)
    - param4 : Int : Cell3 : LiPo Cell 1 voltage in mv (0 is no Cell detected)
- 2 : sensors 
    - param1 : Int : Ultrasonic sensor : Distance in cm, -1 if not plugged
    - param2 : Int : rpm sensor : from ~100 (high rpm) to 2000 (low rpm), to check depending on sensor used

msg_type, from Host to Hat :

- 0 : drive order
    - param1 : int : throttle (signal pulse width in us, expecting value betzeen 1000 and 2000)
    - param2 : int : steering (signal pulse width in us, expecting value betzeen 1000 and 2000)


Dependencies
============

Needed dependencies :
- ros [see rosserial for Arduino](http://wiki.ros.org/rosserial_arduino/Tutorials). The rosserial packaged used is a specific one, available [here](https://github.com/btrinite/rosserial). It features a macro allowing to change serial baud rate (see ROSSERIAL_BAUD)
- PololuLedStrip library to control RGB WS2812B style LED, available [here](https://github.com/pololu/pololu-led-strip-arduino).
- modified Arduino Servo library, available [here](https://github.com/btrinite/Servo). The change is about to reduce memory footprint. For that purpose, we have decreased SERVOS_PER_TIMER from 12 to 4 (src/Servo.h)

The PWM sampling part of the software is largely inspired from Kelvin Nelson work, available [here](https://create.arduino.cc/projecthub/kelvineyeone/read-pwm-decode-rc-receiver-input-and-apply-fail-safe-6b90eb)


History
=======

Nov 19 2021 - conflict with u-boot startup
------------------------------------------

Working on a raspberry 4 with Ubuntu 20.04 64 bits, we found that u-boot provided with Ubuntu allows user to interact with boot at startup time through uart (the one connected through GPIO) for few seconds before default boot occured. As soon as power is supplied, both Raspberry and Arduino starts. Arduino then start sending messages through the serial link with Raspberry (whatever the comunication option selected). However, u-boot starts also listening to UART to let chance for uer to halt boot sequence. This is the conflictual and prevent raspberry to start because of messages received on UART on raspberry side and u-boot deciding to halt default boot seqeuence. For that purpose, until a better solution is identified (such as disabling interaction through UART in u-boot), a delay can be added in Arduino firmware to let a change for Raspberry to boot Linux. Delay is fixed to 5 seconds zhich looks enough. This behavior is controlled though STARTUP_DELAYED macro.