This repositary contains the software part of the DIYRobocarsFr Hat.
This Hat is designed for Raspberry Pi 3/4, Jetson Nano or any other SBC implementing the well known 'rpi' GPIO connector.
This Hat provides various interfaces between SBC and hardware to easely build an autonomous small scale RC-style car.
A typical example of such build is the [Donkey car](https://www.donkeycar.com/)

The Hat features :
- LM2596 or xl4005 DSN5000 based DC/DC converter to power everything from a single battery (7-15V, NiMh or LiPo 2S/3S), through GPIO or through dedicated connector.
- Battery voltage monitoring (battery level and cell level for LiPo)
- PWM output signal to control Throttle (through ESC) and STeering (through Servo)
- PWM input signal acquisition of RC Receiver
- Ultra Sonic acquisition
- RPM sensor acquisition
- 2 pre-wired Aux channels
- High Speed data link (Serial 1Mbps) between Hat and SBC 
- RBG Led to display status, including Alarm on low battery voltage, loss of receiver, loss of link to SBC, loss of active driving 
- Faisafe (Force output PWM to neutral signal in case of alarm)

Communication between Hat and SBC is based on ROS and ROSSerial.
Hat publishes the following topics :
- radio_channels @ 100Hz : Throttle and Steering value acquired from RX Receiver
- battery @ 1Hz : Battery and Cells current voltage
- sensors @ 30Hz : Ultrasonic and RPM sensor

Alternative communication protocol could be implemented.

Needed dependencies :
- ros [see rosserial for Arduino](http://wiki.ros.org/rosserial_arduino/Tutorials). The rosserial packaged used is a specific one, available [here](https://github.com/btrinite/rosserial). It features a macro allowing to change serial baud rate (see ROSSERIAL_BAUD)
- PololuLedStrip library to control RGB WS2812B style LED, available [here](https://github.com/pololu/pololu-led-strip-arduino).
- modified Arduino Servo library, available [here](https://github.com/btrinite/Servo). The change is about to reduce memory footprint. For that purpose, we have decreased SERVOS_PER_TIMER from 12 to 4 (src/Servo.h)

The PWM sampling part of the software is largely inspired from Kelvin Nelson work, available [here](https://create.arduino.cc/projecthub/kelvineyeone/read-pwm-decode-rc-receiver-input-and-apply-fail-safe-6b90eb)

Code has been optimized to fit into the Arduino pro mini.
Very few free memory is currently available (197 Byte), but this is enough to work like a charm.