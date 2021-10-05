
// This is a specific version of ROSSerial, please check ...
#define Use_ROS

// We user Servo arduino library to drive PWM output since it offers better resolution than simple analogWrite.
// However we did a little change compared to original version (from here : https://github.com/arduino-libraries/Servo),
// to reduce memory footprint, we changed SERVOS_PER_TIMER from 12 to 4 (src/Servo.h)
#define Use_Servo

// ROS, if used, define serial speed
#ifdef Use_ROS

#define ROSSERIAL_BAUD 1000000 

#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>

#endif

// define Alarm
#define LED_CTRL_ALARM_BATTERY_LOW  0 // Means that battery voltage is low (Red inversed 2 flash blinking each half second) 
#define LED_CTRL_ALARM_STARTUP      1 // Means that Arduino just started/restarted (Orange blinking fast)
#define LED_CTRL_ALARM_RX_LOSS      2 // Means PCM RX Signal not detected/loss (Blue blinking fast)
#define LED_CTRL_ALARM_ROS_LOSS     3 // Means ROS not connected/disconnected (Green blinking fast)
#define LED_CTRL_ALARM_DRIVE_LOSS   4 // Means no PWM order received, failsafe (White blinking fast)

// default PWN output if RC signal loss or ROS loss
#define PWN_out_throttle_Failsafe 1500
#define PWN_out_steering_Failsafe 1500 

// Battery Low VOltage
#define LIPO_CELL_LOW_VOLTAGE 3200
#define VBAT_LOW_VOLTAGE 5500
// Definition of pin assignment
//

// PWM Outputs, require timer 1
#define pwmOutThrottlePin     9
#define pwmOutSteeringPin     10

// PWM/sensors input, use Pin Change ISR PCINT2_vect
#define pwmInThrottlePin      6
#define pwnInSteeringPin      7
#define USEchoPin             2
#define RPMSignalPin          4

// on Port D
#define RGBLEDPin             5

// Analog INPUT (ADC)
#define batteryCell1Pin       A0
#define batteryCell2Pin       A2
#define batteryVBatPin        A1

// Simple GPIO Output
#define USTriggerPin          3

// Other sizing
#define RGBLEDCount  1

//GLobal state
unsigned char rx_loss=0;
unsigned char drive_loss=0;
unsigned char battery_low=0;
unsigned char failsafe = 0;

void setup (void) {
  pwmdriver_setup(pwmOutThrottlePin, pwmOutSteeringPin);
  setup_pwm_sampler();
  led_controler_setup();
  LIPO_watcher_setup();
  sensor_controler_setup();
  com_controler_setup();
  //Serial.begin(115200);
  led_controler_set_alarm(LED_CTRL_ALARM_STARTUP);
}

void loop() {
  static unsigned int _cnt=0;
  unsigned long tsStart;
  unsigned long tsEnd;
  int throttleIn=1500;
  int steeringIn=1500;

  _cnt++;
  tsStart=millis();

  if (_cnt==500) {
      led_controler_reset_alarm(LED_CTRL_ALARM_STARTUP);  
  }
  //Task to be done at 1Hz
  if (_cnt%100 == 0) {
    LIPO_watcher_update();  
  }
  //Task to be done at 10Hz
  if (_cnt%10 == 0) {
    led_controler_update();
    pwmdriver_check_failsafe();
    pwm_sampler_check_failsafe();
  }
  //Task to be done at 30Hz
  if (_cnt%3 == 0) {
    sensor_controler_update();
  }  

  //Task to be done at 100Hz
  if (_cnt%1 == 0) {
    pwm_sampler_update(&throttleIn, &steeringIn);
    publish_channels_state (throttleIn, steeringIn);
  }

  //Task to be done at 100Hz
  com_controler_update();
  
  tsEnd=millis();
  if ((tsEnd>=tsStart) && ((tsEnd-tsStart)<=10)) {
    delay(10-(tsEnd-tsStart)); // delais until next 100Hz tick
  }
}
