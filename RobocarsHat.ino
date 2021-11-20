
// Select communication protocole to use (Use_ROSSerial for ROS topics overs serial, Use_SLIPSerial for SLIP over serial)
//#define Use_ROSSerial
#define Use_SimpleSerial

// Delay startup to avoid conflict with host bootloader (u-boot for example)
//#define STARTUP_DELAYED

// We user Servo arduino library to drive PWM output since it offers better resolution than simple analogWrite.
// However we did a little change compared to original version (from here : https://github.com/arduino-libraries/Servo),
// to reduce memory footprint, we changed SERVOS_PER_TIMER from 12 to 4 (src/Servo.h)
#define Use_Servo

// ROS, if used, define serial speed
#ifdef Use_ROSSerial

#define ROSSERIAL_BAUD 1000000 

#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>

#endif

// define Alarm
#define LED_CTRL_ALARM_BATTERY_LOW  0 // Means that battery voltage is low (Red inversed 2 flash blinking each half second) 
#define LED_CTRL_ALARM_STARTUP      1 // Means that Arduino just started/restarted (Orange blinking fast)
#define LED_CTRL_ALARM_RX_LOSS      2 // Means PCM RX Signal not detected/loss (Blue blinking fast)
#define LED_CTRL_ALARM_LINK_LOSS    3 // Means ROS not connected/disconnected (Green blinking fast)
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
#define pwmInSteeringPin      7
#define pwmInAux1Pin          12
#define pwmInAux2Pin          8
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
  #ifdef STARTUP_DELAYED
  delay(5000);
  #endif
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
  static unsigned long lastTsStart = 0;
  unsigned long tsStart = 0;
  int throttleIn=1500;
  int steeringIn=1500;
  int aux1=1500;
  int aux2=1500;

  tsStart=micros();

  //Task to be done as soon as possible
  com_controler_update();

  if (tsStart>=lastTsStart) 
  {  
    if ((tsStart-lastTsStart)>=10000)
    {
      _cnt++;
      if (_cnt==500) {
        led_controler_reset_alarm(LED_CTRL_ALARM_STARTUP);  
      }
      //Task to be done at 1Hz
      if (_cnt%100 == 67) {
        battery_watcher_update();  
      }
      //Task to be done at 10Hz
      if (_cnt%10 == 5) {
        led_controler_update();
        pwmdriver_check_failsafe();
        pwm_sampler_check_failsafe();
      }
      //Task to be done at 30Hz
      if (_cnt%3 == 2) {
        sensor_controler_update();
      }  
    
      //Task to be done at 100Hz
      if (_cnt%1 == 0) {
        pwm_sampler_update(&throttleIn, &steeringIn, &aux1, &aux2);
        publish_channels_state (throttleIn, steeringIn, aux1, aux2);
      }
      lastTsStart=tsStart;     
    }
  } else {
      // Time counter overflow
      lastTsStart=tsStart;     
  }
}
