//=========================================
// Configutation part
// - Configure communication protocol 
// - Configure PWM Library
//=========================================

//=========================================
// Features 
//=========================================

// Select communication protocole to use (Use_ROSSerial for ROS topics overs serial, Use_SLIPSerial for SLIP over serial)
//#define Use_ROSSerial
#define Use_SimpleSerial

// Delay startup to avoid conflict with host bootloader (u-boot for example)
#define STARTUP_DELAYED

// Implement failsafe protocol (unless you're sure) 
#define IMPLEMENT_FAILSAFE

// Implement extra led aninmation (could affect PWM Sampler precision)
//#define EXTRA_LED

// Aux2 grounded feature
//#define AUX2_IGNORE_RC_STATE
//#define AUX2_PASSTHROUGH 

//#define USE_US_SENSOR

#define USE_SBCPWCTRL

// New PWM input wiring
#define ALL_PWM_SIGNALS_ON_P2

//=========================================
// Implem option 
//=========================================

// micros() as 4us resolution, while this timer2 lib provides 0.5us
#define USE_TIMER2_COUNTER

#ifdef USE_TIMER2_COUNTER
#include <eRCaGuy_Timer2_Counter.h>
#endif



//=========================================
// PWM Library
//=========================================

// Select PWM Library

//#define Use_ServoLib
// Uncomment above to user Servo Lib
// Servo arduino library offers better resolution than simple analogWrite.
// However we did a little change compared to original version (from here : https://github.com/arduino-libraries/Servo),
// to reduce memory footprint, we changed SERVOS_PER_TIMER from 12 to 4 (src/Servo.h)

#define Use_PWMServoLib
// Uncomment above to user PWMServo Lib
// PWMServo arduino library looks more stable than Servo Library
// https://github.com/PaulStoffregen/PWMServo

//#define Use_BasicPWMDriver

// ROS, if used, define serial speed
#ifdef Use_ROSSerial

#define ROSSERIAL_BAUD 1000000 

#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>

#endif

#ifdef Use_SimpleSerial

//#define SERIAL_BAUD 1000000
#define SERIAL_BAUD 250000

#endif

// define Alarm
#define LED_CTRL_ALARM_BATTERY_LOW  0 // Means that battery voltage is low (Red inversed 2 flash blinking each half second)
#define LED_CTRL_ALARM_STARTUP      1 // Means that Arduino just started/restarted (Orange blinking fast)
#define LED_CTRL_ALARM_PDOWN_PWOFF  2 // - (Red steady)
#define LED_CTRL_ALARM_PDOWN_REQ    3 // - (Red blinking)
#define LED_CTRL_ALARM_RX_LOSS      4 // Means PCM RX Signal not detected/loss (Green blinking fast)
#define LED_CTRL_ALARM_LINK_LOSS    5 // Means ROS not connected/disconnected (White blinking fast)
#define LED_CTRL_ALARM_DRIVE_LOSS   6 // Means no PWM order received, failsafe (Blue blinking fast)
// Passthrough : Light Blue steady
// Ignore RC state : Light Blue blinking
// Ok : Light Blue steady

unsigned int PWM_in_throttle_idle = 0;
unsigned int PWM_in_steering_idle = 0;

#define PWN_out_throttle_Idle 1500 
#define PWN_out_steering_Idle 1500 

// Battery Low VOltage
#define LIPO_CELL_LOW_VOLTAGE 3200
#define VBAT_LOW_VOLTAGE 5500

// RPM Sensor max value
#define RPM_SENSOR_MAX_VALUE 8000
#define RPM_SENSOR_MIN_VALUE 500

// Definition of pin assignment
//

// PWM Outputs, require timer 1
#define pwmOutThrottlePin     9
#define pwmOutSteeringPin     10

#ifndef ALL_PWM_SIGNALS_ON_P2
// PWM/sensors input
#define pwmInThrottlePin      6
#define pwmInSteeringPin      7
#define pwmInAux1Pin          12
#define pwmInAux2Pin          8
#define USEchoPin             2
#define RPMSignalPin          4
#else
// PWM/sensors input all wired on P2
#define pwmInThrottlePin      6
#define pwmInSteeringPin      7
#define pwmInAux1Pin          3 //instead of 12
#define pwmInAux2Pin          2 //instead of 8
#define USEchoPin             12 //instead of 2
#define RPMSignalPin          4
#endif

// on Port D
#define RGBLEDPin             5

// Analog INPUT (ADC)
#define batteryCell1Pin       A0
#define batteryCell2Pin       A2
#define batteryVBatPin        A1

#define debugPin              A5

// Simple GPIO Output
#ifndef ALL_PWM_SIGNALS_ON_P2
#define USTriggerPin          3
#else
#define USTriggerPin          8 //instead of 3
#endif

#define SwOffButtonPin        12 // was A2, mistake
#define SbcPwMonitorPin       A3
#define SbcPwOffTriger        11

// Other sizing
#ifdef EXTRA_LED
#define RGBLEDCount  7
#else
#define RGBLEDCount  1
#endif


// Additional LEDs
#define REAR_LEFT_STOP 0
#define REAR_RIGHT_STOP 1
#define FRONT_RIGHT_FULL_BEAM 2
#define FRONT_RIGHT_LOW_BEAM 3
#define FRONT_LEFT_LOW_BEAM 4
#define FRONT_LEFT_FULL_BEAM 5

//GLobal state
unsigned char rx_loss=0;
unsigned char drive_loss=0;
unsigned char battery_low=0;
unsigned char failsafe = 0;
unsigned char pwm_calibrated = 0;

boolean passthrough = false;
boolean ignore_rc_state = false;

bool detect_aux2_grounded () {
  pinMode(pwmInAux2Pin, INPUT_PULLUP);
  delay(100);
  int sensorVal = digitalRead(pwmInAux2Pin);
  return (sensorVal==0); 
}

void setup (void) {

  #ifdef USE_SBCPWCTRL
  sbc_power_controler_setup();
  #endif

  Serial.begin(115200);
  Serial.println ("Robocars Hat starting");

  pinMode (debugPin, OUTPUT);
  digitalWrite(debugPin, LOW);

  #ifdef USE_TIMER2_COUNTER
  timer2.setup();
  #endif

  pwmdriver_setup(pwmOutThrottlePin, pwmOutSteeringPin);
  pwmdriver_attach();
  #ifdef AUX2_PASSTHROUGH
    passthrough = detect_aux2_grounded();
    Serial.println ("AUX2 : Mode passthrough");
  #endif
  #ifdef AUX2_IGNORE_RC_STATE
    ignore_rc_state = detect_aux2_grounded();
    Serial.println ("AUX2 : Mode Ignore RC state");
  #endif
  setup_pwm_sampler();
  led_controler_setup();
  LIPO_watcher_setup();
  sensor_controler_setup();
  #ifdef STARTUP_DELAYED
  delay(10000);
  #endif

  com_controler_setup();
  led_controler_set_alarm(LED_CTRL_ALARM_STARTUP); 
}

int aux1In=1500;
int aux2In=1500;
int throttleIn=0;
int steeringIn=0;

void loop() {
  static unsigned int _cnt=0;
  static unsigned long lastTsStart = 0;
  unsigned long tsStart = 0;

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
        #ifdef USE_SBCPWCTRL
        sbc_power_controler_update();
        #endif
      }
      //Task to be done at 10Hz
      if (_cnt%10 == 5) {
        led_controler_update();
        pwmdriver_check_failsafe();
        pwm_sampler_check_failsafe();
        if (pwm_calibrated==1) {
          publish_calibration_state (PWM_in_throttle_idle, PWM_in_steering_idle);
        }
      }
      //Task to be done at 33Hz
      if (_cnt%3 == 2) {
#ifdef EXTRA_LED
        if (aux1In > 1600 && failsafe==0) {
          extraLedSparkle (0xaa,0xaa,0xaa,5);
        }
#endif
      }  

      //Task to be done at 50Hz
      if (_cnt%2 == 1) {
        sensor_controler_update();
      }  

      //Task to be done at 100Hz
      if (_cnt%1 == 0) {
        pwm_sampler_update(&throttleIn, &steeringIn, &aux1In, &aux2In);
        if (pwm_calibrated==0) {
          PWM_in_throttle_idle = throttleIn;
          PWM_in_steering_idle = steeringIn;
        } else {
          publish_channels_state (throttleIn, steeringIn, aux1In, aux2In);
          if (passthrough) {
            pwmdriver_set_throttle_output (throttleIn);
            pwmdriver_set_steering_output (steeringIn);
          }
        }
      }
      lastTsStart=tsStart;     
    }
  } else {
      // Time counter overflow
      lastTsStart=tsStart;     
  }
}
