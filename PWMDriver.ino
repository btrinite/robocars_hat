
unsigned char _pwm_received = 0;

#ifdef Use_Servo
#include <Servo.h>

Servo throttle_servo;  // create servo object to control a servo
Servo steering_servo;  // create servo object to control a servo

void _pwmdriver_set_throttle_output (int periodus)
{
  if (failsafe==0) {
    throttle_servo.writeMicroseconds(periodus);
  }
}

void _pwmdriver_set_steering_output (int periodus)
{
  if (failsafe==0) {
    steering_servo.writeMicroseconds(periodus);
  }
}

void pwmdriver_set_throttle_output (int periodus)
{
  _pwm_received=1;
  _pwmdriver_set_throttle_output (periodus);
}

void pwmdriver_set_steering_output (int periodus)
{
  _pwmdriver_set_steering_output (periodus);
}

void pwmdriver_setup(int throttle_pin, int steering_pin)
{
  throttle_servo.attach(throttle_pin);
  steering_servo.attach(steering_pin);
}

void pwmdriver_check_failsafe() {
  // mostly for faisafe check
  if (drive_loss==0) {
    if (_pwm_received==0) {
      led_controler_set_alarm(LED_CTRL_ALARM_DRIVE_LOSS);
      drive_loss=1;  
    }
  } else {
    if (_pwm_received > 0) {
      led_controler_reset_alarm(LED_CTRL_ALARM_DRIVE_LOSS);
      drive_loss=0;  
    }
  }
  _pwm_received = 0;
  if (!com_controler_check_status() || (rx_loss==1) || (drive_loss==1) || (battery_low==1)) {
    _pwmdriver_set_throttle_output (PWN_out_throttle_Failsafe);
    _pwmdriver_set_steering_output (PWN_out_steering_Failsafe);  
    failsafe = 1;
  } else {
    failsafe = 0;
  }
}

#else

//https://www.arduino.cc/reference/en/language/functions/analog-io/analogwrite/
#define PWM_Frequency (33333)
#define PWM_Divisor (1024)
#define PWM_PeriodNs (1000000/(PWM_Frequency/PWM_Divisor))
#define PWM_stepNs (PWM_PeriodNs / 255)

char _throttle_pin = -1;
char _steering_pin = -1;

void setPwmFrequency(int pin, int divisor) {
   byte mode;
   if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
      switch(divisor) {
         case 1: mode = 0x01; break;
         case 8: mode = 0x02; break;
         case 64: mode = 0x03; break;
         case 256: mode = 0x04; break;
         case 1024: mode = 0x05; break;
         default: return;
      }
      if(pin == 5 || pin == 6) {
         TCCR0B = TCCR0B & 0b11111000 | mode;
      } else {
         TCCR1B = TCCR1B & 0b11111000 | mode;
      }
   } else if(pin == 3 || pin == 11) {
      switch(divisor) {
         case 1: mode = 0x01; break;
         case 8: mode = 0x02; break;
         case 32: mode = 0x03; break;
         case 64: mode = 0x04; break;
         case 128: mode = 0x05; break;
         case 256: mode = 0x06; break;
         case 1024: mode = 0x7; break;
         default: return;
      }
      TCCR2B = TCCR2B & 0b11111000 | mode;
   }
}

void _pwmdriver_set_throttle_output (int durationNs)
{
  if (failsafe==0) {
    analogWrite(_throttle_pin, int(durationNs / PWM_stepNs));
  }
}

void _pwmdriver_set_steering_output (int durationNs)
{
  if (failsafe == 0) {
    analogWrite(_steering_pin, int(durationNs / PWM_stepNs)); 
  }
}

void pwmdriver_set_throttle_output (int durationNs)
{
  _pwm_received++;
  _pwmdriver_set_throttle_output (durationNs);
}

void pwmdriver_set_steering_output (int durationNs)
{
  _pwmdriver_set_steering_output(durationNs)
}

void pwmdriver_setup(int throttle_pin, int steering_pin)
{
    _throttle_pin = throttle_pin;
    _steering_pin = steering_pin;
    pinMode(_throttle_pin, OUTPUT);  // sets the pin as output
    pinMode(_steering_pin, OUTPUT);  // sets the pin as output
    
    setPwmFrequency (_throttle_pin, PWM_Divisor); 
    setPwmFrequency (_steering_pin, PWM_Divisor);    
}

void pwmdriver_check_failsafe() {
  // mostly for faisafe check
  if (drive_loss==0) {
    if (_pwm_received==0) {
      led_controler_set_alarm(LED_CTRL_ALARM_DRIVE_LOSS);
      drive_loss=1;  
    }
  } else {
    if (_pwm_received > 0) {
      led_controler_reset_alarm(LED_CTRL_ALARM_DRIVE_LOSS);
      drive_loss=0;  
    }
  }
  _pwm_received = 0;
  if (!com_controler_check_status() || (rx_loss==1) || (drive_loss==1) || (battery_low==1)) {
    _pwmdriver_set_throttle_output (PWN_out_throttle_Failsafe);
    _pwmdriver_set_steering_output (PWN_out_steering_Failsafe);  
    failsafe = 1;
  } else {
    failsafe = 0;
  }
}

#endif
