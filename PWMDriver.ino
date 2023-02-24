
unsigned int _pwm_received = 0;

#ifdef Use_PWMServoLib

#include <PWMServo.h>

PWMServo throttle_ctrl;  // create servo object to control a servo
PWMServo steering_ctrl;  // create servo object to control a servo


char _throttle_pin = -1;
char _steering_pin = -1;
#define ENFORCE_IDLE_OUTPUT_CYCLE 10

void pwmdriver_set_throttle_output (int durationNs) {
  _pwm_received++; //Used to detect if we are getting commands from host
  _pwmdriver_set_throttle_output (durationNs);  
}


void pwmdriver_set_steering_output (int durationNs) {
  _pwmdriver_set_steering_output(durationNs);
}

void _pwmdriver_set_throttle_output (int durationNs) {
  static unsigned enforce_idle_output = ENFORCE_IDLE_OUTPUT_CYCLE;
  if (failsafe==0) {
    // We are safe, we can drive output
    if (enforce_idle_output == 0) {
      // Idle period has been observed, we can passthrough throttle value from host
      if (durationNs<PWN_out_throttle_Idle){
        throttle_ctrl.write(map(durationNs,1000,PWN_out_throttle_Idle,0,90));                  
      } else if (durationNs > PWN_out_throttle_Idle) {
        throttle_ctrl.write(map(durationNs,PWN_out_throttle_Idle,2000,90,180));                  
      } else {
        throttle_ctrl.write(90);                      
      }
    } else {
      // Idle period has not been observed, we enforce idle signal
      if (enforce_idle_output>0)
      {
        throttle_ctrl.write(90);                      
        enforce_idle_output--;  
      }  
    }
  }
  if ((pwm_calibrated==0) && (ignore_rc_state==false)) {
    // Calibration lost means that most likely, power train has been restarted, let's observe idle period again
    enforce_idle_output = ENFORCE_IDLE_OUTPUT_CYCLE;    
  }
}


void _pwmdriver_set_steering_output (int durationNs) {
  static unsigned enforce_idle_output = ENFORCE_IDLE_OUTPUT_CYCLE;
  if (failsafe==0) {
    // We are safe, we can drive output
    if (enforce_idle_output == 0) {
      // Idle period has been observed, we can passthrough throttle value from host
      if (durationNs<PWN_out_steering_Idle){
        steering_ctrl.write(map(durationNs,1000,PWN_out_steering_Idle,0,90));                  
      } else if (durationNs > PWN_out_steering_Idle) {
        steering_ctrl.write(map(durationNs,PWN_out_steering_Idle,2000,90,180));                  
      } else {
        steering_ctrl.write(90);                      
      }
    } else {
      // Idle period has not been observed, we enforce idle signal
      if (enforce_idle_output>0)
      {
        steering_ctrl.write(90);                      
        enforce_idle_output--;  
      }  
    }
  }
  if ((pwm_calibrated==0) && (ignore_rc_state==false)) {
    // Calibration lost means that most likely, power train has been restarted, let's observe idle period again
    enforce_idle_output = ENFORCE_IDLE_OUTPUT_CYCLE;      
  }
}

void pwmdriver_setup(int throttle_pin, int steering_pin) {
  _throttle_pin = throttle_pin;
  _steering_pin = steering_pin;  
}

void pwmdriver_attach() {
  throttle_ctrl.attach(_throttle_pin, 1000, 2000);         
  steering_ctrl.attach(_steering_pin, 1000, 2000);  
  //Set default idle value as soon as possible       
  _pwmdriver_set_throttle_output (PWN_out_throttle_Idle);
  _pwmdriver_set_steering_output (PWN_out_steering_Idle);    
}

void pwmdriver_detach() {
  throttle_ctrl.detach();         
  steering_ctrl.detach();         
}

void pwmdriver_check_failsafe() {
  // mostly for failsafe check
  if (ignore_rc_state==false) {
    if (drive_loss==0) {
      // We are actively drived by host
      if (_pwm_received==0) {
        // No new PWM order received from host, assume we lost the host
        led_controler_set_alarm(LED_CTRL_ALARM_DRIVE_LOSS);
        drive_loss=1;  
      }
    } else {
      // Host was previously lost, do we have any new order
      if (_pwm_received > 0) {
        //Host has resume
        led_controler_reset_alarm(LED_CTRL_ALARM_DRIVE_LOSS);
        drive_loss=0;  
      }
    }
    _pwm_received = 0;
  }
  #ifdef IMPLEMENT_FAILSAFE
  if (!com_controler_check_status() || (rx_loss==1) || (drive_loss==1) || (battery_low==1)) {
    // if we met any of the following condition, force output to idle
    // Lost of com controller, or lost of radio or lost of host or battery low
    _pwmdriver_set_throttle_output (PWN_out_throttle_Idle);
    _pwmdriver_set_steering_output (PWN_out_steering_Idle);  
    failsafe = 1;
  } else {
    failsafe = 0;
  }
  #endif
}

#endif
