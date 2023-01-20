// This module monitor if SBC is powerup and can trigger shutdown
// for rpi4, need to change bootloader config
// 'sudo rpi-eeprom-config' to dump config
// 'sudo rpi-eeprom-config --edit' to edit config
// set POWER_OFF_ON_HALT=1
// set WAKE_ON_GPIO=0
// then edit /boot/firmware/config.txt
// and add : 'dtoverlay=gpio-shutdown,gpio_pin=19'


boolean buttonState = HIGH; 
int pressed=0;

#define ADC_PRE_DIVIDER 11
#define UNITUV (5000000/1024)
#define SBC_LOW_VOLTAGE 2500

boolean debounceButton(boolean state)
{
  boolean stateNow = digitalRead(SwOffButtonPin);
  if(state!=stateNow)
  {
    delay(10);
    stateNow = digitalRead(SwOffButtonPin);
  }
  return stateNow;
  
}

void sbc_power_controler_setup () {
  pinMode(SwOffButtonPin, INPUT_PULLUP);
  //pinMode(SbcPwOffTriger, INPUT_PULLUP);
  digitalWrite(SbcPwOffTriger, HIGH); // do this first
  pinMode(SbcPwOffTriger, OUTPUT);
  
}

void sbc_power_controler_update () {
  int sbc3v3raw;
  int sbc3v3mv;
  
  sbc3v3raw = analogRead(SbcPwMonitorPin);
  sbc3v3mv = (sbc3v3raw*UNITUV)/1000;
  if (sbc3v3mv < SBC_LOW_VOLTAGE){
    led_controler_set_alarm(LED_CTRL_ALARM_PDOWN_PWOFF);
  } else {
    led_controler_reset_alarm(LED_CTRL_ALARM_PDOWN_PWOFF);
    
  }
  if(debounceButton(buttonState) == LOW && buttonState == HIGH) {
    buttonState = LOW;
  }
  else if(debounceButton(buttonState) == HIGH && buttonState == LOW) {
    buttonState = HIGH;
  }
  if (buttonState==LOW) {
    digitalWrite(SbcPwOffTriger, LOW);
    led_controler_set_alarm(LED_CTRL_ALARM_PDOWN_REQ);
  }
}
