#include <PololuLedStrip.h>

// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1:
#define LED_PIN    RGBLEDPin

// How many NeoPixels are attached to the Arduino?
#define LED_COUNT RGBLEDCount

PololuLedStrip<LED_PIN> ledStrip;

#define TIMESTEPS      16 

rgb_color colors[LED_COUNT];

struct Led {
  unsigned char r;
  unsigned char g;
  unsigned char b;
  unsigned int timing;
} ;

struct Led ledControl;

void _setControlLed (unsigned char r, unsigned char g, unsigned char b, unsigned int timing) {
  ledControl.r=r;
  ledControl.g=g;
  ledControl.b=b;
  ledControl.timing = timing;
}

#ifdef EXTRA_LED

#define EXTRA_LED_ALARM_BATTERY_LOW 1
#define EXTRA_LED_ALARM_STARTUP 2
#define EXTRA_LED_RX_LOSS 3
#define EXTRA_LED_ALARM_PASSTHROUGH 4
#define EXTRA_LED_ALARM_LINK_LOSS 5
#define EXTRA_LED_ALARM_DRIVE_LOSS 6
#define EXTRA_LED_READY 7


unsigned char _extra_led_status = -1;

void _setExtraLed (unsigned char led, unsigned char r, unsigned char g, unsigned char b) {
  colors[led].red=r;
  colors[led].green=g;
  colors[led].blue=b;
}

void _setAllExtraLed (unsigned char r, unsigned char g, unsigned char b) {
  //update extra led (starting pos 1)
  for(int j = 1; j < LED_COUNT; j++) {
    colors[j].red=r;
    colors[j].green=g;
    colors[j].blue=b;
  }
}

void extraLedSparkle(byte red, byte green, byte blue, int SpeedDelay) {
  int Pixel = random(LED_COUNT-1)+1;
  _setExtraLed (Pixel, red, green, blue);
  ledStrip.write(colors, LED_COUNT);    
  delay(SpeedDelay);
  _setExtraLed (Pixel, 0, 0, 0);
}

void extraLedStrobe(byte red, byte green, byte blue, int StrobeCount, int FlashDelay){
  for(int j = 0; j < StrobeCount; j++) {
    _setAllExtraLed(red,green,blue);
    ledStrip.write(colors, LED_COUNT);    
    delay(FlashDelay);
    _setAllExtraLed(0,0,0);
    ledStrip.write(colors, LED_COUNT);    
    delay(FlashDelay);
  }
}

void extraLedReady() {
  _setAllExtraLed(0,0,0);
  _setExtraLed (1+FRONT_RIGHT_FULL_BEAM, 0x0, 0x0, 0xaa);
  _setExtraLed (1+FRONT_RIGHT_LOW_BEAM, 0x0, 0x0, 0x55);
  _setExtraLed (1+FRONT_LEFT_LOW_BEAM, 0x0, 0x0, 0x55);
  _setExtraLed (1+FRONT_LEFT_FULL_BEAM, 0x0, 0x0, 0xaa);
  _setExtraLed (1+REAR_LEFT_STOP, 0x0, 0x55, 0x0);
  _setExtraLed (1+REAR_RIGHT_STOP, 0x0, 0x55, 0x0);
  ledStrip.write(colors, LED_COUNT);      
}

void extraLedBatteryLow () {
  _setAllExtraLed(0,0,0);
  _setExtraLed (1+FRONT_RIGHT_FULL_BEAM, 0x0, 0x55, 0x00);
  _setExtraLed (1+FRONT_RIGHT_LOW_BEAM, 0x0, 0x55, 0x00);
  _setExtraLed (1+FRONT_LEFT_LOW_BEAM, 0x00, 0x55, 0x00);
  _setExtraLed (1+FRONT_LEFT_FULL_BEAM, 0x00, 0x55, 0x00);
  _setExtraLed (1+REAR_LEFT_STOP, 0x0, 0x55, 0x0);
  _setExtraLed (1+REAR_RIGHT_STOP, 0x0, 0x55, 0x0);  
}

void extraLedRxLoss () {
  _setAllExtraLed(0,0,0);
  _setExtraLed (1+FRONT_RIGHT_FULL_BEAM, 0xaa, 0x0, 0x0);
  _setExtraLed (1+FRONT_RIGHT_LOW_BEAM, 0x55, 0x0, 0x0);
  _setExtraLed (1+FRONT_LEFT_LOW_BEAM, 0x55, 0x0, 0x0);
  _setExtraLed (1+FRONT_LEFT_FULL_BEAM, 0xaa, 0x0, 0x0);
  _setExtraLed (1+REAR_LEFT_STOP, 0x0, 0x55, 0x0);
  _setExtraLed (1+REAR_RIGHT_STOP, 0x0, 0x55, 0x0);
  ledStrip.write(colors, LED_COUNT);      
}

void extraLedStartup () {
  _setAllExtraLed(0,0,0);
  _setAllExtraLed(0,0,0);
  _setExtraLed (1+FRONT_RIGHT_FULL_BEAM, 0x7f, 0xff, 0x0);
  _setExtraLed (1+FRONT_RIGHT_LOW_BEAM, 0x7f, 0xff, 0x0);
  _setExtraLed (1+FRONT_LEFT_LOW_BEAM, 0x7f, 0xff, 0x0);
  _setExtraLed (1+FRONT_LEFT_FULL_BEAM, 0x7f, 0xff, 0x0);
  _setExtraLed (1+REAR_LEFT_STOP, 0x0, 0x55, 0x0);
  _setExtraLed (1+REAR_RIGHT_STOP, 0x0, 0x55, 0x0);
  ledStrip.write(colors, LED_COUNT);      
}

void extraLedDriveLoss () {
  _setAllExtraLed(0,0,0);
  _setExtraLed (1+FRONT_RIGHT_FULL_BEAM, 0x7f, 0x0, 0x0);
  _setExtraLed (1+FRONT_RIGHT_LOW_BEAM, 0x7f, 0x0, 0x0);
  _setExtraLed (1+FRONT_LEFT_LOW_BEAM, 0x7f, 0x0, 0x0);
  _setExtraLed (1+FRONT_LEFT_FULL_BEAM, 0x7f, 0x0, 0x0);
  _setExtraLed (1+REAR_LEFT_STOP, 0x0, 0x55, 0x0);
  _setExtraLed (1+REAR_RIGHT_STOP, 0x0, 0x55, 0x0);
  ledStrip.write(colors, LED_COUNT);      
}

void extraLedUpdate(unsigned char new_mode) {
  if (new_mode != _extra_led_status) {
    _extra_led_status = new_mode;
    switch (_extra_led_status) {
      case (EXTRA_LED_ALARM_BATTERY_LOW):
        extraLedBatteryLow();
        break;
      case (EXTRA_LED_ALARM_STARTUP):
        extraLedStartup();
        break;
      case (EXTRA_LED_RX_LOSS):
        extraLedRxLoss();
        break;
      case (EXTRA_LED_ALARM_LINK_LOSS):
        break;
      case (EXTRA_LED_ALARM_DRIVE_LOSS):
        extraLedDriveLoss();
        break;
      case (EXTRA_LED_READY):
        extraLedReady();
        break;
    }
  }
  
}
#endif

void led_controler_setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  ledStrip.interruptFriendly=true;
#ifdef EXTRA_LED
  _setAllExtraLed(0,0,0);
#endif
  ledStrip.write(colors, LED_COUNT);    
}

void led_controler_update () {
  static unsigned int seq=0;
  seq++;
  if ((ledControl.timing>>(seq%TIMESTEPS)) & 0x01) {
    colors[0].red = ledControl.r;
    colors[0].green = ledControl.g;
    colors[0].blue = ledControl.b;
    ledStrip.write(colors, 1);    
  } else {
    colors[0].red = 0;
    colors[0].green = 0;
    colors[0].blue = 0;
    ledStrip.write(colors, 1);    
  }
  if ((seq%TIMESTEPS)<8) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {  
    digitalWrite(LED_BUILTIN, LOW);  
  } 
}

unsigned char _alarm = 0x00;

void _update_alarm(unsigned char alarm) {
  if (alarm & (unsigned char )(1<<LED_CTRL_ALARM_BATTERY_LOW)) {
    #ifdef EXTRA_LED
    extraLedUpdate (EXTRA_LED_ALARM_BATTERY_LOW);
    #endif
    _setControlLed (255,0,0,0xafaf);    
  
  }else if (alarm & (unsigned char )(1<<LED_CTRL_ALARM_STARTUP)) {
    #ifdef EXTRA_LED
    extraLedUpdate (EXTRA_LED_ALARM_STARTUP);
    #endif
    _setControlLed (0xff,0x7f,0,0xaaaa);    
  
  }else if (alarm & (unsigned char )(1<<LED_CTRL_ALARM_RX_LOSS)) {
    #ifdef EXTRA_LED
    extraLedUpdate (EXTRA_LED_RX_LOSS);
    #endif
    _setControlLed (0,127,0,0xaaaa);    
  
  }else if (alarm & (unsigned char )(1<<LED_CTRL_ALARM_LINK_LOSS)) {
    #ifdef EXTRA_LED
    extraLedUpdate (EXTRA_LED_ALARM_LINK_LOSS);
    #endif
    _setControlLed (64,64,64,0xaaaa);        
  
  }else if (alarm & (unsigned char )(1<<LED_CTRL_ALARM_DRIVE_LOSS)) {
    #ifdef EXTRA_LED
    extraLedUpdate (EXTRA_LED_ALARM_DRIVE_LOSS);
    #endif
    _setControlLed (0,0,127,0xaaaa);        
  
  }else {
    #ifdef EXTRA_LED
    extraLedUpdate (EXTRA_LED_READY);
    #endif
    if (passthrough) {
      _setControlLed (0,0,20,0xffff);      
    } else if (ignore_rc_state) {
      _setControlLed (0,0,20,0x0101);
    } else {
      _setControlLed (0,0,20,0xffff);    
    }
  }
}

void led_controler_set_alarm(unsigned char alarmId) {
  _alarm = _alarm | (unsigned char)(1<<alarmId);
  _update_alarm(_alarm);
}

void led_controler_reset_alarm(unsigned char alarmId) {
  _alarm = _alarm & (unsigned char)~(1<<alarmId);
  _update_alarm(_alarm);
}
