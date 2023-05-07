#include <PololuLedStrip.h>

// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1:
#define LED_PIN    RGBLEDPin

// How many NeoPixels are attached to the Arduino?
#define LED_COUNT RGBLEDCount
#define LED_STATUS  0 /*Status LED built on Hat is index 0*/

PololuLedStrip<LED_PIN> ledStrip;

#define TIMESTEPS      16 

rgb_color colors[LED_COUNT];

struct Led {
  unsigned char r;
  unsigned char g;
  unsigned char b;
  unsigned int timing;
} ;

struct Led ledControl[LED_COUNT];

void _setLed (unsigned char led, unsigned char r, unsigned char g, unsigned char b, unsigned int timing) {
  ledControl[led].r=r;
  ledControl[led].g=g;
  ledControl[led].b=b;
  ledControl[led].timing = timing;
}

void _resetAllLed() {
  for (int i=0;i<LED_COUNT; i++) {
    _setLed (i,0,0,0,0x0);
  }
}

void _setAllLed(unsigned char r, unsigned char g, unsigned char b, unsigned int timing) {
  for (int i=0;i<LED_COUNT; i++) {
    _setLed (i,r,g,b,timing);
  }
}

#ifdef ANIM_LED
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
#endif

void led_controler_setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  ledStrip.interruptFriendly=true;
  _resetAllLed();
  _setAllLed (0xff,0x7f,0,0xaaaa);
  ledStrip.write(colors, LED_COUNT);    
}

void led_controler_update () { //scheduled at 10hz
  static unsigned int seq=0;
  seq++;
  for(int i = 0; i < LED_COUNT; i++) {
    if ((ledControl[i].timing>>(seq%TIMESTEPS)) & 0x01) {
      colors[i].red = ledControl[i].r;
      colors[i].green = ledControl[i].g;
      colors[i].blue = ledControl[i].b;
    } else {
      colors[i].red = 0;
      colors[i].green = 0;
      colors[i].blue = 0;
    }
  }
  ledStrip.write(colors, LED_COUNT);    

  if ((seq%TIMESTEPS)<8) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {  
    digitalWrite(LED_BUILTIN, LOW);  
  }
   
}

unsigned char _alarm = 0x00;

void _update_alarm(unsigned char alarm) {
  if (alarm & (unsigned char )(1<<LED_CTRL_ALARM_BATTERY_LOW)) {
    _setAllLed (0xff,0,0,0xaaaa);
  
  }else if (alarm & (unsigned char )(1<<LED_CTRL_ALARM_STARTUP)) {
    _setLed (LED_STATUS, 0xff,0x7f,0,0xaaaa);    

  }else if (alarm & (unsigned char )(1<<LED_CTRL_ALARM_PDOWN_PWOFF)) {
    _setAllLed (0xff,0,0,0xffff);  
  
  }else if (alarm & (unsigned char )(1<<LED_CTRL_ALARM_PDOWN_REQ)) {
    _setLed (LED_STATUS, 0xff,0x0,0x0,0xa000);    
  
  }else if (alarm & (unsigned char )(1<<LED_CTRL_ALARM_RX_LOSS)) {
    _setLed (LED_STATUS, 0,127,0,0xaaaa);    
  
  }else if (alarm & (unsigned char )(1<<LED_CTRL_ALARM_LINK_LOSS)) {
    _setLed (LED_STATUS, 64,64,64,0xaaaa);        
  
  }else if (alarm & (unsigned char )(1<<LED_CTRL_ALARM_DRIVE_LOSS)) {
    _setLed (LED_STATUS, 0,0,127,0xaaaa);        
  
  }else {
    if (passthrough) {
      _setLed (LED_STATUS, 0,0,20,0xffff);      
    } else if (ignore_rc_state) {
      _setLed (LED_STATUS, 0,0,20,0x0101);
    } else {
      _setLed (LED_STATUS, 0,0,20,0xffff);    
    }
  }
}

void led_controler_set_alarm(unsigned char alarmId) {
  _alarm = _alarm | (unsigned char)(1<<alarmId);
  _update_alarm(_alarm);
  publish_alarm(alarmId);
}

void led_controler_reset_alarm(unsigned char alarmId) {
  _alarm = _alarm & (unsigned char)~(1<<alarmId);
  _update_alarm(_alarm);
}

void led_controler_set_other_led (unsigned char led, unsigned char r, unsigned char g, unsigned char b, unsigned int timing) {
  if ((led+1)<LED_COUNT) {
    _setLed (led+1, r, g, b, timing);
  }
}
