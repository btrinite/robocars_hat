#include <PololuLedStrip.h>

// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1:
#define LED_PIN    RGBLEDPin

// How many NeoPixels are attached to the Arduino?
#define LED_COUNT RGBLEDCount

PololuLedStrip<LED_PIN> ledStrip;
rgb_color colors[LED_COUNT];

#define TIMESTEPS      16 

struct Led {
  unsigned char r;
  unsigned char g;
  unsigned char b;
  unsigned int timing;
} ;

struct Led ledControl;

void _setLed (unsigned char r, unsigned char g, unsigned char b, unsigned int timing) {
  ledControl.r=r;
  ledControl.g=g;
  ledControl.b=b;
  ledControl.timing = timing;
}


void led_controler_setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  ledStrip.interruptFriendly=true;
}

void led_controler_update () {
  static unsigned int seq=0;
  seq++;
  if ((ledControl.timing>>(seq%TIMESTEPS)) & 0x01) {
    colors[0].red = ledControl.r;
    colors[0].green = ledControl.g;
    colors[0].blue = ledControl.b;
    ledStrip.write(colors, LED_COUNT);    
  } else {
    colors[0].red = 0;
    colors[0].green = 0;
    colors[0].blue = 0;
    ledStrip.write(colors, LED_COUNT);    
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
    _setLed (255,0,0,0xafaf);    
  }else if (alarm & (unsigned char )(1<<LED_CTRL_ALARM_STARTUP)) {
    _setLed (0xff,0x7f,0,0xaaaa);    
  }else if (alarm & (unsigned char )(1<<LED_CTRL_ALARM_RX_LOSS)) {
    _setLed (0,0,127,0xaaaa);    
  }else if (alarm & (unsigned char )(1<<LED_CTRL_ALARM_LINK_LOSS)) {
    _setLed (0,127,0,0xaaaa);        
  }else if (alarm & (unsigned char )(1<<LED_CTRL_ALARM_DRIVE_LOSS)) {
    _setLed (127,127,127,0xaaaa);        
  }else {
    _setLed (0,20,0,0xffff);
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
