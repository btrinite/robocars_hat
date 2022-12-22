#include "MovingAverage.h"

byte triggerPin = USTriggerPin;
byte echoPin = USEchoPin;

#define TEMPERATURE 19.307
#define speedOfSoundInCmPerMicroSec  (0.03313 + 0.0000606 * TEMPERATURE) // Cair ≈ (331.3 + 0.606 ⋅ ϑ) m/s

MovingAverage <uint8_t, 4> rpm_filter;

void sensor_controler_trigger() {
  // Make sure that trigger pin is LOW.
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  // Hold trigger for 10 microseconds, which is signal for sensor to measure distance.
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
}
void sensor_controler_setup () {
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void sensor_controler_update () {
  static int distanceCm = -1;
  static int rpm = -1;
  
  if (PWM_read(US_PWM_CHANNEL)==HIGH) {
    unsigned long durationMicroSec = PWM();
    distanceCm = (int)(durationMicroSec / 2.0 * speedOfSoundInCmPerMicroSec);    
  }
  if (PWM_read(RPM_SENSOR_CHANNEL)==HIGH) {
    rpm = rpm_filter.add((int)PWM());
  }
  publish_sensors_state(distanceCm, rpm);
  sensor_controler_trigger();
}
