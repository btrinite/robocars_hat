#include "MovingAverage.h"

byte triggerPin = USTriggerPin;
byte echoPin = USEchoPin;

#define TEMPERATURE 19.307
#define speedOfSoundInCmPerMicroSec  (0.03313 + 0.0000606 * TEMPERATURE) // Cair ≈ (331.3 + 0.606 ⋅ ϑ) m/s

MovingAverage <int32_t, 4> rpm_filter;

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
  #ifdef USE_US_SENSOR
  pinMode(echoPin, INPUT);
  #endif
}

void sensor_controler_update () {
  static int distanceCm = -1;
  static int rpm = -1;

  #ifdef USE_US_SENSOR
  if (PWM_read(US_PWM_CHANNEL)==HIGH) {
    unsigned long durationMicroSec = PWM();
    distanceCm = (int)(durationMicroSec / 2.0 * speedOfSoundInCmPerMicroSec);    
  }
  #endif
  if (PWM_read(RPM_SENSOR_CHANNEL)==HIGH) {
    rpm = rpm_filter.add((uint32_t)max(min(RPM_SENSOR_MAX_VALUE,PWM()),RPM_SENSOR_MIN_VALUE));
  } else {
    rpm = rpm_filter.add((uint32_t)RPM_SENSOR_MAX_VALUE);    
  }
  publish_sensors_state(distanceCm, rpm);

  #ifdef USE_US_SENSOR
  sensor_controler_trigger();
  #endif
}
