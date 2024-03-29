
/*  Kelvin Nelson 24/07/2019
 *  
 *  Pulse Width Modulation (PWM) decoding of RC Receiver with failsafe
 *  
 *  This code contains easy to use functions to measure square wave signals on any arduiuno pro mini, nano or uno pins, excluding A6 and A7.
 *  The code is intended to be used with RC receivers, but could also be used in most other PWM measurement applications as a direct replacement for pulseIn(PIN, HIGH). 
 *  (to date it hasn't been tested at a frequency greater than 1khz or on an arduino mega)
 *  
 *  An RC signal pulse can be converted from a pulse width duration (1000-2000uS) at each input pin into an -+100% (-+1.0) output for use in a sketch.
 *  The calibration for this conversion plus a failsafe setting can be set for each channel. (fail safe tolerances 10-330Hz and 500-2500uS). 
 *  
 *  The raw data for each pin can also be extracted, i.e. time of pulse, pulse width, frame length, duty and frequency.
 *  
 *  Set-up is quick, the organisation of this file is as follows:
 *  
 *    - Overview of the code
 *    - List of functions
 *    - How to use, including example sketches
 *    - User defined variables -> specify input pins, transmitter calibration, and failsafe.
 *    - Global variables and functions
 *  
 *  OVERVIEW OF THE CODE:
 *  
 *  The code enables pin change interrupts on the selected pins by setting the appropriate registers.
 *  
 *  A voltage change on any of the selected pins will trigger one of three Interrupt Service Routines depending on which register the pin belongs to.
 *    - ISR(PCINT0_vect), ISR(PCINT1_vect) or ISR(PCINT2_vect)
 *    
 *  Within each ISR the code determines which pin has changed, and makes a note of the time before returning back to the main loop().
 *  
 *  The time intervals between pin changes are used to calculate pulse width and frame length. 
 *  
 *  Flags are set by the ISR to indicate when new pulses are received.
 *  
 *  The Flags are then used to extract and process the data collected by each ISR.
 * 
 *  Although it's not exactly the same, this code follows similar principles to those explained in this video: https://youtu.be/bENjl1KQbvo
 * 
 */
// LIST OF FUNCTIONS:
// OUTPUT TYPE    NAME OF FUNCTION           NOTES

// void           setup_pwmRead()            initialise the PWM measurement using pin change interrupts

// RC RECEIVER DECODING
// boolean        RC_avail()                 returns a HIGH when new RC data is available
// float          RC_decode(channel number)  decodes the selected RC channel into the range +-100%, and applies a failsafe.
// void           print_RCpwm()              Prints the RC channel raw data to serial port (used for calibration).

// GENERIC PWM MEASUREMENTS
// boolean        PWM_read(channel number)   returns a HIGH when a new pulse has been detected on a particular channel. 
//                                           The function saves the pulse data to variables outside the interrupt routines
//                                           and must be called just before using the rest of PWM functions.
// unsigned long  PWM_time()                 returns the time at the start of pulse 
// float          PWM()                      returns the pulse width
// float          PWM_period()               returns the time between pulses
// float          PWM_freq()                 calculates the frequency
// float          PWM_duty()                 calculates the duty

// NOTE: PWM_read(CH) and RC_decode(CH) use the same flags to detect when new data is available, meaning data could be lost if both are used on the same channel at the same time.
// SUGESTION: if you want to use PWM_read(CH) to find the frame rate of an RC channel call it before RC_decode(CH). The output from RC_decode(CH) will then default to the failsafe.

// HOW TO USE, including example sketches

// under the "USER DEFINED VARIABLES" title in the code below:
//
//    Step 1: enter the input pins into the array pwmPIN[] = {}. 
//
//            - Any number of pins can be entered into pwmPIN[] (pins available 0 - 13 and A0 - A5)
//            - The pins do not need to be in numerical order, for example pwmPIN[] = {A0,5,6,10,8} for 5 channels, or pwmPIN[] = {A0,5} for 2 channels
//            - The first element in the array is the pin number for  "channel 1", and the second is the pin number for "channel 2"... etc.
//            - All pins connected to the RC receiver need to be at the start of the array. i.e. the first 2 channels could be RC inputs and the 3rd channel could be connected to another device like the echo pin of an ultrasonic sensor.
//
//    Step 2: if an RC receiver is connected to all of the inputs then set RC_inputs to 0, if not specify the number of channels connected to the receiver i.e. RC_inputs = 2;
//
//    Step 3: calibrate your transmitter by uploading a simple sketch with this .ino file included in the sketch folder, and print the raw PWM values to serial (alternatively copy and paste the functions needed into the sketch).
//            Using the info from the serial monitor manually update the values in arrays RC_min[], RC_mid[], RC_max[] to suit your transmitter (use full rates to get the best resolution).
        
//            an example sketch for printing the RC channel PWM data to serial. 
              /* 
              void setup()  {
                  setup_pwmRead();
                  Serial.begin(9600);
              }
              void loop() {
                  if(RC_avail()) print_RCpwm();
              }
               */

//    Step 4: Choose a failsafe position for each channel, in the range -1.0 to +1.0, and enter it into the array RC_failsafe[] = {}
//            Note: if you would like the arduino to respond to the loss of transmitter signal you may need to disable the failsafe feature on your receiver (if it has one).
//            an example sketch to check the operation of the failsafe, and for printing the calibrated channels to serial:
/* 
              unsigned long now;                        // timing variables to update data at a regular interval                  
              unsigned long rc_update;
              const int channels = 6;                   // specify the number of receiver channels
              float RC_in[channels];                    // an array to store the calibrated input from receiver 
              
              void setup()  {
                  setup_pwmRead();                      
                  Serial.begin(9600);
              }
              
              void loop()  {
                  now = millis();
                  
                  if(RC_avail() || now - rc_update > 25){   // if RC data is available or 25ms has passed since last update (adjust to suit frame rate of receiver)
                    
                    rc_update = now;                           
                    
                    //print_RCpwm();                        // uncommment to print raw data from receiver to serial
                    
                    for (int i = 0; i<channels; i++){       // run through each RC channel
                      int CH = i+1;
                      
                      RC_in[i] = RC_decode(CH);             // decode receiver channel and apply failsafe
                      
                      print_decimal2percentage(RC_in[i]);   // uncomment to print calibrated receiver input (+-100%) to serial       
                    }
                    Serial.println();                       // uncomment when printing calibrated receiver input to serial.
                  }
              }
               */

// EXAMPLE USE OF GENERIC PWM FUNCTIONS:
  /*
 // Print the pulse width of channel 1 to the serial monitor. 
 // This is equivelant to the using standard arduino pulseIn(pin, HIGH) function, but without blocking the code.
 
 if (PWM_read(1)){          // if a new pulse is detected on channel 1, print the pulse width to serial monitor.
   Serial.println(PWM());
 } 
  */
 // Or 
 /*
 // Print RC receiver frame length and frame rate
 
 if (PWM_read(1)){                                      // if a new pulse is detected on channel 1
   Serial.print(PWM_period(),0);Serial.print("uS ");     
   Serial.print(PWM_freq());Serial.println("Hz");
 }

 */

/*
 *  USER DEFINED VARIABLES (MODIFY TO SUIT YOUR APPLICATION)
 */
 
// PWM input pins, any of the following pins can be used: digital 0 - 13 or analog A0 - A5 
const unsigned char pwmPIN[]={pwmInThrottlePin,pwmInSteeringPin, pwmInAux1Pin, pwmInAux2Pin, USEchoPin, RPMSignalPin}; // an array to identify the PWM input pins (the array can be any length) 
                                                        // first pin is channel 1, second is channel 2...etc
#define THROTTLE_PWM_CHANNEL  1
#define STEERING_PWM_CHANNEL  2
#define AUX1_PWM_CHANNEL      3
#define AUX2_PWM_CHANNEL      4
#define US_PWM_CHANNEL        5
#define RPM_SENSOR_CHANNEL    6

unsigned char RC_inputs = 2;                // The number of pins in pwmPIN that are connected to an RC receiver. Addition pins not connected to an RC receiver could be used for any other purpose i.e. detecting the echo pulse on an HC-SR04 ultrasonic distance sensor
                                  // When 0, it will automatically update to the number of pins specified in pwmPIN[] after calling setup_pwmRead().                                                

/*
 *    GLOBAL PWM DECODE VARIABLES
 */

//const unsigned char num_ch = sizeof(pwmPIN)/sizeof(unsigned char);  // calculate the number of input pins (or channels)
#define NUM_CH (sizeof(pwmPIN)/sizeof(unsigned char))  // calculate the number of input pins (or channels)
volatile int PW[NUM_CH];                        // an array to store pulsewidth measurements
volatile boolean prev_pinState[NUM_CH];         // an array used to determine whether a pin has gone low-high or high-low
volatile unsigned long pciTime;                 // the time of the current pin change interrupt
volatile unsigned long pwmTimer[NUM_CH];        // an array to store the start time of each PWM pulse

volatile boolean pwmFlag[NUM_CH];               // flag whenever new data is available on each pin
volatile boolean RC_data_rdy;                   // flag when all RC receiver channels have received a new pulse
unsigned long pwmPeriod[NUM_CH];                 // period, mirco sec, between two pulses on each pin

byte pwmPIN_reg[NUM_CH];                        // each of the input pins expressed as a position on it's associated port register
byte pwmPIN_port[NUM_CH];                       // identify which port each input pin belongs to (0 = PORTB, 1 = PORTC, 2 = PORTD)


// FUNCTION USED TO TURN ON THE INTERRUPTS ON THE RELEVANT PINS
// code from http://playground.arduino.cc/Main/PinChangeInterrupt

void pciSetup(byte pin){
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin));                   // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin));                   // enable interrupt for the group
}

// FUNCTION USED TO FIND THE PIN POSITION ON EACH PORT REGISTER: helps the interrupt service routines, ISR, run faster

void pwmPIN_to_port(){
  for (int i = 0; i < NUM_CH; i++){

    // determine which port and therefore ISR (PCINT0_vect, PCINT1_vect or PCINT2_vect) each pwmPIN belongs to.
                                                                  pwmPIN_port[i] = 1;    // pin belongs to PCINT1_vect (PORT C)
    if (pwmPIN[i] >= 0 && pwmPIN[i] <= 7)                         pwmPIN_port[i] = 2;    // pin belongs to PCINT2_vect (PORT D)
    else if (pwmPIN[i] >= 8 && pwmPIN[i] <= 13)                   pwmPIN_port[i] = 0;    // pin belongs to PCINT0_vect (PORT B)

    // covert the pin number (i.e. pin 11 or pin A0) to the pin position in the port register. There is most likely a better way of doing this using a macro...
    // (Reading the pin state directly from the port registers speeds up the code in the ISR)
    
    if(pwmPIN[i] == 0 || pwmPIN[i] == A0 || pwmPIN[i] == 8)         pwmPIN_reg[i] = 0b00000001;
    else if(pwmPIN[i] == 1 || pwmPIN[i] == A1 || pwmPIN[i] == 9)    pwmPIN_reg[i] = 0b00000010;
    else if(pwmPIN[i] == 2 || pwmPIN[i] == A2 || pwmPIN[i] == 10)   pwmPIN_reg[i] = 0b00000100;
    else if(pwmPIN[i] == 3 || pwmPIN[i] == A3 || pwmPIN[i] == 11)   pwmPIN_reg[i] = 0b00001000;
    else if(pwmPIN[i] == 4 || pwmPIN[i] == A4 || pwmPIN[i] == 12)   pwmPIN_reg[i] = 0b00010000;
    else if(pwmPIN[i] == 5 || pwmPIN[i] == A5 || pwmPIN[i] == 13)   pwmPIN_reg[i] = 0b00100000;
    else if(pwmPIN[i] == 6)                                         pwmPIN_reg[i] = 0b01000000;
    else if(pwmPIN[i] == 7)                                         pwmPIN_reg[i] = 0b10000000;
    
  }
}

// SETUP OF PIN CHANGE INTERRUPTS

void setup_pwmRead(){
  
  for(int i = 0; i < NUM_CH; i++){              // run through each input pin
    pciSetup(pwmPIN[i]);                        // enable pinchange interrupt for pin
  }
  pwmPIN_to_port();                             // determines the port for each input pin
                                                // pwmPIN_to_port() also coverts the pin number in pwmPIN[] (i.e. pin 11 or pin A0) to the pin position in the port register (i.e. 0b00000001) for use in the ISR.
  
  if(RC_inputs == 0 || RC_inputs > NUM_CH) RC_inputs = NUM_CH;    // define the number of pins connected to an RC receiver.                                          
} 

// INTERRUPT SERVICE ROUTINES (ISR) USED TO READ PWM INPUT

// the PCINT0_vect (B port register) reacts to any changes on pins D8-13.
// the PCINT1_vect (C port register)          ""        ""         A0-A5.
// the PCINT2_vect (D port register)          ""        ""         D0-7.

// port registers are used to speed up if statements in ISR code:
// https://www.arduino.cc/en/Reference/PortManipulation http://tronixstuff.com/2011/10/22/tutorial-arduino-port-manipulation/
// http://harperjiangnew.blogspot.co.uk/2013/05/arduino-port-manipulation-on-mega-2560.html


// READ INTERRUPTS ON PINS D8-D13: ISR routine detects which pin has changed, and returns PWM pulse width, and pulse repetition period.

ISR(PCINT0_vect){                                                 // this function will run if a pin change is detected on portB

  #ifdef USE_TIMER2_COUNTER
  pciTime = int (timer2.get_count()/2);                                             // Record the time of the PIN change in microseconds
  #else
  pciTime = micros();                                             // Record the time of the PIN change in microseconds
  #endif
  
  for (int i = 0; i < NUM_CH; i++){                               // run through each of the channels
    if (pwmPIN_port[i] == 0){                                     // if the current channel belongs to portB
      
      if(prev_pinState[i] == 0 && PINB & pwmPIN_reg[i]){          // and the pin state has changed from LOW to HIGH (start of pulse)
        prev_pinState[i] = 1;                                     // record pin state
        pwmPeriod[i] = pciTime - pwmTimer[i];                     // calculate the time period, micro sec, between the current and previous pulse
        pwmTimer[i] = pciTime;                                    // record the start time of the current pulse
      }
      else if (prev_pinState[i] == 1 && !(PINB & pwmPIN_reg[i])){ // or the pin state has changed from HIGH to LOW (end of pulse)
        prev_pinState[i] = 0;                                     // record pin state
        PW[i] = pciTime - pwmTimer[i];                            // calculate the duration of the current pulse
        pwmFlag[i] = HIGH;                                        // flag that new data is available
        if(i+1 == RC_inputs) RC_data_rdy = HIGH;                  
      }
    }
  }
}

// READ INTERRUPTS ON PINS A0-A5: ISR routine detects which pin has changed, and returns PWM pulse width, and pulse repetition period.
/*
ISR(PCINT1_vect){                                                 // this function will run if a pin change is detected on portC

  pciTime = micros();                                             // Record the time of the PIN change in microseconds

  for (int i = 0; i < num_ch; i++){                               // run through each of the channels
    if (pwmPIN_port[i] == 1){                                     // if the current channel belongs to portC
      
      if(prev_pinState[i] == 0 && PINC & pwmPIN_reg[i]){          // and the pin state has changed from LOW to HIGH (start of pulse)
        prev_pinState[i] = 1;                                     // record pin state
        pwmPeriod[i] = pciTime - pwmTimer[i];                     // calculate the time period, micro sec, between the current and previous pulse
        pwmTimer[i] = pciTime;                                    // record the start time of the current pulse
      }
      else if (prev_pinState[i] == 1 && !(PINC & pwmPIN_reg[i])){ // or the pin state has changed from HIGH to LOW (end of pulse)
        prev_pinState[i] = 0;                                     // record pin state
        PW[i] = pciTime - pwmTimer[i];                             // calculate the duration of the current pulse
        pwmFlag[i] = HIGH;                                         // flag that new data is available
        if(i+1 == RC_inputs) RC_data_rdy = HIGH;
      }
    }
  }
}
*/
// READ INTERRUPTS ON PINS D0-7: ISR routine detects which pin has changed, and returns PWM pulse width, and pulse repetition period.

ISR(PCINT2_vect){                                                 // this function will run if a pin change is detected on portD

  #ifdef USE_TIMER2_COUNTER
  pciTime = int (timer2.get_count()/2);                                             // Record the time of the PIN change in microseconds
  #else
  pciTime = micros();                                             // Record the time of the PIN change in microseconds
  #endif

  for (int i = 0; i < NUM_CH; i++){                               // run through each of the channels
    if (pwmPIN_port[i] == 2){                                     // if the current channel belongs to portD
      
      if(prev_pinState[i] == 0 && PIND & pwmPIN_reg[i]){          // and the pin state has changed from LOW to HIGH (start of pulse)
        prev_pinState[i] = 1;                                     // record pin state
        pwmPeriod[i] = pciTime - pwmTimer[i];                     // calculate the time period, micro sec, between the current and previous pulse
        pwmTimer[i] = pciTime;                                    // record the start time of the current pulse
      }
      else if (prev_pinState[i] == 1 && !(PIND & pwmPIN_reg[i])){ // or the pin state has changed from HIGH to LOW (end of pulse)
        prev_pinState[i] = 0;                                     // record pin state
        PW[i] = pciTime - pwmTimer[i];                            // calculate the duration of the current pulse
        pwmFlag[i] = HIGH;                                        // flag that new data is available
        if(i+1 == RC_inputs) RC_data_rdy = HIGH;
      }
    }
  }
}


/*
 *  RC OUTPUT FUNCTIONS
 */

 boolean RC_avail(){
    boolean avail = RC_data_rdy;
    RC_data_rdy = LOW;                          // reset the flag
    return avail;
    }

/*
 *  Receiver Calibration
 */

// Basic Receiver FAIL SAFE
// check for 500-2500us and 10-330Hz (same limits as pololu)

boolean FAILSAFE(int CH){

   int i = CH-1;
   boolean failsafe_flag = LOW;
        
       if(pwmFlag[i] == 1)                             // if a new pulse has been measured.
         {
            pwmFlag[i] = 0;                            // set flag to zero
      
            if(pwmPeriod[i] > 100000)                  // if time between pulses indicates a pulse rate of less than 10Hz   
            {
              failsafe_flag = HIGH;                       
            }
            else if(pwmPeriod[i] < 3000)               // or if time between pulses indicates a pulse rate greater than 330Hz   
            {
              failsafe_flag = HIGH;                             
            }

            if(PW[i] < 500 || PW[i] > 2500)           // if pulswidth is outside of the range 500-2500ms
            {
              failsafe_flag = HIGH;                        
            }   
         }
        else if (micros() - pwmTimer[i] > 100000)     // if there is no new pulswidth measurement within 100ms (10hz)
        {
          failsafe_flag = HIGH;                      
        }

    return failsafe_flag;   
}

/*
 * GENERIC PWM FUNCTIONS
 */

unsigned long pin_time;
int pin_pwm;
unsigned long pin_period;

boolean PWM_read(int CH){
  if(CH < 1 && CH > NUM_CH) return false;
  int i = CH-1;
  boolean avail = pwmFlag[i];
  if (avail == HIGH){
    pwmFlag[i] = LOW;
    noInterrupts();
    pin_time = pwmTimer[i];
    pin_pwm = PW[i];
    pin_period = pwmPeriod[i];
    interrupts();
  }
  return avail;
}

unsigned long PWM_time(){return pin_time;}
unsigned long PWM_period(){return pin_period;}
int PWM(){return pin_pwm;}


void setup_pwm_sampler()  {
   if (ignore_rc_state==false) {
      led_controler_set_alarm(LED_CTRL_ALARM_RX_LOSS);
   }
   for(int i = 0; i < NUM_CH; i++){              // run through each input pin
      pinMode (pwmPIN[i], INPUT_PULLUP);
   }

   setup_pwmRead();
}

void print_RCpwm(){                             // display the raw RC Channel PWM Inputs
  for (int i = 0; i < RC_inputs; i++){
    //Serial.print(" ch");Serial.print(i+1);
    Serial.print("  ");
    if(PW[i] < 1000) Serial.print(" ");
    Serial.print(PW[i]);
  }
  Serial.println("");
}

unsigned int _rx_received = 0;
void pwm_sampler_update (int *throttle, int *steering, int * aux1, int * aux2) {
  if (RC_avail()) {
    //print_RCpwm();
    if (PWM_read(THROTTLE_PWM_CHANNEL)) {          // if a new pulse is detected on channel 1, print the pulse width to serial monitor.
      _rx_received++;
      *throttle = PWM();
    } 
    if (PWM_read(STEERING_PWM_CHANNEL)) {          // if a new pulse is detected on channel 1, print the pulse width to serial monitor.
      *steering = PWM();
    }      
    if (PWM_read(AUX1_PWM_CHANNEL)) {          // if a new pulse is detected on channel 1, print the pulse width to serial monitor.
      *aux1 = PWM();
    }     
    if (PWM_read(AUX2_PWM_CHANNEL)) {          // if a new pulse is detected on channel 1, print the pulse width to serial monitor.
      *aux2 = PWM();
    }     
  }
}

void pwm_sampler_check_failsafe () {
  if (ignore_rc_state==false) {
    if (pwm_calibrated==1) {
      if (rx_loss==0) {
        if (_rx_received ==0) {
          led_controler_set_alarm(LED_CTRL_ALARM_RX_LOSS);
          pwm_calibrated=0;
          PWM_in_throttle_idle = 0;
          PWM_in_throttle_idle = 0;
          rx_loss = 1;      
        }
      } else {
        if (_rx_received > 0) {
          led_controler_reset_alarm(LED_CTRL_ALARM_RX_LOSS);    
          rx_loss=0; 
        }
      }
      _rx_received = 0;
    } else {
      if (_rx_received>30) {
        if ((PWM_in_throttle_idle >0) && (PWM_in_throttle_idle >0)) {
          led_controler_reset_alarm(LED_CTRL_ALARM_RX_LOSS);    
          pwm_calibrated=1;
        } else {
          _rx_received = 0;
  
        }
      }
    }
  }
}
