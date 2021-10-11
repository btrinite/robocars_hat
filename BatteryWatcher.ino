
int cell1Raw = 0;  // variable to store the value coming from the sensor
int cell2Raw = 0;  // variable to store the value coming from the sensor
int vbatRaw = 0;  // variable to store the value coming from the sensor
int cell1mv = 0;  // variable to store the value coming from the sensor
int cell2mv = 0;  // variable to store the value coming from the sensor
int vbatmv = 0;  // variable to store the value coming from the sensor
const int divider=11;
const long unituv=5000000/1024;

void LIPO_watcher_setup() {
}

#define LIPO_CELL_LOW_VOLTAGE 3200
#define VBAT_LOW_VOLTAGE 5500

unsigned char battery_alarm_records=0;
unsigned char idx_alarm_record=0;

void record_battery_alarm (unsigned char alarm) {
  if (alarm==1) {
    battery_alarm_records |= (1<<idx_alarm_record);
  } else {
    battery_alarm_records &= ~(1<<idx_alarm_record);
  }
  idx_alarm_record=(idx_alarm_record+1)%8;
}

void evaluate_low_battery() {
  if (battery_alarm_records == 0xff) {
    battery_low=1;
  }
}

void battery_watcher_update() {
  // read the value from the sensor:
  cell1Raw = analogRead(batteryCell1Pin);
  cell2Raw = analogRead(batteryCell2Pin);
  vbatRaw = analogRead(batteryVBatPin);
  cell1mv = (cell1Raw*unituv*divider)/1000;
  cell2mv = (cell2Raw*unituv*divider)/1000;
  vbatmv = (vbatRaw*unituv*divider)/1000;
  if (battery_low==0) {

    //Global VBat
    if (vbatmv<VBAT_LOW_VOLTAGE) {
      record_battery_alarm(1);
    }

    //Is there a Lipo load balance connected
    if (cell1mv>100) {
      // Cell1
      if (cell1mv<LIPO_CELL_LOW_VOLTAGE) {
        record_battery_alarm(1);       
      }            

      if (cell2mv < 100) {
        //2S, 2nd cell
        if ((vbatmv-cell1mv)<LIPO_CELL_LOW_VOLTAGE) {
          record_battery_alarm(1);       
        }
      }
      else {
        //3S, 2nd cell
        if ((cell2mv-cell1mv)<LIPO_CELL_LOW_VOLTAGE) {
          record_battery_alarm(1);       
        }
        //3rd cell
        if ((vbatmv-cell2mv)<LIPO_CELL_LOW_VOLTAGE) {
          record_battery_alarm(1);       
        }
      }
    }

    evaluate_low_battery();
    if (battery_low==1) {
      led_controler_set_alarm(LED_CTRL_ALARM_BATTERY_LOW);
    }
  }
  
  //Publish battery state
  if (cell1mv < 100) {
    //Simple battery/power supply, no cells to monitor
    publish_battery_state (vbatmv,0,0,0);      
  }else if (cell2mv < 100) {
    // most likely a 2S
    publish_battery_state (vbatmv, cell1mv, vbatmv-cell1mv,0);
  } else {
    // most likely a 3S
    publish_battery_state (vbatmv, cell1mv, cell2mv-cell1mv,vbatmv-cell2mv);
  }
}
