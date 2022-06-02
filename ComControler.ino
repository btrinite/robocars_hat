
unsigned char _com_controler_status = 0;

bool com_controler_check_status() {
  return (_com_controler_status == 1);
}


#ifdef Use_ROSSerial

ros::NodeHandle  nh;

void throttle_cb( const std_msgs::Int16& throttle_msg) {
  if (!passthrough) {
    pwmdriver_set_throttle_output (throttle_msg.data);  
  }
}

void steering_cb( const std_msgs::Int16& steering_msg) {
  if (!passthrough) {
    pwmdriver_set_steering_output (steering_msg.data);
  }
}

ros::Subscriber<std_msgs::Int16> throttle_sub("throttle_ctrl/output", throttle_cb);
ros::Subscriber<std_msgs::Int16> steering_sub("steering_ctrl/output", steering_cb);

std_msgs::Int16MultiArray channels_msg;
std_msgs::Int16MultiArray lipo_msg;
std_msgs::Int16MultiArray sensors_msg;
ros::Publisher pub_channels( "radio_channels", &channels_msg);
ros::Publisher pub_lipo( "battery", &lipo_msg);
ros::Publisher pub_sensors( "sensors", &sensors_msg);

void publish_battery_state (int vbatmv, int cell1mv, int cell2mv, int cell3mv) {
  lipo_msg.data[0] = vbatmv;
  lipo_msg.data[1] = cell1mv;
  lipo_msg.data[2] = cell2mv;
  lipo_msg.data[3] = cell3mv;
  pub_lipo.publish(&lipo_msg);
}

void publish_channels_state (int throttle, int steering, int aux1, int aux2) {
  channels_msg.data[0] = throttle;
  channels_msg.data[1] = steering;
  channels_msg.data[2] = aux1;
  channels_msg.data[3] = aux2;
  pub_channels.publish(&channels_msg);
}


void publish_sensors_state (int distance, int rpm) {
  sensors_msg.data[0] = distance;
  sensors_msg.data[1] = rpm;
  pub_sensors.publish(&sensors_msg);
}

void com_controler_setup() {
  lipo_msg.data = (int16_t*) malloc(sizeof(int) * 3);
  lipo_msg.data_length = 4;

  channels_msg.data = (int16_t*) malloc(sizeof(int) * 2);
  channels_msg.data_length = 4;

  sensors_msg.data = (int16_t*) malloc(sizeof(int) * 1);
  sensors_msg.data_length = 2;

  nh.initNode();
  nh.advertise(pub_channels);
  nh.advertise(pub_lipo);
  nh.advertise(pub_sensors);
  nh.subscribe(steering_sub);
  nh.subscribe(throttle_sub);
}

void com_controler_update() {
  static unsigned char alarmRaised = 0;

  if (!nh.connected()) {
    _com_controler_status = 0;
    if (alarmRaised == 0) {
      led_controler_set_alarm(LED_CTRL_ALARM_LINK_LOSS);
      alarmRaised = 1;
    }
  } else {
    _com_controler_status = 1;
    if (alarmRaised == 1) {
      led_controler_reset_alarm(LED_CTRL_ALARM_LINK_LOSS);
      alarmRaised = 0;
    }
  }
  nh.spinOnce();
}

#endif

// SimpleSerial
// Communication based on simple stupid serial protocol
#ifdef Use_SimpleSerial

#define SERIAL_BAUD 921600

// Sent message types
#define TX_MSG_BAT_STATE     0
#define TX_MSG_RX_CHANNELS   1
#define TX_MSG_SENSORS_STATE 2

//received message type
#define RX_MSG_DRIVE_CHANNELS   1


void publish_buff(char * buff)
{
  int len = Serial.print(buff);
  if (len != strlen(buff)) {
    if (_com_controler_status == 1) {
      led_controler_set_alarm(LED_CTRL_ALARM_LINK_LOSS);
      _com_controler_status = 0;
    } else {
      led_controler_reset_alarm(LED_CTRL_ALARM_LINK_LOSS);
      _com_controler_status = 1;
    }
  }
}

char buff[64];
void publish_battery_state (int vbatmv, int cell1mv, int cell2mv, int cell3mv) {

  int fit = snprintf (buff, sizeof(buff), "%d,%d,%d,%d,%d\r\n", TX_MSG_BAT_STATE, vbatmv, cell1mv, cell2mv, cell3mv);
  if (fit > 0 and fit < sizeof(buff)) {
    publish_buff(buff);
  }
}

void publish_channels_state (int throttle, int steering, int aux1, int aux2) {
  int fit = snprintf (buff, sizeof(buff), "%d,%d,%d,%d,%d \r\n", TX_MSG_RX_CHANNELS, throttle, steering, aux1, aux2);
  if (fit > 0 and fit < sizeof(buff)) {
    publish_buff(buff);
  }
}

void publish_sensors_state (int distance, int rpm) {
  int fit = snprintf (buff, sizeof(buff), "%d,%d,%d\r\n", TX_MSG_SENSORS_STATE, distance, rpm);
  if (fit > 0 and fit < sizeof(buff)) {
    publish_buff(buff);
  }
}

String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}


void com_controler_setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.setTimeout(5);
  //Assume everything is OK by default
  _com_controler_status = 1;
}

bool valid_pwm_value(int value) {
  if (value >=1000 and value <=2000) return true; else return false;
}

String inData;

void serialEvent() {
  unsigned char chRead = 0;
  char recieved = 0;
  while ((recieved = Serial.read())>0) {

    inData += recieved;
    chRead++;

    // Process message when new line character is recieved
    if (recieved == '\n')
    { 
      Serial.println("Got msg...");
      Serial.println(inData);
      unsigned char msgType = getValue(inData, ',', 0).toInt();
      switch (msgType) {
        case RX_MSG_DRIVE_CHANNELS:
          int throttle = getValue(inData, ',', 1).toInt();
          int steering = getValue(inData, ',', 2).toInt();
          if (valid_pwm_value(throttle) and valid_pwm_value(steering) and !passthrough) {
            pwmdriver_set_throttle_output (throttle);
            pwmdriver_set_steering_output (steering);
          }
          break;
      }
      inData = ""; // Clear recieved buffer
      chRead = 0;
    }
  }
}

void com_controler_update() {
}
#endif
