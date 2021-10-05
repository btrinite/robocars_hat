
unsigned char _com_controler_status=0;

bool com_controler_check_status() {
  return (_com_controler_status==1);
}


#ifdef Use_ROS

ros::NodeHandle  nh;

void throttle_cb( const std_msgs::Int16& throttle_msg){
  pwmdriver_set_throttle_output (throttle_msg.data);
}

void steering_cb( const std_msgs::Int16& steering_msg){
  pwmdriver_set_steering_output (steering_msg.data);
}

ros::Subscriber<std_msgs::Int16> throttle_sub("throttle_ctrl/output", throttle_cb);
ros::Subscriber<std_msgs::Int16> steering_sub("steering_ctrl/output", steering_cb);

std_msgs::Int16MultiArray channels_msg;  
std_msgs::Int16MultiArray lipo_msg;  
std_msgs::Int16MultiArray sensors_msg;  
ros::Publisher pub_channels( "radio_channels", &channels_msg);
ros::Publisher pub_lipo( "lipo", &lipo_msg);
ros::Publisher pub_sensors( "sensors", &sensors_msg);

void publish_lipo_state (int vbatmv, int cell1mv, int cell2mv, int cell3mv) {
  lipo_msg.data[0]=vbatmv;  
  lipo_msg.data[1]=cell1mv;
  lipo_msg.data[2]=cell2mv;
  lipo_msg.data[3]=cell3mv;  
  pub_lipo.publish(&lipo_msg);
}

void publish_channels_state (int throttle, int steering) {
  channels_msg.data[0]=throttle;
  channels_msg.data[1]=steering;
  pub_channels.publish(&channels_msg);
}


void publish_sensors_state (int distance, int rpm) {
  sensors_msg.data[0]=distance;
  sensors_msg.data[1]=rpm;
  pub_sensors.publish(&sensors_msg);
}

void com_controler_setup() {
  lipo_msg.data = (int16_t*) malloc(sizeof(int) * 3);
  lipo_msg.data_length=4;

  channels_msg.data = (int16_t*) malloc(sizeof(int) * 2);
  channels_msg.data_length=2;

  sensors_msg.data = (int16_t*) malloc(sizeof(int) * 1);
  sensors_msg.data_length=2;

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
    if (alarmRaised==0) {
      led_controler_set_alarm(LED_CTRL_ALARM_ROS_LOSS);    
      alarmRaised=1;
    }
  } else {
    _com_controler_status = 1;
    if (alarmRaised==1) {
      led_controler_reset_alarm(LED_CTRL_ALARM_ROS_LOSS);
      alarmRaised=0;
    }
  }
  nh.spinOnce();
}

#else

void publish_lipo_state (int cell1mv, int cell2mv, int cell3mv) {
}

void publish_channels_state (int throttle, int steering) {
}

void publish_sensors_state (int distance) {
}

void com_controler_setup() {
}

void com_controler_update() {
}


#endif
