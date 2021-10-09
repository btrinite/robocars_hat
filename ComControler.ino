
unsigned char _com_controler_status=0;

bool com_controler_check_status() {
  return (_com_controler_status==1);
}


#ifdef Use_ROSSerial

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
ros::Publisher pub_lipo( "battery", &lipo_msg);
ros::Publisher pub_sensors( "sensors", &sensors_msg);

void publish_battery_state (int vbatmv, int cell1mv, int cell2mv, int cell3mv) {
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
      led_controler_set_alarm(LED_CTRL_ALARM_LINK_LOSS);    
      alarmRaised=1;
    }
  } else {
    _com_controler_status = 1;
    if (alarmRaised==1) {
      led_controler_reset_alarm(LED_CTRL_ALARM_LINK_LOSS);
      alarmRaised=0;
    }
  }
  nh.spinOnce();
}

#endif

// SLIPSerial
// Communication based on SLIP protocol
#ifdef Use_SLIPSerial
//PacketSerial based implementation
//https://github.com/bakercp/PacketSerial

#define SLIP_BAUD 1000000

#include <PacketSerial.h>
SLIPPacketSerial myPacketSerial;

//Little/Big Endian macro
#define htons(x) ( ((x)<< 8 & 0xFF00) | \
                   ((x)>> 8 & 0x00FF) )
#define ntohs(x) htons(x)

#define htonl(x) ( ((x)<<24 & 0xFF000000UL) | \
                   ((x)<< 8 & 0x00FF0000UL) | \
                   ((x)>> 8 & 0x0000FF00UL) | \
                   ((x)>>24 & 0x000000FFUL) )
#define ntohl(x) htonl(x)

// Sent message types
#define TX_MSG_BAT_STATE     0
#define TX_MSG_RX_CHANNELS   1
#define TX_MSG_SENSORS_STATE 2

//received message type
#define RX_MSG_DRIVE_CHANNELS   0

// static buffer to old messages sent
struct s_battery_msg{
  unsigned char msg = TX_MSG_BAT_STATE;
  unsigned int vbatmv;
  unsigned int cell1mv;
  unsigned int cell2mv;
  unsigned int cell3mv;
} battery_msg;

struct s_rx_channels_msg{
  unsigned char msg = TX_MSG_RX_CHANNELS;
  int throttle;
  int steering;
} rx_channels_msg;

struct s_rx_sensors_msg{
  unsigned char msg = TX_MSG_SENSORS_STATE;
  int distance;
  int rpm;
} sensors_msg;

// struture to map received messages
typedef struct s_tx_channels_msg{
  unsigned char msg;
  int throttle;
  int steering;
} t_tx_channels_msg;


void publish_battery_state (int vbatmv, int cell1mv, int cell2mv, int cell3mv) {
  battery_msg.vbatmv  = htonl(vbatmv);
  battery_msg.cell1mv = htonl(cell1mv);
  battery_msg.cell2mv = htonl(cell2mv);
  battery_msg.cell3mv = htonl(cell3mv);
  
  myPacketSerial.send((uint8_t*)&battery_msg, sizeof(battery_msg));
}

void publish_channels_state (int throttle, int steering) {
  rx_channels_msg.throttle = htonl(throttle);
  rx_channels_msg.steering = htonl(steering);
  myPacketSerial.send((uint8_t*)&rx_channels_msg, sizeof(rx_channels_msg));
}

void publish_sensors_state (int distance, int rpm) {
  sensors_msg.distance = htonl(distance);
  sensors_msg.rpm = htonl(rpm);
  myPacketSerial.send((uint8_t*)&sensors_msg, sizeof(sensors_msg));
}


void com_controler_setup() {
    myPacketSerial.begin(SLIP_BAUD);
    myPacketSerial.setPacketHandler(&onPacketReceived); 
    //Assume everything is OK by default
    _com_controler_status = 1;
}

void onPacketReceived(const void* sender, const uint8_t* buffer, size_t size)
{
  if (sender == &myPacketSerial)
  {
    if (_com_controler_status == 0) {
      led_controler_reset_alarm(LED_CTRL_ALARM_LINK_LOSS);    
      _com_controler_status=1;      
    }
    // In this example, we will simply reverse the contents of the array and send
    // it back to the sender.
    // Make a temporary buffer.
    uint8_t tempBuffer[size];

    // Copy the packet into our temporary buffer.
    memcpy(tempBuffer, buffer, size);

    switch (tempBuffer[0]) {
      case RX_MSG_DRIVE_CHANNELS:
        t_tx_channels_msg * pmsg=(t_tx_channels_msg *)&tempBuffer[0];
        pwmdriver_set_throttle_output (ntohl(pmsg->throttle));
        pwmdriver_set_steering_output (ntohl(pmsg->steering));
      break;
    }
  }
}

void com_controler_update() {
  myPacketSerial.update();

  // Check for a receive buffer overflow (optional).
  if (myPacketSerial.overflow())
  {
    if (_com_controler_status==1) {
      led_controler_set_alarm(LED_CTRL_ALARM_LINK_LOSS);    
      _com_controler_status=0;
    }
  }
}

#endif

// SimpleSerial
// Communication based on simple stupid serial protocol
#ifdef Use_SimpleSerial

#define SERIAL_BAUD 1000000

// Sent message types
#define TX_MSG_RX_CHANNELS   0
#define TX_MSG_BAT_STATE     1
#define TX_MSG_SENSORS_STATE 2

//received message type
#define RX_MSG_DRIVE_CHANNELS   0


void publish_buff(char * buff)
{
  int len = Serial.print(buff);
  if (len != strlen(buff)) {
    if (_com_controler_status==1) {
      led_controler_set_alarm(LED_CTRL_ALARM_LINK_LOSS);    
      _com_controler_status=0;
    } else {
      led_controler_reset_alarm(LED_CTRL_ALARM_LINK_LOSS);    
      _com_controler_status=1;        
    }
  }  
}

char buff[64];
void publish_battery_state (int vbatmv, int cell1mv, int cell2mv, int cell3mv) {

  int fit = snprintf (buff, sizeof(buff), "%d,%d,%d,%d,%d\r\n", TX_MSG_BAT_STATE, vbatmv, cell1mv, cell2mv, cell3mv);
  if (fit>0 and fit<sizeof(buff)) {
    publish_buff(buff);
  }
}

void publish_channels_state (int throttle, int steering) {
  int fit = snprintf (buff, sizeof(buff), "%d,%d,%d\r\n", TX_MSG_RX_CHANNELS, throttle, steering);
  if (fit>0 and fit<sizeof(buff)) {
    publish_buff(buff);
  }
}

void publish_sensors_state (int distance, int rpm) {
  int fit = snprintf (buff, sizeof(buff), "%d,%d,%d\r\n", TX_MSG_SENSORS_STATE, distance, rpm);
  if (fit>0 and fit<sizeof(buff)) {
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
            strIndex[1] = (i == maxIndex) ? i+1 : i;
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

String inData;

void com_controler_update() {
  while (Serial.available()) {
    char recieved = Serial.read();
    inData += recieved; 

    // Process message when new line character is recieved
    if (recieved == '\n')
    {
      //Serial.println("Got");
      //Serial.println(inData);
      //Serial.println(inData.length());
      unsigned char msgType = getValue(inData, ',', 0).toInt();
      switch (msgType) {
        case RX_MSG_DRIVE_CHANNELS:
          int throttle = getValue(inData, ',', 1).toInt();   
          int steering = getValue(inData, ',', 2).toInt(); 
          if ((throttle>0) and (steering>0)) {
            pwmdriver_set_throttle_output (throttle);        
            pwmdriver_set_steering_output (steering);                
            /*  
            if (_com_controler_status==0) {
              led_controler_reset_alarm(LED_CTRL_ALARM_LINK_LOSS);    
              _com_controler_status=1;        
            }*/
          } else {
            Serial.println("Link Loss2");
            Serial.println(inDatax);
            /*
            if (_com_controler_status==1) {
              Serial.println("Link Loss2");
              led_controler_set_alarm(LED_CTRL_ALARM_LINK_LOSS);    
              _com_controler_status=0;
            }*/
          }
        break;
      }
      inData = ""; // Clear recieved buffer
    }
  }
}
#endif
