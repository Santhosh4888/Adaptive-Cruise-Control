#include <Arduino_CAN.h>
#include <JrkG2.h>
#include <ros.h>
#include <std_msgs/Float32.h>

#define CAN_BITRATE CanBitRate::BR_500k
#define CAN_ID 0x626

#ifdef SERIAL_PORT_HARDWARE_OPEN
  #define jrkSerial SERIAL_PORT_HARDWARE_OPEN
#else
  #include <SoftwareSerial.h>
  SoftwareSerial jrkSerial(10, 11);
#endif

JrkG2Serial jrk(jrkSerial);

// Object dictionary indices and scaling factors
#define OD_MaxSpeed     0x3840
#define OD_MaxSpCont    0x3559
#define OD_AccelRate    0x3843
#define OD_DecelRate    0x3843
#define OD_BrakeRate    0x3848
#define OD_DriveCurrL   0x305B
#define OD_RegenCurrL   0x305C
#define OD_ForwardDB    0x3001
#define OD_ForwardMap   0x3002
#define OD_ForwardMax   0x3003
#define OD_ForwardOff   0x3004
#define OD_SpeedToRPM   0x304C

#define OD_MotorRPM     0x3207
#define OD_MasterTime   0x3160
#define OD_ThrottlePot  0x3215
#define OD_ThrottleCmd  0x3216
#define OD_CurrentRMS   0x3209
#define OD_RegenState   0x322B
#define OD_VehSpeed     0x320A
#define OD_VehDist      0x3616
#define OD_VehAccel     0x35C1

uint16_t const read_indices[] = {OD_MaxSpeed, OD_MaxSpCont, OD_AccelRate, OD_DecelRate, OD_BrakeRate, OD_DriveCurrL, OD_RegenCurrL, OD_ForwardDB, OD_ForwardMap, OD_ForwardMax, OD_ForwardOff, OD_SpeedToRPM};
float const readindices_scaling[] = {1, 1, 30.0/3000, 30.0/30000, 30.0/30000, 100.0/32767, 100.0/32767, 5.0/32767, 100.0/32767, 5.0/32767, 100.0/32767, 0.1};
int const readindices_length = sizeof(read_indices)/sizeof(read_indices[0]);
float readindices_values[readindices_length] = {};

uint16_t const monitor_indices[] = {OD_MotorRPM, OD_ThrottlePot, OD_ThrottleCmd, OD_CurrentRMS, OD_RegenState, OD_VehSpeed, OD_VehDist, OD_VehAccel, OD_MasterTime};
float const monitorindices_scaling[] = {1, 5.5/36044, 100.0/32767, 0.1, 1, 0.1, 0.1, 10.0/10000, 0.1};
int const monitorindices_length = sizeof(monitor_indices)/sizeof(monitor_indices[0]);
float monitorindices_values[monitorindices_length] = {};

float desired_voltage = 0;
bool targetReached = false;
float max_rpm = 0;
float deadband_voltage = 0;
float map_setting = 0;
float voltage_max = 0;
float offset = 0;

bool reverseTest = false;
float reverse_voltage = 1;

unsigned long start_time = 0xFFFFFFFF;
unsigned long test_time = 30000;

ros::NodeHandle nh;
std_msgs::Float32 vel_msg;

// Publisher Node Initialization
ros::Publisher velocity_pub("/velocity_feedback", &vel_msg, 1);

void motorCommandCallback(const std_msgs::Float32 &msg) {
  desired_voltage = msg.data;
  Serial.print("Received motor command from Python: ");
  Serial.println(desired_voltage);
}

// Subsciber Node Initialization
ros::Subscriber<std_msgs::Float32> motor_sub("/motor_command", &motorCommandCallback, 1);

void clear_receive_buffer() {
  while (CAN.available()) {
    CAN.read();
  }
}

int sdo_download(uint16_t odvalue, uint16_t datavalue) {
  uint8_t firstbyte = odvalue % 256;
  uint8_t secondbyte = odvalue / 256;
  uint8_t databyte1 = datavalue % 256;
  uint8_t databyte2 = datavalue / 256;
  uint8_t const msg_data[] = {0x20, firstbyte, secondbyte, 0x00, databyte1, databyte2, 0, 0};
  CanMsg const msg(CanStandardId(CAN_ID), sizeof(msg_data), msg_data);
  int const rc = CAN.write(msg);

  if(rc < 0) {
    return rc;
  }

  unsigned long timeout = millis() + 100;
  while (millis() < timeout) {
    if(CAN.available()) {
      CanMsg const ackmsg = CAN.read();
      uint8_t readbyte1 = ackmsg.data[4];
      uint8_t readbyte2 = ackmsg.data[5];
      short readvalue = ((readbyte2 << 8) | readbyte1);
      uint8_t indexbyte1 = ackmsg.data[1];
      uint8_t indexbyte2 = ackmsg.data[2];
      uint16_t odindex = ((indexbyte2 << 8) | indexbyte1);
      if(odindex == odvalue && readvalue == 0) {
        return 1;
      }
    }
    nh.spinOnce();
    delay(1);
  }
  return 0;
}

int send_sdo_upload(uint16_t odvalue) {
  uint8_t firstbyte = odvalue % 256;
  uint8_t secondbyte = odvalue / 256;
  uint8_t const msg_data[] = {0x40, firstbyte, secondbyte, 0x00, 0, 0, 0, 0};
  CanMsg const msg(CanStandardId(CAN_ID), sizeof(msg_data), msg_data);
  return CAN.write(msg);
}

void receive_sdo_upload() {
  unsigned long timeout = millis() + 50;
  while (!CAN.available() && millis() < timeout) {
    nh.spinOnce();
    delay(1);
  }
  
  if (CAN.available()) {
    CanMsg const msg = CAN.read();
    if(msg.id == 0x726) return;
    
    uint8_t databytes[] = {msg.data[4], msg.data[5], msg.data[6], msg.data[7]};
    int datavalue = databytes[3] << 24 | databytes[2] << 16 | databytes[1] << 8 | databytes[0];
    uint8_t indexbyte1 = msg.data[1];
    uint8_t indexbyte2 = msg.data[2];
    uint16_t odindex = ((indexbyte2 << 8) | indexbyte1);
    
    for (int i = 0; i < readindices_length; i++) {
      if (odindex == read_indices[i]) {
        readindices_values[i] = readindices_scaling[i] * datavalue;
      }
    }
  }
}

void receive_sdo_upload_monitor() {
  unsigned long timeout = millis() + 50;
  while (!CAN.available() && millis() < timeout) {
    nh.spinOnce();
    delay(1);
  }
  
  if (CAN.available()) {
    CanMsg const msg = CAN.read();
    if(msg.id == 0x726) return;
    
    uint8_t databytes[] = {msg.data[4], msg.data[5], msg.data[6], msg.data[7]};
    int datavalue = databytes[3] << 24 | databytes[2] << 16 | databytes[1] << 8 | databytes[0];
    uint8_t indexbyte1 = msg.data[1];
    uint8_t indexbyte2 = msg.data[2];
    uint16_t odindex = ((indexbyte2 << 8) | indexbyte1);
    for (int i = 0; i < monitorindices_length; i++) {
      if (odindex == monitor_indices[i]) {
        monitorindices_values[i] = monitorindices_scaling[i] * datavalue;
      }
    }
  }
}

void read_params() {
  clear_receive_buffer();
  for (int i = 0; i < readindices_length; i++) {
    send_sdo_upload(read_indices[i]);
    delay(5);
    
    unsigned long timeout = millis() + 100;
    while (!CAN.available() && millis() < timeout) {
      nh.spinOnce();
      delay(1);
    }
    
    while (CAN.available()) {
      receive_sdo_upload();
    }
    delay(5);
  }
  
  max_rpm = readindices_values[0];
  deadband_voltage = readindices_values[7];
  map_setting = readindices_values[8];
  voltage_max = readindices_values[9];
  offset = readindices_values[10];
  
  if(max_rpm == -1 || deadband_voltage == -1 || map_setting == -1 || voltage_max == -1 || offset == -1) {
    while (true) {
      nh.spinOnce();
      delay(10);
    }
  }
}

void setup() { 
  
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(A0, OUTPUT);
  Serial.begin(115200);
  
  
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  delay(1000);
  nh.subscribe(motor_sub);
  nh.advertise(velocity_pub);

  delay(1000);
  //jrkSerial.begin(9600);

    // Wait for ROS connection
  while (!nh.connected()) {
    nh.spinOnce();
    Serial.println("Waiting for ROS connection...");
    delay(100);
  }

  Serial.println("ROS connected!");

  if (!CAN.begin(CAN_BITRATE)) {
    while (true) {
      nh.spinOnce();
      delay(10);
    }
  }
  
  if (sdo_download(OD_AccelRate, 1000) < 0) {     
    while(true) {
      nh.spinOnce();
      delay(10);
    }
  }
  delay(10);
  
  if (sdo_download(OD_DecelRate, 100) < 0) {     
    while(true) {
      nh.spinOnce();
      delay(10);
    }
  }
  delay(10);
  
  if (sdo_download(OD_BrakeRate, 100) < 0) {     
    while(true) {
      nh.spinOnce();
      delay(10);
    }
  }
  
  read_params();
  start_time = millis();
}

void loop() {
  static uint8_t monitor_index = 0;
  
  send_sdo_upload(monitor_indices[monitor_index]);
  monitor_index = (monitor_index + 1) % monitorindices_length;
  
  receive_sdo_upload_monitor();
  

  
  nh.spinOnce();
  
  if (!nh.connected()) {
    // Optional: indicate disconnected state with an LED
    return;  // Skip publishing until connection is ready
  }
  
  vel_msg.data = monitorindices_values[5];
  velocity_pub.publish(&vel_msg);
  
  if (millis() - start_time <= test_time) {
    desired_voltage = constrain(desired_voltage, 0.5, 4.5);
    int dc = (int)(255 * (desired_voltage - 0.5) / 4.0);
    analogWrite(A0, dc);
  }
  else {
    analogWrite(A0, 0);
    
    jrk.setTarget(690);
    delay(5000);
    jrk.setTarget(2600);
    
    while (true) {
      nh.spinOnce();
      delay(10);
    }
  }
  
  delay(10);
}
