#include <Arduino_CAN.h>
#include <JrkG2.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

#define CAN_BITRATE CanBitRate::BR_500k
#define CAN_ID 0x626

#ifdef SERIAL_PORT_HARDWARE_OPEN
  #define jrkSerial SERIAL_PORT_HARDWARE_OPEN
#else
  #include <SoftwareSerial.h>
  SoftwareSerial jrkSerial(10, 11); // RX, TX
#endif

JrkG2Serial jrk(jrkSerial);

// -------------------- Object Dictionary --------------------
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

// -------------------- Arrays --------------------
uint16_t const read_indices[] = {OD_MaxSpeed, OD_MaxSpCont, OD_AccelRate, OD_DecelRate, OD_BrakeRate, OD_DriveCurrL, OD_RegenCurrL, OD_ForwardDB, OD_ForwardMap, OD_ForwardMax, OD_ForwardOff, OD_SpeedToRPM};
float const readindices_scaling[] = {1, 1, 30.0/3000, 30.0/30000, 30.0/30000, 100.0/32767, 100.0/32767, 5.0/32767, 100.0/32767, 5.0/32767, 100.0/32767, 0.1};
int const readindices_length = sizeof(read_indices)/sizeof(read_indices[0]);
float readindices_values[readindices_length] = {};

const char* read_indices_names[] = {
  "OD_MaxSpeed",      // Index 0
  "OD_MaxSpCont",     // Index 1
  "OD_AccelRate",     // Index 2
  "OD_DecelRate",     // Index 3
  "OD_BrakeRate",     // Index 4
  "OD_DriveCurrL",    // Index 5
  "OD_RegenCurrL",    // Index 6
  "OD_ForwardDB",     // Index 7
  "OD_ForwardMap",    // Index 8
  "OD_ForwardMax",    // Index 9
  "OD_ForwardOff",    // Index 10
  "OD_SpeedToRPM"     // Index 11
};

uint16_t const monitor_indices[] = {OD_MotorRPM, OD_ThrottlePot, OD_ThrottleCmd, OD_CurrentRMS, OD_RegenState, OD_VehSpeed, OD_VehDist, OD_VehAccel, OD_MasterTime};
float const monitorindices_scaling[] = {1, 5.5/36044, 100.0/32767, 0.1, 1, 0.1, 0.1, 10.0/10000, 0.1};
int const monitorindices_length = sizeof(monitor_indices)/sizeof(monitor_indices[0]);
float monitorindices_values[monitorindices_length] = {};

// -------------------- Global Variables --------------------
float desired_voltage = 0;
float max_rpm = 0;
float deadband_voltage = 0;
float map_setting = 0;
float voltage_max = 0;
float offset = 0;

unsigned long start_time = 0xFFFFFFFF;
unsigned long test_time = 50000;

const float Brake_Voltage_Threshold = 0.5;
const uint16_t Jrk_Brake_position = 690;
const uint16_t Jrk_Neutral_position = 2048;
const uint16_t Jrk_Normal_position = 2600; 

// -------------------- ROS Setup --------------------
ros::NodeHandle nh;

// Subscriber Initialization
void motorCommandCallback(const std_msgs::Float32 &msg) {
  desired_voltage = msg.data;
  char control_msg[40];
  snprintf(control_msg, sizeof(control_msg),"Received motor command:%.3f V", desired_voltage);
  nh.loginfo(control_msg);
}

ros::Subscriber<std_msgs::Float32>sub("motor_command", &motorCommandCallback);

// Publisher Initialization
std_msgs::Float32 vel_msg;
ros::Publisher chatter("velocity_feedback", &vel_msg);


// -------------------- CAN Functions --------------------
void clear_receive_buffer() {
  while (CAN.available()) CAN.read();
}

int sdo_download(uint16_t odvalue, uint16_t datavalue) {
  uint8_t const msg_data[] = {
    0x20, odvalue & 0xFF, odvalue >> 8, 0x00,
    datavalue & 0xFF, datavalue >> 8, 0, 0
  };
  CanMsg const msg(CanStandardId(CAN_ID), sizeof(msg_data), msg_data);
  int rc = CAN.write(msg);
  if (rc < 0) return rc;

  unsigned long timeout = millis() + 100;
  while (millis() < timeout) {
    if (CAN.available()) {
      CanMsg ackmsg = CAN.read();
      uint16_t ack_od = (ackmsg.data[2] << 8) | ackmsg.data[1];
      uint16_t ack_val = (ackmsg.data[5] << 8) | ackmsg.data[4];
      if (ack_od == odvalue && ack_val == 0) return 1;
    }
    nh.spinOnce();
    delay(1);
  }
  return 0;
}

int send_sdo_upload(uint16_t odvalue) {
  uint8_t const msg_data[] = {0x40, odvalue & 0xFF, odvalue >> 8, 0x00, 0, 0, 0, 0};
  CanMsg msg(CanStandardId(CAN_ID), sizeof(msg_data), msg_data);
  return CAN.write(msg);
}

void receive_sdo_upload_array(uint16_t const* indices, float const* scalings, float* values, int length) {
  unsigned long timeout = millis() + 100;
  while (millis() < timeout) {
    if (CAN.available()) {
      CanMsg msg = CAN.read();
      uint16_t odindex = (msg.data[2] << 8) | msg.data[1];
      int32_t val = (msg.data[7] << 24) | (msg.data[6] << 16) | (msg.data[5] << 8) | msg.data[4];
      for (int i = 0; i < length; ++i) {
        if (odindex == indices[i]) {
          values[i] = scalings[i] * val;
        }
      }
    }
    nh.spinOnce();
    delay(1);
  }
}

void read_params() {
  clear_receive_buffer();
  for (int i = 0; i < readindices_length; ++i) {
    send_sdo_upload(read_indices[i]);
    delay(10);
    receive_sdo_upload_array(read_indices, readindices_scaling, readindices_values, readindices_length);
  }


  // Log all read parameters
  nh.loginfo("-------------------");
  nh.loginfo("Read Parameters:");
  for (int i = 0; i < readindices_length; ++i) {
    char log_msg[60]; // Buffer to hold the log message
    snprintf(
      log_msg, sizeof(log_msg),
      "%s (0x%04X): %.2f",
      read_indices_names[i],    // Parameter name (e.g., "OD_MaxSpeed")
      read_indices[i],          // OD index in hex (e.g., 0x3840)
      readindices_values[i]     // Scaled value (e.g., 5000.00 RPM)
    );
    nh.loginfo(log_msg);
  }
  nh.loginfo("-------------------");

  max_rpm = readindices_values[0];
  deadband_voltage = readindices_values[7];
  map_setting = readindices_values[8];
  voltage_max = readindices_values[9];
  offset = readindices_values[10];



  if (max_rpm <= 0 || voltage_max <= 0) {
    nh.loginfo("Invalid read parameters. Halting.");
    while (true) {
      nh.spinOnce();
      delay(10);
    }
  }
}

// -------------------- Setup --------------------
void setup() {
  
  pinMode(A0, OUTPUT);
  //Serial.begin(115200);
  jrkSerial.begin(9600);
  
  nh.initNode();
  // Wait for ROS connection first
  nh.getHardware()->setBaud(115200);
  
  delay(1000);

  while (!nh.connected()) {
    nh.loginfo("Waiting for ROS...");
    nh.spinOnce();
    delay(1000);
  }
  nh.loginfo("ROS connected :)");

  nh.subscribe(sub);
  nh.advertise(chatter);
  
  for (int i = 0; i < 50 ; i++) {
    nh.spinOnce();
    delay(100);
  }

  // CAN initialization
  if (!CAN.begin(CAN_BITRATE)) {
    nh.loginfo("CAN init failed!");
    while (true) {
      nh.spinOnce();
      delay(10);
    }
  }
  nh.loginfo("CAN started.");

  // SDO setup
  if (sdo_download(OD_AccelRate, 1000) < 0 || sdo_download(OD_DecelRate, 100) < 0 || sdo_download(OD_BrakeRate, 100) < 0) {
    nh.loginfo("SDO setup failed.");
    while (true) {
      nh.spinOnce();
      delay(10);
    }
  }

  read_params();
  nh.loginfo("Setup completed.");
  start_time = millis();
}

// -------------------- Loop --------------------
void loop() {
  static uint8_t monitor_index = 0;

  send_sdo_upload(monitor_indices[monitor_index]);
  monitor_index = (monitor_index + 1) % monitorindices_length;
  receive_sdo_upload_array(monitor_indices, monitorindices_scaling, monitorindices_values, monitorindices_length);

  if (!nh.connected()) {
    nh.loginfo(" ROS connection lost....");
  }
  else{
    static unsigned long last_pub_time = 0;
    static bool topic_configured = false;
    
    if (millis() - last_pub_time >= 100) {

      char speed_str[20];
      dtostrf(monitorindices_values[5], 7, 3, speed_str);
      char rpm_str[20];
      dtostrf(monitorindices_values[0], 9, 3, rpm_str);

      char log_msg[50];
      snprintf(log_msg, sizeof(log_msg), "Speed: %s km/h | RPM: %s", speed_str, rpm_str);
      nh.loginfo(log_msg);

      
      vel_msg.data = monitorindices_values[5]; // VehSpeed
      chatter.publish(&vel_msg);
      last_pub_time = millis();
    }
  }
 

  if (millis() - start_time < test_time) {
    // Braking/jrk activation during testing
    if (desired_voltage < Brake_Voltage_Threshold || start_time < 10000 ) {
      analogWrite(A0, 0);
      nh.loginfo("Braking activated");
      jrk.setTarget(Jrk_Brake_Position);
      delay(6000);
      jrk.setTarget(Jrk_Normal_Position);
    }
    else {
      float constrained_voltage = constrain(desired_voltage, 0.5, 4.5);
      int dc = (int)(255 * (desired_voltage - 0.5) / 4.0);
      analogWrite(A0, dc);
      jrk.setTarget(Jrk_Normal_Position);
    }
    

  } 
  else {
    analogWrite(A0,0);
    jrk.setTarget(Jrk_Brake_Position);
    delay(6000);
    jrk.setTarget(Jrk_Normal_Position);
    nh.loginfo("Testing Completed");
    while (true) {
      nh.spinOnce();
      delay(100);
    }
  }

  delay(10);
}
