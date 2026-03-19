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

// Object dictionary indices and scaling factors (unchanged from your code)
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
#define OD_VehSpeed     0x320A   // For feedback, we use vehicle speed (adjust if needed)
#define OD_VehDist      0x3616
#define OD_VehAccel     0x35C1

uint16_t const read_indices[] = {OD_MaxSpeed, OD_MaxSpCont, OD_AccelRate, OD_DecelRate, OD_BrakeRate, OD_DriveCurrL, OD_RegenCurrL, OD_ForwardDB, OD_ForwardMap, OD_ForwardMax, OD_ForwardOff, OD_SpeedToRPM};
float const readindices_scaling[] = {1, 1, 30.0/3000, 30.0/30000, 30.0/30000, 100.0/32767, 100.0/32767, 5.0/32767, 100.0/32767, 5.0/32767, 100.0/32767, 0.1};
int const readindices_length = sizeof(read_indices)/sizeof(read_indices[0]);
float readindices_values[readindices_length] = {};

// Monitor indices (for feedback)
uint16_t const monitor_indices[] = {OD_MotorRPM, OD_ThrottlePot, OD_ThrottleCmd, OD_CurrentRMS, OD_RegenState, OD_VehSpeed, OD_VehDist, OD_VehAccel, OD_MasterTime};
float const monitorindices_scaling[] = {1, 5.5/36044, 100.0/32767, 0.1, 1, 0.1, 0.1, 10.0/10000, 0.1};
int const monitorindices_length = sizeof(monitor_indices)/sizeof(monitor_indices[0]);
float monitorindices_values[monitorindices_length] = {};

// Global control variables
float desired_voltage = 0;        // Will be updated via ROS subscriber callback
bool targetReached = false;

// Parameters read from the object dictionary
float max_rpm = 0;
float deadband_voltage = 0;
float map_setting = 0;
float voltage_max = 0;
float offset = 0;

// For optional reverse test (not used unless set to true)
bool reverseTest = false;
float reverse_voltage = 1;

unsigned long start_time = 0xFFFFFFFF;           // Sentinel value for "not initialized"
unsigned long test_time = 20000;                 // Time (ms) during which we apply the command

// ROS node handle
ros::NodeHandle nh;

// Publisher for velocity feedback
std_msgs::Float32 vel_msg;
ros::Publisher velocity_pub("velocity_feedback", &vel_msg);

// ROS subscriber callback for receiving motor commands
void motorCommandCallback(const std_msgs::Float32 &msg) {
  desired_voltage = msg.data;
}
ros::Subscriber<std_msgs::Float32> motor_sub("motor_command", motorCommandCallback);

// Clears any pending messages in the CAN receive buffer.
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
    Serial.print("SDO download failed with error code: ");
    Serial.println(rc);
    return rc;
  }
  // Wait for acknowledgement
  while (true) {
    if(CAN.available()) {
      CanMsg const ackmsg = CAN.read();
      uint8_t readbyte1 = ackmsg.data[4];
      uint8_t readbyte2 = ackmsg.data[5];
      short readvalue = ((readbyte2 << 8) | readbyte1);
      uint8_t indexbyte1 = ackmsg.data[1];
      uint8_t indexbyte2 = ackmsg.data[2];
      uint16_t odindex = ((indexbyte2 << 8) | indexbyte1);
      if(odindex == odvalue && readvalue == 0) {
        Serial.print("Successfully changed: ");
        Serial.print(odindex);
        Serial.print(" => ");
        Serial.println(datavalue);
        return 1;
      }
      else {
        Serial.println("Error in updating value");
        Serial.println(ackmsg);
        return 0;
      }
    }
  }
  return 0;
}

int send_sdo_upload(uint16_t odvalue) {
  uint8_t firstbyte = odvalue % 256;
  uint8_t secondbyte = odvalue / 256;
  uint8_t const msg_data[] = {0x40, firstbyte, secondbyte, 0x00, 0, 0, 0, 0};
  CanMsg const msg(CanStandardId(CAN_ID), sizeof(msg_data), msg_data);
  int const rc = CAN.write(msg);
  if(rc < 0) {
    Serial.print("SDO upload failed with error code: ");
    Serial.println(rc);
  }
  return rc;
}

void receive_sdo_upload() {
  CanMsg const msg = CAN.read();
  // Skip heartbeat message if needed
  if(msg.id == 0x726) {
    return;
  }
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

void receive_sdo_upload_monitor() {
  CanMsg const msg = CAN.read();
  if(msg.id == 0x726) {
    return;
  }
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

// Reads parameters from the motor controller via CAN SDO uploads.
void read_params() {
  // Clear any pending messages
  clear_receive_buffer();
  for (int i = 0; i < readindices_length; i++) {
    send_sdo_upload(read_indices[i]);
    delay(5);
    while (!CAN.available());
    while (CAN.available()) {
      receive_sdo_upload();
    }
    delay(5);
  }
  // Assign parameter values
  max_rpm = readindices_values[0];
  deadband_voltage = readindices_values[7];
  map_setting = readindices_values[8];
  voltage_max = readindices_values[9];
  offset = readindices_values[10];
  // If parameters were not read correctly, halt.
  if(max_rpm == -1 || deadband_voltage == -1 || map_setting == -1 || voltage_max == -1 || offset == -1) {
    Serial.println("All parameters not read");
    while (true);
  }
}

void setup() { 
  // Initialize hardware and CAN
  pinMode(A0, OUTPUT);
  Serial.begin(115200);
  jrkSerial.begin(9600);

  if (!CAN.begin(CAN_BITRATE)) {
    Serial.println("CAN.begin(...) failed.");
    for (;;) {}
  }
  delay(2000);
  Serial.println("Setting up parameters");
  clear_receive_buffer();
  
  // Example SDO downloads to configure acceleration/brake rates
  if (sdo_download(OD_AccelRate, 1000) < 0) {     
    Serial.println("SDO download failed");
    for(;;) {}
  }
  delay(10);
  if (sdo_download(OD_DecelRate, 100) < 0) {     
    Serial.println("SDO download failed");
    for(;;) {}
  }
  delay(10);
  if (sdo_download(OD_BrakeRate, 100) < 0) {     
    Serial.println("SDO download failed");
    for(;;) {}
  }
  delay(2000);
  
  // Read parameters from the motor controller
  read_params(); 
  
  // Initialize ROS node, subscriber and publisher.
  nh.initNode();
  nh.subscribe(motor_sub);
  nh.advertise(velocity_pub);
  
  // Initialize timer for applying the voltage command.
  start_time = millis();
}

void loop() {
  // Request and process monitor data via CAN SDO uploads
  for (int i = 0; i < monitorindices_length; i++) {
    send_sdo_upload(monitor_indices[i]);
    delay(5);
    while (!CAN.available());
    while (CAN.available()) {
      receive_sdo_upload_monitor();
      delay(5);
    }
  }
  
  // Publish velocity feedback.
  // Here we use the vehicle speed (monitorindices_values[5]). Adjust the index as needed.
  vel_msg.data = monitorindices_values[5];
  velocity_pub.publish(&vel_msg);
  
  // Process incoming ROS messages
  nh.spinOnce();
  
  // Motor control: if within test time, apply the voltage command received via ROS.
  if (millis() - start_time <= test_time) {
    // Clamp desired_voltage between 0.5 and 4.5 (example limits)
    if (desired_voltage < 0.5) {
      desired_voltage = 0.5;
    } else if (desired_voltage > 4.5) {
      desired_voltage = 4.5;
    }
    // Calculate PWM value based on desired_voltage (assumes 0.5V => 0 PWM, 4.5V => 255 PWM)
    int dc = (int)(255 * (desired_voltage - 0.5) / 4.0);
    analogWrite(A0, dc);
  }
  else {
    // Once test time is over, stop the PWM output.
    targetReached = true;
    analogWrite(A0, 0);
    // send commands to the Jrk motor controller.
    jrk.setTarget(690);
    delay(4000);
    jrk.setTarget(2600);
    while (true) {
      nh.spinOnce();
      delay(100);
    }
  }
  
  delay(100);  // Loop delay (adjust as needed)
}
