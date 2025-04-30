#include <Arduino_CAN.h>
#include <JrkG2.h>

#define CAN_BITRATE CanBitRate::BR_500k
#define CAN_ID 0x626

#ifdef SERIAL_PORT_HARDWARE_OPEN
#define jrkSerial SERIAL_PORT_HARDWARE_OPEN
#else
#include <SoftwareSerial.h>
SoftwareSerial jrkSerial(10, 11);
#endif

JrkG2Serial jrk(jrkSerial); 

//#define OD_MaxSpeedLim  0x3559
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
//#define OD_ThrFilter    0x3030

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
float const readindices_scaling[] = {1, 1, 30.0/3000, 30.0/30000 ,30.0/30000 ,100.0/32767 ,100.0/32767 ,5.0/32767, 100.0/32767, 5.0/32767, 100.0/32767, 0.1};
int const readindices_length = sizeof(read_indices)/sizeof(read_indices[0]);
float readindices_values[readindices_length] = {};

uint16_t const monitor_indices[] = {OD_MotorRPM, OD_ThrottlePot, OD_ThrottleCmd, OD_CurrentRMS, OD_RegenState, OD_VehSpeed, OD_VehDist, OD_VehAccel, OD_MasterTime};
float const monitorindices_scaling[] = {1, 5.5/36044, 100.0/32767, 0.1, 1, 0.1, 0.1, 10.0/10000, 0.1};
int const monitorindices_length = sizeof(monitor_indices)/sizeof(monitor_indices[0]);
float monitorindices_values[monitorindices_length] = {};

float desired_voltage = 0.5;        // Input will be obtained from the user or start with default voltage
bool targetReached = false;

float max_rpm = readindices_values[0];
float deadband_voltage = readindices_values[7];
float map_setting = readindices_values[8];
float voltage_max = readindices_values[9];
float offset = readindices_values[10];

//Implement forward/reverse mode select using Arduino before doing braking test using reverse command
bool reverseTest = false;
float reverse_voltage = 1;

unsigned long start_time = 0xFFFFFFFF;           // Variable to store the start time, The initialized value is sentinal value indicating 'Not Initialized'
unsigned long test_time = 10000;                 // Sends throttle pot signal to the motor controller for this time (in milliseconds)

// unsigned long lastInputTime = 0;   // Variable to store the time when the last input is given
// const unsigned long inputInterval = 100; // 0.1 seconds (100ms)
// unsigned long lastControlTime = 0;
// const unsigned long controlInterval = 20; // 50Hz control loop

void clear_receive_buffer() {
  while(CAN.available()) {
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
    Serial.print("Upload failed with error code: ");
    Serial.println(rc);
    return rc;
  }

  //Checking
  while(true) {
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
      break;
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
    Serial.print("Upload failed with error code: ");
    Serial.println(rc);
  }

  return rc;

}

void receive_sdo_upload() {

  CanMsg const msg = CAN.read();

  if(msg.id == 0x726) {   //do nothing if heartbeat
    return;
  }

  uint8_t databytes[] = {msg.data[4], msg.data[5], msg.data[6], msg.data[7]};
  int datavalue = databytes[3] << 24 | databytes[2] << 16 | databytes[1] << 8 | databytes[0];    //bitshift and concatenate; using int for accurate mapping from unsigned 32-bit to signed

  uint8_t indexbyte1 = msg.data[1];
  uint8_t indexbyte2 = msg.data[2];
  uint16_t odindex = ((indexbyte2 << 8) | indexbyte1); 

  for(int i=0; i < readindices_length; i++) {
    if(odindex == read_indices[i]) {
      readindices_values[i] = readindices_scaling[i]*datavalue;
    }
  }

}

void receive_sdo_upload_monitor() {

  CanMsg const msg = CAN.read();

  if(msg.id == 0x726) {   //do nothing if heartbeat
    return;
  }

  uint8_t databyte1 = msg.data[4];
  uint8_t databytes[] = {msg.data[4], msg.data[5], msg.data[6], msg.data[7]};
  int datavalue = databytes[3] << 24 | databytes[2] << 16 | databytes[1] << 8 | databytes[0];    //bitshift and concatenate; using int for accurate mapping from unsigned 32-bit to signed

  uint8_t indexbyte1 = msg.data[1];
  uint8_t indexbyte2 = msg.data[2];
  uint16_t odindex = ((indexbyte2 << 8) | indexbyte1); 

  for(int i=0; i < monitorindices_length; i++) {
    if(odindex == monitor_indices[i]) {
      monitorindices_values[i] = monitorindices_scaling[i]*datavalue;
    }
  }

}

void read_params() {
  int rc = 0;
  CAN.read();
  for(int i=0; i < readindices_length; i++) {
    send_sdo_upload(read_indices[i]);
    delay(5);

    while(!CAN.available());
    while(CAN.available()) {
      receive_sdo_upload();
    }
    delay(5);
  }

  for(int i=0; i < readindices_length; i++) {
    Serial.print(readindices_values[i]);
    Serial.print(", ");
  }

  max_rpm = readindices_values[0];
  deadband_voltage = readindices_values[7];
  map_setting = readindices_values[8];
  voltage_max = readindices_values[9];
  offset = readindices_values[10];

  if(max_rpm == -1 || deadband_voltage == -1 || map_setting == -1 || voltage_max == -1 || offset == -1) {
    Serial.println("All parameters not read");
    while(true);
  }
  
}


void setup() { 
  pinMode(A0, OUTPUT);
  Serial.begin(115200);
  jrkSerial.begin(9600); // Need to check on this 

  if (!CAN.begin(CAN_BITRATE))
  {
    Serial.println("CAN.begin(...) failed.");
    for (;;) {}
  }
  delay(2000);
  Serial.println();

  Serial.println("Setting up parameters");
  clear_receive_buffer();
  if(int rc = sdo_download(OD_AccelRate, 1000) < 0) {     
    Serial.println("SDO download failed");
    for(;;) {}
  }
  delay(10);
  if(int rc = sdo_download(OD_DecelRate, 100) < 0) {     
    Serial.println("SDO download failed");
    for(;;) {}
  }
  delay(10);
  if(int rc = sdo_download(OD_BrakeRate, 100) < 0) {     
  Serial.println("SDO download failed");
  for(;;) {}
  }

  delay(2000);
  Serial.println();
  Serial.println("1: Read parameters");
  Serial.println("2: System Ready, Start test");

  int choice;

  while(Serial.available() <= 0);                                                                                                                                                           
  if(Serial.available()>0) {
    choice = Serial.parseInt();
    Serial.parseInt();
  }
  
  switch(choice) {
    case 1:
    delay(1000);
    read_params();
    while(true);
    break;
    
    case 2:
    delay(1000);
    read_params();

    // Set default desired voltage and inform user
    desired_voltage = 0.5;
    Serial.println("Starting test. Send desired voltage (0.5-4.5V) during the test.");
    delay(1000); // Short delay before starting loop
    break;

    default:
    Serial.println("Invalid choice");
    while(true);
  }
  
}

//Confirm if all measured and used values belong to the correct index in the list
void loop() {
  int rc = 0;

  for(int i=0; i < monitorindices_length; i++) {
    send_sdo_upload(monitor_indices[i]);
    delay(5);
    
    while(!CAN.available());
    while(CAN.available()) {
      receive_sdo_upload_monitor();
      delay(5);
    }
  }
  

  for(int i=0; i < monitorindices_length; i++) {
      Serial.print(monitorindices_values[i]);
      Serial.print(",");
  }
  Serial.print(0.001*millis());
  Serial.println();

  //Confirm if all measured and used values belong to the correct index in the list
  
  if (start_time == 0xFFFFFFFF) {
    start_time = millis();
  }



  if(millis() - start_time <= test_time) {

    if (Serial.available() > 0){
      desired_voltage = Serial.parseFloat();
      desired_voltage = constrain(desired_voltage, 0.5, 4.5); // Clamp the voltage
      // Clear the buffer
      while (Serial.available() > 0) {
        Serial.read();
      }
      Serial.print("New voltage set: ");
      Serial.println(desired_voltage);

    }
    
    int dc = (int) 255 * (desired_voltage - 0.5)/ (4.0); // Giving Voltage as input

    analogWrite(A0, dc);
  }

  else {
    targetReached = true;

    if(reverseTest) {
      if(monitorindices_values[0] > 100) {
        analogWrite(A0, 255 * (reverse_voltage -0.5) / 4);
      }
      else {
        analogWrite(A0, 0);
        jrk.setTarget(690);
        delay(4000);
        jrk.setTarget(2600);
        exit(0);
      }
    }
    else {
      analogWrite(A0, 0);
      jrk.setTarget(690);
      delay(4000);
      jrk.setTarget(2600);
      exit(0);
    }
  }
}
