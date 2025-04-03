# Adaptive-Cruise-Control
In this all the work and files regarding our project will be found 



#include <ros.h>
ros::NodeHandle nh;

void setup() {
  Serial.begin(115200);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  delay(1000);
  Serial.println("Test message from Arduino");
}

void loop() {
  Serial.println("Hello from Arduino");
  delay(1000);
  nh.spinOnce();
}
