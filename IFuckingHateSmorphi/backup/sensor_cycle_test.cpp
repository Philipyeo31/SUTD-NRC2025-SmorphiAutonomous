#include <smorphi.h>

Smorphi my_robot;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  my_robot.BeginSmorphi();
}

void loop() {
  // put your main code here, to run repeatedly:
  //int sensor_status = my_robot.module1/2/3/4_sensor_status(0/2/4/6/8/10);  change the paramter according to which module and port you use
  int sensor_status = my_robot.module1_sensor_status(3);  // using sensor on module 1 port 2 on hardware side
  Serial.println(sensor_status);
  //If sensor returns LOW, something is in front
  if (sensor_status == HIGH){
    Serial.println("Nothing is in front!");
    // robot moves forward with speed 10

    }
  else {
    Serial.println("Something is in front!");    
    my_robot.stopSmorphi();
    delay(2000); // stop for 2 seconds
   }
}