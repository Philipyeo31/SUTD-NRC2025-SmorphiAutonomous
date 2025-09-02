#include <smorphi.h>

Smorphi my_robot;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  my_robot.BeginSmorphi();
  my_robot.O();
  
  int front_sensor_status = my_robot.module2_sensor_status(0);

  /* my_robot.MoveForward(255);
  delay(1000);
  my_robot.MoveRight(40);
  delay(1000); */
  
  while (front_sensor_status == 1 ){
    if ((front_sensor_status) == 0){
      my_robot.stopSmorphi();
  }
  else {
    my_robot.MoveForward(20);
    delay(4000);
  }
  }
  
}

void loop() {
  

}