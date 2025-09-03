#include <Smorphi.h>
#include <Arduino.h>
#include <Pixy2ICSP_ESP32.h>

Smorphi my_robot;
Pixy2ICSP_ESP32 pixy;

// initialise IR Sensors
bool front_sensor;
bool right_sensor;
bool rear_sensor;
bool left_sensor;
string sensor_readings;


// Global x and y speed variables
int y_speed = 150; // Forward & Backward speed 
int x_speed = 120; // Lateral Left & Right speed 





// Shape completion flags
String I_shape_done = "false";
String O_shape_done = "false";
String L_shape_done = "false";

// Color detection flags
String red_color_done = "false";
String green_color_done = "false";
String blue_color_done = "false";

// Movement state tracking
String movement_instruction = "forward";

long x_pos = 0;
long y_pos = 0;
long delta_x_pos = 0;

unsigned long newLoopTime = 0;
unsigned long movement_start_time = 0;
unsigned long movement_duration = 0;


//Define Functions
void move(movement_instruction){
  //debug prints
  Serial.println("=============================")
  Serial.println("move() called @ " + String(millis()) + " ms");
  Serial.println("movement_instruction: " + movement_instruction);
  Serial.println("sensor_readings: " + sensor_readings);
  Serial.println("x_pos: " + String(x_pos));
  Serial.println("y_pos: " + String(y_pos));

  switch (movement_instruction){
    //move forward
    case forward{
      if front_sensor == 0 { //no obstacle in front
        my_robot.moveForward(y_speed) //move forward
        movement_start_time = millis(); //record time movement started
        while (front_sensor == 0){ //keep moving forward until obstacle detected
          sensor_initialisation(); //update sensor readings
          Serial.println("fwd, sensor_readings: " + sensor_readings);
          delay(50);
        }
        my_robot.stopSmorphi(); //stop movement
        movement_duration = millis() - movement_start_time; //calculate duration of movement
        y_pos += movement_duration; //update y position
        delay(100);
      }
    }

    //move right
    case right{
      if right_sensor == 0{ //no obstacle on right
        my_robot.moveRight(x_speed); //move right
        movement_start_time = millis(); //record time movement started
        while (right_sensor == 0){ //keep moving right until obstacle detected
          sensor_initialisation(); //update sensor readings
          Serial.println("right, sensor_readings: " + sensor_readings);
          delay(50);
        }
        my_robot.stopSmorphi(); //stop movement
        movement_duration = millis() - movement_start_time; //calculate duration of movement
        delta_x_pos = movement_duration; //distance strayed from original path.
        x_pos += movement_duration; //update x position
        delay(100);
      }
    }

    //move left
    case left{  
      if left_sensor == 0{ //no obstacle on left
        my_robot.moveLeft(x_speed); //move left
        movement_start_time = millis(); //record time movement started
        while (left_sensor == 0){ //keep moving left until obstacle detected
          sensor_initialisation(); //update sensor readings
          Serial.println("left, sensor_readings: " + sensor_readings);
          delay(50);
        }
        my_robot.stopSmorphi(); //stop movement
        movement_duration = millis() - movement_start_time; //calculate duration of movement
        delta_x_pos -= movement_duration; //distance strayed from original path.
        x_pos -= movement_duration; //update x position
        delay(100);
      }
    }

    //move back
    case backward{
      if rear_sensor == 0{ //no obstacle at back
        my_robot.moveBackward(y_speed); //move backward
        movement_start_time = millis(); //record time movement started
        while (rear_sensor == 0){ //keep moving backward until obstacle detected
          sensor_initialisation(); //update sensor readings
          Serial.println("bkwd, sensor_readings: " + sensor_readings);
          delay(50);
        }
        my_robot.stopSmorphi(); //stop movement
        movement_duration = millis() - movement_start_time; //calculate duration of movement
        y_pos -= movement_duration; //update y position
        delay(100);
      }
    }
    case stop{
      my_robot.stopSmorphi(); //stop movement
      delay(100);
    }
    case sidestep_right{
      while (front_sensor == 1 && right_sensor == 0){ //no obstacle on right
        my_robot.moveRight(x_speed); //move right
        sensor_initialisation(); //update sensor readings
        Serial.println("sdstp_rgt, sensor_readings: " + sensor_readings);
        delay(50);
      }
      my_robot.stopSmorphi(); //stop movement
      movement_duration = millis() - movement_start_time; //calculate duration of movement
      delta_x_pos = movement_duration; //distance strayed from original path.
      x_pos += movement_duration; //update x position
      delay(100);
      my_robot.moveFront(y_speed);
      delay(100);
      my_robot.moveLeft(x_speed);
      delay(delta_x_pos);
      my_robot.stopSmorphi();
      x_pos -= delta_x_pos; //update x position
    }
    case sidestep_left{
      while (front_sensor == 1 && left_sensor == 0){ //no obstacle on left
        my_robot.moveLeft(x_speed); //move left
        sensor_initialisation(); //update sensor readings
        Serial.println("sdestp_lft, sensor_readings: " + sensor_readings);
        delay(50);
      }
      my_robot.stopSmorphi(); //stop movement
      movement_duration = millis() - movement_start_time; //calculate duration of movement
      delta_x_pos = movement_duration; //distance strayed from original path.
      x_pos -= movement_duration; //update x position
      delay(100);
      my_robot.moveFront(y_speed);
      delay(100);
      my_robot.moveRight(x_speed);
      delay(delta_x_pos);
      my_robot.stopSmorphi();
      x_pos += delta_x_pos; //update x position
    }
    }
//phil's functions


void basic_obstacle_avoidance(){
  move(forward);
  if sensor_readings == "1000"{ //obstacle in front
    move(right);
    move(forward);
    move(right);
  }
  else if sensor_readings == "0100"{ //obstacle on right
    move(left);
    move(forward);
    move(left);
  }
  else if sensor_readings == "0010"{ //obstacle at back
    move(forward); //keep going forward
  }
  else if sensor_readings == "0001"{ //obstacle on left
    move(right);
    move(forward);
    move(right);
  }
  else if sensor_readings == "1100"{ //obstacle in front and right
    move(left);
    move(forward);
    move(left);
  }
  else if sensor_readings == "1010"{ //obstacle in front and back
    move(right); //prefer right turn
    move(forward);
    move(right);
  }
  else if sensor_readings == "1001"{ //obstacle in front and left
    move(right); //prefer right turn
    move(forward);
    move(right);
  }
  else if sensor_readings == "0110"{ //obstacle on right and back
    move(left);
    move(forward);
    move(left);
  }
  else if sensor_readings == "0101"{ //obstacle on right and left
    move(forward); //keep going forward
  }
  else if sensor_readings == "0011"{ //obstacle on back and left
    move(right);
    move(forward);
    move(right);
  }
  else if sensor_readings == "1110"{ //obstacle in front, right, and back
    move(left);
    move(forward);
    move(left);
  }
  else if sensor_readings == "1101"{ //obstacle in front, right, and left
    move(left); //prefer left turn
    move(forward);
    move(left);
  }
  else if sensor_readings == "1011"{ //obstacle in front, back, and left
    move(right); //prefer right turn
    move(forward);
    move(right);
  }
  else if sensor_readings == "0111"{ //obstacle on right, back, and left
    move(forward); //keep going forward
  }
  else if sensor_readings == "1111"{ //obstacles all around
    movement_instruction = stop; //stop
  }
}

void lawnmower_pattern(){
  move(forward);
  sensor_initialisation();
  sidestep_right(); // moves right until no obstacle in front
  move(forward); //move forward a bit
  sensor_initialisation();
  

}


void setup() {
  Serial.begin(115200);
  my_robot.BeginSmorphi();
  pixy.init();
}

void loop() {
  


  /*
  // Read all sensors
  sensor_initialisation();
  
  // Check for color markers with Pixy
  check_color_markers();
  
  // Execute movement based on current state
  execute_movement();
  */
}









//all functions below generated by AI

void sensor_initialisation() {
  // Read all IR sensors - adjust module numbers based on your robot's configuration
  // Note: Sensor logic is inverted (0 = obstacle, 1 = clear) therefore the '!' operator is used
  front_sensor = !my_robot.module1_sensor_status(0);
  right_sensor = !my_robot.module1_sensor_status(4);
  rear_sensor = !my_robot.module3_sensor_status(0);
  left_sensor = !my_robot.module1_sensor_status(10);

  sensor_readings = String(front_sensor) + String(right_sensor) + String(rear_sensor) + String(left_sensor);
}

void check_color_markers() {
  pixy.ccc.getBlocks();
  
  if (pixy.ccc.numBlocks) {
    int color_signature = pixy.ccc.blocks[0].m_signature;
    
    // Red marker - change to I-shape
    if (color_signature == 1 && red_color_done != "True") {
      if (my_robot.sm_getShape() != 'i') {
        change_shape("I");
        I_shape_done = "True";
        red_color_done = "True";
        // Reset other color flags
        green_color_done = "false";
        blue_color_done = "false";
      }
    }
    // Green marker - change to O-shape
    else if (color_signature == 2 && green_color_done != "True") {
      if (my_robot.sm_getShape() != 'o') {
        change_shape("O");
        O_shape_done = "True";
        green_color_done = "True";
        // Reset other color flags
        red_color_done = "false";
        blue_color_done = "false";
      }
    }
    // Blue marker - change to L-shape
    else if (color_signature == 3 && blue_color_done != "True") {
      if (my_robot.sm_getShape() != 'l') {
        change_shape("L");
        L_shape_done = "True";
        blue_color_done = "True";
        // Reset other color flags
        red_color_done = "false";
        green_color_done = "false";
      }
    }
  }
}

void change_shape(String shape) {
  my_robot.sm_reset_M1();
  my_robot.sm_reset_M2();
  my_robot.sm_reset_M3();
  my_robot.sm_reset_M4();
  
  if (shape == "I") {
    my_robot.I();
  } else if (shape == "O") {
    my_robot.O();
  } else if (shape == "L") {
    my_robot.L();
  }
  
  // Update sensor readings after shape change
  sensor_initialisation();
}

