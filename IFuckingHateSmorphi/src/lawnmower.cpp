#include <smorphi.h>

Smorphi my_robot;
int front_sensor_status;
int left_sensor_status;
int right_sensor_status;
int rear_sensor_status;
bool turning_right = true;

// Variables for timing sensor readings
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 2000; // Print every 2 seconds


void setup() {
    Serial.begin(115200);
    my_robot.BeginSmorphi();
    my_robot.I();
    Serial.println("beginning!");
}

void printSensorStatus() {
    // just for debug
    unsigned long currentTime = millis();
    if (currentTime - lastPrintTime >= 500) {
        Serial.println("\n--- Sensor Readings ---");
        Serial.print("Front: ");
        Serial.print(front_sensor_status);
        Serial.print(" | Left: ");
        Serial.print(left_sensor_status);
        Serial.print(" | Right: ");
        Serial.print(right_sensor_status);
        Serial.print(" | Rear: ");
        Serial.println(rear_sensor_status);
        lastPrintTime = currentTime;
    }
}

void performTurn() {
  Serial.println(" performturn called");
  my_robot.MoveBackward(255);
  delay(900);
  
    if (turning_right) {
      Serial.println("turning right!");
      // need to tune turning duration of all the below
        // First turn right
        my_robot.CenterPivotRight(200);
        delay(3000);
        my_robot.stopSmorphi();
        delay(100);
        
        // Move forward a bit
        Serial.println("Moving forward");
        my_robot.MoveForward(100);
        delay(1000);
        my_robot.stopSmorphi();
        delay(100);
        
        // Second turn right
        Serial.println("Second right turn");
        my_robot.CenterPivotRight(200);
        delay(2800);
        my_robot.stopSmorphi();
        delay(100);
        
    } else {
        Serial.println("turning left!");

        // First turn left
        my_robot.CenterPivotLeft(200);
        delay(3000);
        my_robot.stopSmorphi();
        delay(500);

        // Move forward a bit
        Serial.println("Moving forward");
        my_robot.MoveForward(100);
        delay(1000);
        my_robot.stopSmorphi();
        delay(100);
        
        // Second turn left
        Serial.println("Second left turn");
        my_robot.CenterPivotLeft(200);
        delay(3500);
        my_robot.stopSmorphi();
        delay(500);
    }
    turning_right = !turning_right;
}

void loop() {
    front_sensor_status = my_robot.module1_sensor_status(4);
    left_sensor_status = my_robot.module4_sensor_status(3);
    right_sensor_status = my_robot.module2_sensor_status(3);
    rear_sensor_status = my_robot.module3_sensor_status(4);
    Serial.println("loop started!");

    // Print sensor status periodically
    printSensorStatus();

    if (front_sensor_status == 0) {
        Serial.println("front sensor detected something!");
        my_robot.stopSmorphi();
        delay(100);
        Serial.println("calling performturn!");
        performTurn();
    } else {
        // my_robot.MoveForward(100);
        delay(50);
    }


}