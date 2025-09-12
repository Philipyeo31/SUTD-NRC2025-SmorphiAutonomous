/*
 * Smorphi 2 Autonomous Navigation Algorithm
 * National Robotics Challenge - Area Coverage Logic
 * 
 * This code provides the navigation algorithm and logic.
 * Hardware control functions should be implemented separately.
 */

#include <vector>
#include <queue>
#include <algorithm>
#include <math.h>
#include <Arduino.h>
#include <smorphi.h>

Smorphi my_robot;

// ============= CONFIGURATION PARAMETERS =============
// Adjust these based on your robot and competition setup
#define MAP_SIZE_MM 3000          // 3m x 3m map
#define GRID_CELL_SIZE_MM 50      // Resolution for coverage tracking
#define GRID_SIZE (MAP_SIZE_MM / GRID_CELL_SIZE_MM)
#define MOVEMENT_INCREMENT_MM 50   // Distance to move in each step
#define SWEEP_OVERLAP_MM 20       // Overlap between parallel sweep lines

// ============= DATA STRUCTURES =============

// Robot State Machine States
enum RobotState {
    INITIALIZING,
    FORWARD_SWEEP,
    LATERAL_SHIFT,
    TURNING,
    OBSTACLE_DETECTED,
    COLOR_ZONE_DETECTED,
    SHAPE_CHANGING,
    EXPLORING_NEW_ROOM,
    BACKTRACKING,
    WALL_FOLLOWING,
    COMPLETED
};

// Movement directions
enum Direction {
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3
};

// Movement commands for your robot
enum MovementCommand {
    MOVE_FORWARD,
    MOVE_BACKWARD,
    TURN_LEFT_90,
    TURN_RIGHT_90,
    STOP,
    LATERAL_MOVE_LEFT,
    LATERAL_MOVE_RIGHT
};

// Sensor data structure
struct SensorData {
    bool frontObstacle;
    bool backObstacle;
    bool leftObstacle;
    bool rightObstacle;
    int colorDetected;  // 0 = no color, 1-4 = color IDs
    int frontDistance;  // Optional: distance in mm if available
    int leftDistance;
    int rightDistance;
    int backDistance;
};

// Position tracking
struct Position {
    int x;
    int y;
    
    bool operator==(const Position& other) const {
        return x == other.x && y == other.y;
    }
};

// Room information
struct Room {
    int id;
    Position entryPoint;
    bool fullyExplored;
    std::vector<Position> unexploredAreas;
};

// ============= MAIN NAVIGATION CLASS =============

class NavigationSystem {
private:
    // Robot state
    RobotState currentState;
    RobotState previousState;
    Direction heading;
    Position currentPosition;
    Position targetPosition;
    
    // Shape management
    int currentShapeIndex;  // 0: I-shape, 1: O-shape, 2: J-shape, 3: L-shape
    int robotWidthMM;       // Current width based on shape
    int robotLengthMM;      // Current length based on shape
    
    // Coverage tracking
    bool coverageGrid[GRID_SIZE][GRID_SIZE];
    int totalCellsCovered;
    std::vector<Position> visitedPositions;
    std::queue<Position> explorationQueue;
    
    // Room management
    std::vector<Room> rooms;
    int currentRoomId;
    
    // Sweep parameters
    int sweepDirection;     // 1 or -1
    int currentSweepLine;
    bool sweepComplete;
    Position sweepStartPos;
    int sweepLineSpacing;
    
    // Navigation flags
    bool isBacktracking;
    bool colorZoneProcessed;
    int stuckCounter;
    unsigned long lastProgressTime;
    Position lastPosition;
    
    // Wall following
    bool wallFollowMode;
    Direction wallSide;  // Which side the wall is on
    
public:
    NavigationSystem() {
        currentState = INITIALIZING;
        heading = NORTH;
        currentPosition = {0, 0};
        currentShapeIndex = 0;
        robotWidthMM = 200;  // Default I-shape width
        robotLengthMM = 400; // Default I-shape length
        totalCellsCovered = 0;
        sweepDirection = 1;
        currentSweepLine = 0;
        sweepComplete = false;
        isBacktracking = false;
        colorZoneProcessed = false;
        stuckCounter = 0;
        currentRoomId = 0;
        wallFollowMode = false;
        sweepLineSpacing = robotWidthMM - SWEEP_OVERLAP_MM;
        
        // Initialize coverage grid
        for(int i = 0; i < GRID_SIZE; i++) {
            for(int j = 0; j < GRID_SIZE; j++) {
                coverageGrid[i][j] = false;
            }
        }
        
        // Create first room
        Room firstRoom;
        firstRoom.id = 0;
        firstRoom.entryPoint = currentPosition;
        firstRoom.fullyExplored = false;
        rooms.push_back(firstRoom);
    }
    
    // ============= MAIN NAVIGATION FUNCTION =============
    
    MovementCommand navigate(SensorData sensors) {
        // Update coverage grid
        updateCoverageGrid();
        
        // Check for stuck condition
        if(detectStuckCondition()) {
            return handleStuckRecovery(sensors);
        }
        
        // Process color detection (highest priority)
        if(sensors.colorDetected > 0 && !colorZoneProcessed) {
            return handleColorZoneDetection(sensors.colorDetected);
        }
        
        // State machine logic
        switch(currentState) {
            case INITIALIZING:
                return initializeNavigation();
                
            case FORWARD_SWEEP:
                return executeForwardSweep(sensors);
                
            case LATERAL_SHIFT:
                return executeLateralShift(sensors);
                
            case TURNING:
                return executeTurn();
                
            case OBSTACLE_DETECTED:
                return handleObstacle(sensors);
                
            case COLOR_ZONE_DETECTED:
                return processColorZone(sensors.colorDetected);
                
            case SHAPE_CHANGING:
                return executeShapeChange();
                
            case EXPLORING_NEW_ROOM:
                return exploreNewRoom(sensors);
                
            case BACKTRACKING:
                return executeBacktrack(sensors);
                
            case WALL_FOLLOWING:
                return executeWallFollow(sensors);
                
            case COMPLETED:
                return STOP;
                
            default:
                return STOP;
        }
    }
    
    // ============= STATE HANDLERS =============
    
    MovementCommand initializeNavigation() {
        // INSERT YOUR INITIALIZATION CODE HERE
        // Example: calibrate sensors, set initial position, etc.
        
        currentState = FORWARD_SWEEP;
        sweepStartPos = currentPosition;
        return STOP;
    }
    
    MovementCommand executeForwardSweep(SensorData sensors) {
        // Check for obstacles in movement direction
        bool obstacleAhead = (sweepDirection > 0) ? sensors.frontObstacle : sensors.backObstacle;
        
        if(obstacleAhead) {
            // Reached end of sweep line
            if(shouldContinueSweep()) {
                currentState = LATERAL_SHIFT;
                return STOP;
            } else {
                // Sweep pattern complete, find unexplored areas
                Position nextTarget;
                if(findNearestUnexploredArea(nextTarget)) {
                    targetPosition = nextTarget;
                    currentState = BACKTRACKING;
                } else {
                    // Check if there are unexplored rooms
                    if(hasUnexploredRooms()) {
                        currentState = EXPLORING_NEW_ROOM;
                    } else {
                        currentState = COMPLETED;
                    }
                }
                return STOP;
            }
        }
        
        // Continue moving forward
        updatePosition(MOVEMENT_INCREMENT_MM);
        return (sweepDirection > 0) ? MOVE_FORWARD : MOVE_BACKWARD;
    }
    
    MovementCommand executeLateralShift(SensorData sensors) {
        static int shiftStage = 0;
        
        switch(shiftStage) {
            case 0: // Turn towards shift direction
                shiftStage = 1;
                return (currentSweepLine % 2 == 0) ? TURN_RIGHT_90 : TURN_LEFT_90;
                
            case 1: // Move laterally
                shiftStage = 2;
                updatePosition(sweepLineSpacing);
                return MOVE_FORWARD;
                
            case 2: // Turn back to sweep direction
                shiftStage = 3;
                return (currentSweepLine % 2 == 0) ? TURN_LEFT_90 : TURN_RIGHT_90;
                
            case 3: // Turn 180 for reverse sweep
                shiftStage = 4;
                sweepDirection *= -1;
                return TURN_RIGHT_90;
                
            case 4: // Complete 180 turn
                shiftStage = 0;
                currentSweepLine++;
                currentState = FORWARD_SWEEP;
                return TURN_RIGHT_90;
        }
        
        return STOP;
    }
    
    MovementCommand executeTurn() {
        // Simple 90-degree turn execution
        static bool turnComplete = false;
        
        if(!turnComplete) {
            turnComplete = true;
            return TURN_RIGHT_90;  // Or TURN_LEFT_90 based on need
        } else {
            turnComplete = false;
            currentState = previousState;
            return STOP;
        }
    }
    
    MovementCommand handleObstacle(SensorData sensors) {
        // Intelligent obstacle avoidance
        std::vector<Direction> availableDirections;
        
        if(!sensors.frontObstacle) availableDirections.push_back(NORTH);
        if(!sensors.rightObstacle) availableDirections.push_back(EAST);
        if(!sensors.backObstacle) availableDirections.push_back(SOUTH);
        if(!sensors.leftObstacle) availableDirections.push_back(WEST);
        
        if(availableDirections.empty()) {
            // Completely blocked - try to backtrack
            currentState = BACKTRACKING;
            return MOVE_BACKWARD;
        }
        
        // Choose direction that maximizes unexplored area
        Direction bestDirection = chooseBestDirection(availableDirections);
        return turnToDirection(bestDirection);
    }
    
    MovementCommand handleColorZoneDetection(int colorId) {
        // Stop and prepare for shape change
        currentState = COLOR_ZONE_DETECTED;
        colorZoneProcessed = true;
        
        // INSERT YOUR COLOR ZONE DETECTION LOGIC HERE
        // This should trigger when your HuskyLens detects a color
        
        return STOP;
    }
    
    MovementCommand processColorZone(int colorId) {
        // Determine new shape based on color
        int newShape = mapColorToShape(colorId);
        
        if(newShape != currentShapeIndex) {
            currentShapeIndex = newShape;
            currentState = SHAPE_CHANGING;
            
            // Update robot dimensions based on new shape
            updateRobotDimensions(newShape);
            
            // Mark entry to new room
            currentRoomId++;
            Room newRoom;
            newRoom.id = currentRoomId;
            newRoom.entryPoint = currentPosition;
            newRoom.fullyExplored = false;
            rooms.push_back(newRoom);
        } else {
            currentState = FORWARD_SWEEP;
        }
        
        return STOP;
    }
    
    MovementCommand executeShapeChange() {
        // INSERT YOUR SHAPE CHANGE CODE HERE
        // Call your Smorphi shape transformation functions
        // Example: 
        // if(currentShapeIndex == 0) smorphi.sm_reset_M();
        // if(currentShapeIndex == 1) smorphi.sm_O_formation();
        // etc.
        
        // After shape change complete
        currentState = FORWARD_SWEEP;
        colorZoneProcessed = false;
        
        // Recalculate sweep parameters for new shape
        sweepLineSpacing = robotWidthMM - SWEEP_OVERLAP_MM;
        
        return STOP;
    }
    
    MovementCommand exploreNewRoom(SensorData sensors) {
        // Navigate to unexplored room entry point
        for(auto& room : rooms) {
            if(!room.fullyExplored) {
                targetPosition = room.entryPoint;
                currentState = BACKTRACKING;
                break;
            }
        }
        return STOP;
    }
    
    MovementCommand executeBacktrack(SensorData sensors) {
        // Simple pathfinding to target position
        int dx = targetPosition.x - currentPosition.x;
        int dy = targetPosition.y - currentPosition.y;
        
        if(abs(dx) < GRID_CELL_SIZE_MM && abs(dy) < GRID_CELL_SIZE_MM) {
            // Reached target
            currentState = FORWARD_SWEEP;
            isBacktracking = false;
            return STOP;
        }
        
        // Determine movement direction
        if(abs(dx) > abs(dy)) {
            // Move in X direction
            if(dx > 0) {
                return turnToDirection(EAST);
            } else {
                return turnToDirection(WEST);
            }
        } else {
            // Move in Y direction
            if(dy > 0) {
                return turnToDirection(NORTH);
            } else {
                return turnToDirection(SOUTH);
            }
        }
    }
    
    MovementCommand executeWallFollow(SensorData sensors) {
        // Right-hand wall following algorithm
        if(!sensors.rightObstacle) {
            // No wall on right, turn right
            return TURN_RIGHT_90;
        } else if(!sensors.frontObstacle) {
            // Wall on right, path ahead clear
            updatePosition(MOVEMENT_INCREMENT_MM);
            return MOVE_FORWARD;
        } else {
            // Wall ahead, turn left
            return TURN_LEFT_90;
        }
    }
    
    // ============= HELPER FUNCTIONS =============
    
    void updateCoverageGrid() {
        int gridX = currentPosition.x / GRID_CELL_SIZE_MM;
        int gridY = currentPosition.y / GRID_CELL_SIZE_MM;
        
        // Mark cells covered by robot footprint
        int widthCells = robotWidthMM / GRID_CELL_SIZE_MM + 1;
        int lengthCells = robotLengthMM / GRID_CELL_SIZE_MM + 1;
        
        for(int dx = -widthCells/2; dx <= widthCells/2; dx++) {
            for(int dy = -lengthCells/2; dy <= lengthCells/2; dy++) {
                int cellX = gridX + dx;
                int cellY = gridY + dy;
                
                if(cellX >= 0 && cellX < GRID_SIZE && cellY >= 0 && cellY < GRID_SIZE) {
                    if(!coverageGrid[cellX][cellY]) {
                        coverageGrid[cellX][cellY] = true;
                        totalCellsCovered++;
                    }
                }
            }
        }
    }
    
    bool shouldContinueSweep() {
        // Check if we have room for another sweep line
        int totalSweepWidth = (currentSweepLine + 1) * sweepLineSpacing;
        return totalSweepWidth < MAP_SIZE_MM;
    }
    
    bool findNearestUnexploredArea(Position& target) {
        int minDistance = INT_MAX;
        bool found = false;
        
        for(int x = 0; x < GRID_SIZE; x++) {
            for(int y = 0; y < GRID_SIZE; y++) {
                if(!coverageGrid[x][y]) {
                    Position pos = {x * GRID_CELL_SIZE_MM, y * GRID_CELL_SIZE_MM};
                    int distance = manhattanDistance(currentPosition, pos);
                    
                    if(distance < minDistance) {
                        minDistance = distance;
                        target = pos;
                        found = true;
                    }
                }
            }
        }
        
        return found;
    }
    
    int manhattanDistance(Position a, Position b) {
        return abs(a.x - b.x) + abs(a.y - b.y);
    }
    
    bool hasUnexploredRooms() {
        for(const auto& room : rooms) {
            if(!room.fullyExplored) {
                return true;
            }
        }
        return false;
    }
    
    Direction chooseBestDirection(std::vector<Direction>& available) {
        // Choose direction that leads to most unexplored area
        Direction best = available[0];
        int maxUnexplored = 0;
        
        for(Direction dir : available) {
            int unexplored = countUnexploredInDirection(dir);
            if(unexplored > maxUnexplored) {
                maxUnexplored = unexplored;
                best = dir;
            }
        }
        
        return best;
    }
    
    int countUnexploredInDirection(Direction dir) {
        // Count unexplored cells in given direction
        int count = 0;
        Position checkPos = currentPosition;
        
        for(int i = 0; i < 10; i++) {  // Look ahead 10 cells
            switch(dir) {
                case NORTH: checkPos.y += GRID_CELL_SIZE_MM; break;
                case SOUTH: checkPos.y -= GRID_CELL_SIZE_MM; break;
                case EAST: checkPos.x += GRID_CELL_SIZE_MM; break;
                case WEST: checkPos.x -= GRID_CELL_SIZE_MM; break;
            }
            
            int gridX = checkPos.x / GRID_CELL_SIZE_MM;
            int gridY = checkPos.y / GRID_CELL_SIZE_MM;
            
            if(gridX >= 0 && gridX < GRID_SIZE && gridY >= 0 && gridY < GRID_SIZE) {
                if(!coverageGrid[gridX][gridY]) {
                    count++;
                }
            }
        }
        
        return count;
    }
    
    MovementCommand turnToDirection(Direction targetDir) {
        int turnCount = (targetDir - heading + 4) % 4;
        
        if(turnCount == 0) {
            updatePosition(MOVEMENT_INCREMENT_MM);
            return MOVE_FORWARD;
        } else if(turnCount == 1) {
            heading = targetDir;
            return TURN_RIGHT_90;
        } else if(turnCount == 3) {
            heading = targetDir;
            return TURN_LEFT_90;
        } else {  // 180 degree turn
            return TURN_RIGHT_90;  // Will need two turns
        }
    }
    
    void updatePosition(int distance) {
        lastPosition = currentPosition;
        
        switch(heading) {
            case NORTH: currentPosition.y += distance; break;
            case SOUTH: currentPosition.y -= distance; break;
            case EAST: currentPosition.x += distance; break;
            case WEST: currentPosition.x -= distance; break;
        }
        
        // Bound checking
        currentPosition.x = std::max(0, std::min(MAP_SIZE_MM, currentPosition.x));
        currentPosition.y = std::max(0, std::min(MAP_SIZE_MM, currentPosition.y));
    }
    
    bool detectStuckCondition() {
        if(currentPosition == lastPosition) {
            stuckCounter++;
            if(stuckCounter > 5) {
                return true;
            }
        } else {
            stuckCounter = 0;
        }
        return false;
    }
    
    MovementCommand handleStuckRecovery(SensorData sensors) {
        static int recoveryStep = 0;
        
        switch(recoveryStep) {
            case 0:
                recoveryStep++;
                return MOVE_BACKWARD;
            case 1:
                recoveryStep++;
                return TURN_RIGHT_90;
            case 2:
                recoveryStep = 0;
                stuckCounter = 0;
                return MOVE_FORWARD;
        }
        
        return STOP;
    }
    
    int mapColorToShape(int colorId) {
        // Map detected color to shape index
        // Customize based on competition rules
        switch(colorId) {
            case 1: return 0;  // Red -> I-shape
            case 2: return 1;  // Blue -> O-shape
            case 3: return 2;  // Green -> J-shape
            case 4: return 3;  // Yellow -> L-shape
            default: return (currentShapeIndex + 1) % 4;  // Cycle through shapes
        }
    }
    
    void updateRobotDimensions(int shapeIndex) {
        // Update robot dimensions based on shape
        // Adjust these values based on your robot's actual dimensions
        switch(shapeIndex) {
            case 0:  // I-shape
                robotWidthMM = 200;
                robotLengthMM = 400;
                break;
            case 1:  // O-shape
                robotWidthMM = 300;
                robotLengthMM = 300;
                break;
            case 2:  // J-shape
                robotWidthMM = 250;
                robotLengthMM = 350;
                break;
            case 3:  // L-shape
                robotWidthMM = 250;
                robotLengthMM = 350;
                break;
        }
    }
    
    // ============= PUBLIC UTILITY FUNCTIONS =============
    
    float getCoveragePercentage() {
        return (float)totalCellsCovered / (float)(GRID_SIZE * GRID_SIZE) * 100.0;
    }
    
    Position getCurrentPosition() {
        return currentPosition;
    }
    
    Direction getCurrentHeading() {
        return heading;
    }
    
    RobotState getCurrentState() {
        return currentState;
    }
    
    void printDebugInfo() {
        Serial.print("State: ");
        Serial.print(getStateName(currentState));
        Serial.print(" | Pos: (");
        Serial.print(currentPosition.x);
        Serial.print(",");
        Serial.print(currentPosition.y);
        Serial.print(") | Heading: ");
        Serial.print(heading);
        Serial.print(" | Coverage: ");
        Serial.print(getCoveragePercentage());
        Serial.println("%");
    }
    
    const char* getStateName(RobotState state) {
        switch(state) {
            case INITIALIZING: return "INIT";
            case FORWARD_SWEEP: return "SWEEP";
            case LATERAL_SHIFT: return "SHIFT";
            case TURNING: return "TURN";
            case OBSTACLE_DETECTED: return "OBSTACLE";
            case COLOR_ZONE_DETECTED: return "COLOR";
            case SHAPE_CHANGING: return "SHAPE";
            case EXPLORING_NEW_ROOM: return "NEW_ROOM";
            case BACKTRACKING: return "BACKTRACK";
            case WALL_FOLLOWING: return "WALL_FOLLOW";
            case COMPLETED: return "DONE";
            default: return "UNKNOWN";
        }
    }
    
    void printCoverageGrid() {
        Serial.println("=== Coverage Grid ===");
        for(int y = GRID_SIZE-1; y >= 0; y--) {
            for(int x = 0; x < GRID_SIZE; x++) {
                Serial.print(coverageGrid[x][y] ? "█" : "·");
            }
            Serial.println();
        }
    }
};

void smorphiStop() {
  my_robot.stopSmorphi();
  my_robot.sm_reset_M1();
  my_robot.sm_reset_M2();
  my_robot.sm_reset_M3();
  my_robot.sm_reset_M4();
}

void executeMovement(MovementCommand command) {
    // INSERT YOUR MOVEMENT CONTROL CODE HERE
    switch(command) {
        case MOVE_FORWARD:
            my_robot.MoveForward(200); // Move forward at speed 200
            delay(1000); // Move for 1 second
            smorphiStop();
            
            break;
            
        case MOVE_BACKWARD:
            my_robot.MoveBackward(200); // Move backward at speed 200
            delay(1000); // Move for 1 second
            smorphiStop();
            break;
            
        case TURN_LEFT_90:
            my_robot.CenterPivotLeft(80); // Turn left at speed 80
            delay(600); // Adjust delay for 90 degree turn
            smorphiStop();
            break;
            
        case TURN_RIGHT_90:
            my_robot.CenterPivotRight(80); // Turn right at speed 80
            delay(600); // Adjust delay for 90 degree turn
            smorphiStop();
            break;
            
        case STOP:
            smorphiStop();
            break;
            
        case LATERAL_MOVE_LEFT:
            my_robot.MoveLeft(200); // Your lateral movement code
            delay(1000); // Move for 1 second
            smorphiStop();
            break;
            
        case LATERAL_MOVE_RIGHT:
            my_robot.MoveRight(200); // Your lateral movement code
            delay(1000);          // Move for 1 second
            smorphiStop();
            break;
    }
}


// INSERT YOUR SENSOR READING FUNCTIONS HERE

bool readFrontIR() {
    return !my_robot.module1_sensor_status(3);
    Serial.println("Front IR: " + String(!my_robot.module1_sensor_status(3)));
}

bool readBackIR() {
    return !my_robot.module3_sensor_status(4);    
    Serial.println("Back IR: " + String(!my_robot.module3_sensor_status(4)));
}

bool readLeftIR() {
    return !my_robot.module4_sensor_status(3);
    Serial.println("Left IR: " + String(!my_robot.module4_sensor_status(3)));
}

bool readRightIR() {
    return !my_robot.module2_sensor_status(3);
    Serial.println("Right IR: " + String(!my_robot.module2_sensor_status(3)));
}

int readHuskyLens() {
    // Your HuskyLens reading code
    // Return 0 if no color detected
    // Return 1-4 for different colors
    return 0;
}


// INSERT YOUR SHAPE CHANGE FUNCTIONS HERE
/*
void changeToIShape() {
    // smorphi.sm_reset_M();
}

void changeToOShape() {
    // smorphi.sm_O_formation();
}

void changeToJShape() {
    // smorphi.sm_J_formation();
}

void changeToLShape() {
    // smorphi.sm_L_formation();
}
*/

// ============= MAIN ARDUINO INTERFACE =============

NavigationSystem navSystem;

void setup() {
    Serial.begin(115200);
    Serial.println("Navigation System Initialized");
    
    // INSERT YOUR HARDWARE INITIALIZATION HERE
    // Initialize Smorphi:
    my_robot.BeginSmorphi();
    
    // Initialize HuskyLens:
    // Wire.begin();
    // huskylens.begin(Wire);
    // huskylens.writeAlgorithm(ALGORITHM_COLOR_RECOGNITION);
    
    // Initialize IR sensors:
    // pinMode(IR_FRONT_PIN, INPUT);
    // etc...
    
    delay(3000);  // Give time to place robot
    Serial.println("Starting navigation...");
}

void loop() {
    // Step 1: Read sensors
    SensorData sensors;
    
    // INSERT YOUR SENSOR READING CODE HERE
    sensors.frontObstacle = readFrontIR();
    sensors.backObstacle = readBackIR();
    sensors.leftObstacle = readLeftIR();
    sensors.rightObstacle = readRightIR();
    sensors.colorDetected = readHuskyLens();
    
    // Step 2: Get navigation command
    MovementCommand command = navSystem.navigate(sensors);
    
    // Step 3: Execute movement command
    executeMovement(command);
    
    // Step 4: Debug output
    static unsigned long lastDebugTime = 0;
    if(millis() - lastDebugTime > 1000) {
        navSystem.printDebugInfo();
        lastDebugTime = millis();
    }
    
    // Small delay for stability
    delay(10);
}



