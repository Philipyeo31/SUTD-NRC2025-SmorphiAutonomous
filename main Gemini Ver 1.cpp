/*
 * Smorphi 2 Autonomous Navigation Algorithm
 * National Robotics Challenge - Area Coverage Logic (Modified)
 *
 * This code provides the navigation algorithm and logic.
 * Hardware control functions should be implemented separately.
 *
 * Key Modifications in this version:
 * 1.  SWEEPING BEHAVIOR: A new 'enableSweeping' flag allows you to easily turn the systematic
 * "lawn-mower" coverage on or off. If off, the robot defaults to wall-following.
 * 2.  LATERAL MOVEMENT PRIORITY: The 'LATERAL_SHIFT' state has been rewritten to use a direct
 * lateral (sideways) move instead of a multi-step turn-and-move sequence.
 * 3.  ENHANCED COMMENTS: Added detailed comments throughout the NavigationSystem class
 * to explain the logic, state transitions, and decision-making processes.
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
#define MAP_SIZE_MM 3000          // The width/length of the 3m x 3m map.
#define GRID_CELL_SIZE_MM 50      // The resolution of our internal map. Smaller values are more accurate but use more memory.
#define GRID_SIZE (MAP_SIZE_MM / GRID_CELL_SIZE_MM)
#define MOVEMENT_INCREMENT_MM 50  // The distance the robot attempts to move in a single step. Should be the same as GRID_CELL_SIZE_MM.
#define SWEEP_OVERLAP_MM 20       // How much each sweep line should overlap to avoid gaps.

// ============= DATA STRUCTURES =============

// Defines the possible states the robot can be in. This controls its behavior in the main loop.
enum RobotState {
    INITIALIZING,         // Setting up the robot.
    FORWARD_SWEEP,        // Moving forward in a straight line (part of the lawn-mower pattern).
    LATERAL_SHIFT,        // Moving sideways to the next line in the lawn-mower pattern.
    TURNING,              // Performing a turn (used less frequently now).
    OBSTACLE_DETECTED,    // An obstacle has been found, and a decision needs to be made.
    COLOR_ZONE_DETECTED,  // The HuskyLens has detected a colored area for a shape change.
    SHAPE_CHANGING,       // Actively performing a shape transformation.
    EXPLORING_NEW_ROOM,   // Logic to handle transitioning to a new, unexplored room.
    BACKTRACKING,         // Navigating to a previously known, unexplored point on the map.
    WALL_FOLLOWING,       // An alternative exploration strategy to sweeping.
    COMPLETED             // The robot believes it has covered the entire map.
};

// Represents the four cardinal directions for navigation and heading.
enum Direction {
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3
};

// The set of specific movement commands the navigation logic can issue.
enum MovementCommand {
    MOVE_FORWARD,
    MOVE_BACKWARD,
    TURN_LEFT_90,
    TURN_RIGHT_90,
    STOP,
    LATERAL_MOVE_LEFT,
    LATERAL_MOVE_RIGHT
};

// A container for all sensor readings taken in a single loop.
struct SensorData {
    bool frontObstacle;
    bool backObstacle;
    bool leftObstacle;
    bool rightObstacle;
    int colorDetected;  // 0 = no color, 1-4 = color IDs
};

// Represents a coordinate on our internal map.
struct Position {
    int x;
    int y;
    
    bool operator==(const Position& other) const {
        return x == other.x && y == other.y;
    }
};

// Stores information about each discovered room.
struct Room {
    int id;
    Position entryPoint;
    bool fullyExplored;
};

// ============= MAIN NAVIGATION CLASS =============

class NavigationSystem {
private:
    // Robot state variables
    RobotState currentState;
    RobotState previousState;
    Direction heading;
    Position currentPosition;
    Position targetPosition;
    
    // Shape management
    int currentShapeIndex; // 0: I-shape, 1: O-shape, etc.
    int robotWidthMM;      // Current robot width, updated after each shape change.
    int robotLengthMM;     // Current robot length.
    
    // Coverage tracking
    bool coverageGrid[GRID_SIZE][GRID_SIZE]; // A 2D array representing the map, true means "covered".
    int totalCellsCovered;
    
    // Room management
    std::vector<Room> rooms;
    int currentRoomId;
    
    // Sweep parameters
    int sweepDirection;      // 1 for forward (North), -1 for backward (South).
    bool sweepComplete;
    
    // Navigation flags and counters
    bool isBacktracking;
    bool colorZoneProcessed;
    int stuckCounter;
    Position lastPosition;

    // *** NEW: CONTROL FLAG FOR SWEEPING BEHAVIOR ***
    bool enableSweeping; // Set to 'true' for lawn-mower pattern, 'false' for wall-following.
    
public:
    NavigationSystem() {
        currentState = INITIALIZING;
        heading = NORTH;
        currentPosition = {GRID_CELL_SIZE_MM, GRID_CELL_SIZE_MM}; // Start slightly off the corner.
        lastPosition = {0,0};
        currentShapeIndex = 0;   // Default to I-shape
        robotWidthMM = 200;      // Default I-shape width
        robotLengthMM = 400;     // Default I-shape length
        totalCellsCovered = 0;
        sweepDirection = 1;      // Start by moving forward.
        sweepComplete = false;
        isBacktracking = false;
        colorZoneProcessed = false;
        stuckCounter = 0;
        currentRoomId = 0;

        // *** TOGGLE SWEEPING STRATEGY HERE ***
        enableSweeping = false; // <-- SET TO 'false' TO USE WALL-FOLLOWING INSTEAD OF SWEEPING.

        // Initialize the coverage grid to all 'false' (uncovered).
        for(int i = 0; i < GRID_SIZE; i++) {
            for(int j = 0; j < GRID_SIZE; j++) {
                coverageGrid[i][j] = false;
            }
        }
        
        // Create the first room record.
        Room firstRoom;
        firstRoom.id = 0;
        firstRoom.entryPoint = currentPosition;
        firstRoom.fullyExplored = false;
        rooms.push_back(firstRoom);
    }
    
    // ============= DEBUGGING VISUALIZATION =============
    void printDebugMap(const SensorData& sensors) {
        Serial.println("\n=== Robot Navigation Debug Map ===");
        /*
        // Print column numbers
        Serial.print("   ");
        for(int x = 0; x < GRID_SIZE; x++) {
            Serial.print(x % 10);
            Serial.print(" ");
        }
        Serial.println();
        
        // Print the grid
        for(int y = GRID_SIZE-1; y >= 0; y--) {
            // Print row numbers
            Serial.print(y % 10);
            Serial.print("  ");
            
            for(int x = 0; x < GRID_SIZE; x++) {
                char cellChar = '.';  // Default: uncovered space
                
                // Check if this is the robot's position
                if(x == currentPosition.x / GRID_CELL_SIZE_MM && y == currentPosition.y / GRID_CELL_SIZE_MM) {
                    // Show robot's heading with arrows ↑→↓←
                    switch(heading) {
                        case NORTH: cellChar = '^'; break;
                        case EAST:  cellChar = '>'; break;
                        case SOUTH: cellChar = 'v'; break;
                        case WEST:  cellChar = '<'; break;
                    }
                }
                // Show covered areas
                else if(coverageGrid[x][y]) {
                    cellChar = '#';  // Covered area
                }
                
                Serial.print(cellChar);
                Serial.print(" ");
            }
            Serial.println();
        }
            */
        
        // Print sensor status
        Serial.println("\nSensor Status:");
        Serial.print("Front: "); Serial.println(sensors.frontObstacle ? "BLOCKED" : "Clear");
        Serial.print("Back:  "); Serial.println(sensors.backObstacle ? "BLOCKED" : "Clear");
        Serial.print("Left:  "); Serial.println(sensors.leftObstacle ? "BLOCKED" : "Clear");
        Serial.print("Right: "); Serial.println(sensors.rightObstacle ? "BLOCKED" : "Clear");
        
        // Print current state
        Serial.print("\nCurrent State: ");
        switch(currentState) {
            case INITIALIZING: Serial.println("INITIALIZING"); break;
            case FORWARD_SWEEP: Serial.println("FORWARD_SWEEP"); break;
            case LATERAL_SHIFT: Serial.println("LATERAL_SHIFT"); break;
            case TURNING: Serial.println("TURNING"); break;
            case OBSTACLE_DETECTED: Serial.println("OBSTACLE_DETECTED"); break;
            case COLOR_ZONE_DETECTED: Serial.println("COLOR_ZONE_DETECTED"); break;
            case SHAPE_CHANGING: Serial.println("SHAPE_CHANGING"); break;
            case EXPLORING_NEW_ROOM: Serial.println("EXPLORING_NEW_ROOM"); break;
            case BACKTRACKING: Serial.println("BACKTRACKING"); break;
            case WALL_FOLLOWING: Serial.println("WALL_FOLLOWING"); break;
            case COMPLETED: Serial.println("COMPLETED"); break;
        }
        
        Serial.print("Position: (");
        Serial.print(currentPosition.x);
        Serial.print(", ");
        Serial.print(currentPosition.y);
        Serial.println(")");
        
        Serial.println("==============================\n");
    }

    // ============= MAIN NAVIGATION FUNCTION (THE "BRAIN") =============
    
    MovementCommand navigate(SensorData sensors) {
        // Print debug visualization at start of navigation
        printDebugMap(sensors);
        // This is the primary decision-making function, called on every loop.
        
        // First, always update our internal map with the robot's current position.
        updateCoverageGrid();
        
        // Second, check if the robot is physically stuck and try to recover.
        if(detectStuckCondition()) {
            return handleStuckRecovery(sensors);
        }
        
        // Third, prioritize reacting to color zones for shape changes.
        if(sensors.colorDetected > 0 && !colorZoneProcessed) {
            return handleColorZoneDetection(sensors.colorDetected);
        }
        
        // Finally, execute the logic for the current state.
        switch(currentState) {
            case INITIALIZING:
                return initializeNavigation();
            
            case FORWARD_SWEEP:
                return executeForwardSweep(sensors);
            
            case LATERAL_SHIFT:
                return executeLateralShift(sensors);
            
            case OBSTACLE_DETECTED:
                return handleObstacle(sensors);
                
            case COLOR_ZONE_DETECTED:
                return processColorZone(sensors.colorDetected);
                
            case SHAPE_CHANGING:
                // This state just waits for the shape change to complete.
                // Your physical shape change function should transition the state back to FORWARD_SWEEP or WALL_FOLLOWING.
                return STOP;
                
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
        // This function runs once at the beginning.
        // It decides which exploration strategy to start with based on the 'enableSweeping' flag.
        if (enableSweeping) {
            currentState = FORWARD_SWEEP;
        } else {
            currentState = WALL_FOLLOWING;
        }
        return STOP; // Return STOP to give it a moment before the first move.
    }
    
    MovementCommand executeForwardSweep(SensorData sensors) {
        // Handles moving the robot in a straight line during the sweep.
        
        // Check if there's an obstacle in the direction of our sweep.
        bool obstacleAhead = sensors.frontObstacle;
        
        if(obstacleAhead) {
            // We've hit a wall or obstacle at the end of the line.
            // Reverse the sweep direction for the return path.
            sweepDirection *= -1;
            // Transition to the LATERAL_SHIFT state to move to the next line.
            currentState = LATERAL_SHIFT;
            return STOP;
        }
        
        // If the path is clear, continue moving forward.
        updatePosition(MOVE_FORWARD);
        return MOVE_FORWARD;
    }
    
    MovementCommand executeLateralShift(SensorData sensors) {
        // *** MODIFIED LOGIC ***
        // This function now uses a direct lateral move instead of turning.
        
        // Check if there's an obstacle to the side, preventing the shift.
        if (sensors.rightObstacle) {
            // Can't shift, the area is blocked. Mark the sweep as complete for this room.
            sweepComplete = true;
            // Find the nearest unexplored spot to go to next.
            Position nextTarget;
            if (findNearestUnexploredArea(nextTarget)) {
                targetPosition = nextTarget;
                currentState = BACKTRACKING;
            } else {
                currentState = COMPLETED; // No unexplored areas left.
            }
            return STOP;
        }
        
        // If the path is clear for a lateral shift:
        // Update our internal position first.
        updatePosition(LATERAL_MOVE_RIGHT);
        
        // Transition back to sweeping for the return journey.
        currentState = FORWARD_SWEEP;
        
        // Issue the command for a physical lateral move.
        return LATERAL_MOVE_RIGHT;
    }
    
    MovementCommand handleObstacle(SensorData sensors) {
        // This is a general-purpose obstacle avoidance handler.
        // If sweeping is enabled, it might try to divert to wall-following temporarily.
        if (enableSweeping) {
            // If we hit an obstacle mid-sweep, it's complex. Default to wall-following to get around it.
            currentState = WALL_FOLLOWING;
            return executeWallFollow(sensors);
        } else {
            // If we are already wall-following, the logic within executeWallFollow is sufficient.
            return executeWallFollow(sensors);
        }
    }
    
    MovementCommand handleColorZoneDetection(int colorId) {
        // Called immediately when a color is detected.
        currentState = COLOR_ZONE_DETECTED; // Change state to process it.
        colorZoneProcessed = true; // Set a flag so we don't trigger this repeatedly on the same color patch.
        return STOP; // Stop all movement to handle the zone.
    }
    
    MovementCommand processColorZone(int colorId) {
        // Logic for what to do once stopped on a color zone.
        int newShape = mapColorToShape(colorId);
        
        if(newShape != currentShapeIndex) {
            currentShapeIndex = newShape;
            currentState = SHAPE_CHANGING;
            
            // IMPORTANT: Update internal dimensions to match the new shape for accurate coverage tracking.
            updateRobotDimensions(newShape);
            
            // Assume a color zone means entering a new room.
            currentRoomId++;
            Room newRoom;
            newRoom.id = currentRoomId;
            newRoom.entryPoint = currentPosition;
            newRoom.fullyExplored = false;
            rooms.push_back(newRoom);
            
            // INSERT YOUR SHAPE CHANGE TRIGGER CODE HERE.
            // Your function should be non-blocking or handle the state change back to
            // FORWARD_SWEEP or WALL_FOLLOWING itself upon completion.
            
        } else {
            // We're already in the correct shape, so just continue.
            currentState = enableSweeping ? FORWARD_SWEEP : WALL_FOLLOWING;
        }
        
        return STOP;
    }

    MovementCommand executeBacktrack(SensorData sensors) {
        // Handles navigation to a specific target coordinate (e.g., an unexplored area).
        // This is a simplified pathfinding algorithm.
        int dx = targetPosition.x - currentPosition.x;
        int dy = targetPosition.y - currentPosition.y;
        
        // Check if we have arrived at the target.
        if(abs(dx) < GRID_CELL_SIZE_MM && abs(dy) < GRID_CELL_SIZE_MM) {
            isBacktracking = false;
            currentState = enableSweeping ? FORWARD_SWEEP : WALL_FOLLOWING; // Resume normal operation.
            return STOP;
        }
        
        // Decide whether to move along the X or Y axis first.
        if(abs(dx) > abs(dy)) {
            // Move horizontally.
            Direction targetHeading = (dx > 0) ? EAST : WEST;
            if (heading != targetHeading) return turnToDirection(targetHeading);
            else {
                updatePosition(MOVE_FORWARD);
                return MOVE_FORWARD;
            }
        } else {
            // Move vertically.
            Direction targetHeading = (dy > 0) ? NORTH : SOUTH;
            if (heading != targetHeading) return turnToDirection(targetHeading);
            else {
                updatePosition(MOVE_FORWARD);
                return MOVE_FORWARD;
            }
        }
    }
    
    MovementCommand executeWallFollow(SensorData sensors) {
        // A simple "right-hand rule" wall-following algorithm.
        // It keeps a wall to its right at all times.
        if(!sensors.rightObstacle) {
            // If there's no wall on the right, turn right to find it.
            updatePosition(TURN_RIGHT_90);
            return TURN_RIGHT_90;
        } else if(!sensors.frontObstacle) {
            // If there's a wall on the right and the front is clear, move forward.
            updatePosition(MOVE_FORWARD);
            return MOVE_FORWARD;
        } else {
            // If wall is on the right and in front, turn left.
            updatePosition(TURN_LEFT_90);
            return TURN_LEFT_90;
        }
    }
    
    // ============= HELPER & UTILITY FUNCTIONS =============
    
    void updateCoverageGrid() {
        int gridX = currentPosition.x / GRID_CELL_SIZE_MM;
        int gridY = currentPosition.y / GRID_CELL_SIZE_MM;
        
        if (gridX >= 0 && gridX < GRID_SIZE && gridY >= 0 && gridY < GRID_SIZE) {
            if (!coverageGrid[gridX][gridY]) {
                coverageGrid[gridX][gridY] = true;
                totalCellsCovered++;
            }
        }
    }
    
    bool findNearestUnexploredArea(Position& target) {
        int minDistance = 999999;
        bool found = false;
        
        for(int x = 0; x < GRID_SIZE; x++) {
            for(int y = 0; y < GRID_SIZE; y++) {
                if(!coverageGrid[x][y]) {
                    Position pos = {x * GRID_CELL_SIZE_MM, y * GRID_CELL_SIZE_MM};
                    int distance = abs(currentPosition.x - pos.x) + abs(currentPosition.y - pos.y); // Manhattan distance
                    
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

    MovementCommand turnToDirection(Direction targetDir) {
        int turnCount = (targetDir - heading + 4) % 4;
        
        if (turnCount == 1) { // 90-degree right turn
            heading = targetDir;
            return TURN_RIGHT_90;
        } else if (turnCount == 3) { // 90-degree left turn
            heading = targetDir;
            return TURN_LEFT_90;
        } else if (turnCount == 2) { // 180-degree turn
            heading = static_cast<Direction>((heading + 1) % 4); // Turn right once
            return TURN_RIGHT_90; // The logic will call this again next loop to complete the 180.
        }
        return MOVE_FORWARD; // Already facing the right way.
    }
    
    void updatePosition(MovementCommand lastMove) {
        lastPosition = currentPosition;
        int distance = MOVEMENT_INCREMENT_MM;
        
        // This function updates the robot's INTERNAL coordinates, not its physical location.
        switch(lastMove) {
            case MOVE_FORWARD:
                if (heading == NORTH) currentPosition.y += distance * sweepDirection;
                if (heading == SOUTH) currentPosition.y -= distance * sweepDirection;
                if (heading == EAST)  currentPosition.x += distance * sweepDirection;
                if (heading == WEST)  currentPosition.x -= distance * sweepDirection;
                break;
            case LATERAL_MOVE_RIGHT:
                if (heading == NORTH || heading == SOUTH) currentPosition.x += distance;
                if (heading == EAST || heading == WEST) currentPosition.y -= distance;
                break;
            case LATERAL_MOVE_LEFT:
                 if (heading == NORTH || heading == SOUTH) currentPosition.x -= distance;
                 if (heading == EAST || heading == WEST) currentPosition.y += distance;
                break;
            case TURN_LEFT_90:
                heading = static_cast<Direction>((heading - 1 + 4) % 4);
                break;
            case TURN_RIGHT_90:
                heading = static_cast<Direction>((heading + 1) % 4);
                break;
            default:
                break; // No position change for other commands.
        }
        
        // Clamp position to be within map boundaries.
        currentPosition.x = std::max(0, std::min(MAP_SIZE_MM, currentPosition.x));
        currentPosition.y = std::max(0, std::min(MAP_SIZE_MM, currentPosition.y));
    }
    
    bool detectStuckCondition() {
        if(currentPosition == lastPosition) {
            stuckCounter++;
            if(stuckCounter > 10) { // Increased threshold for more tolerance
                return true;
            }
        } else {
            stuckCounter = 0;
        }
        return false;
    }
    
    MovementCommand handleStuckRecovery(SensorData sensors) {
        stuckCounter = 0; // Reset counter
        // Simple recovery: back up and turn right.
        currentState = WALL_FOLLOWING; // Switch to a different strategy to get unstuck.
        return MOVE_BACKWARD; 
    }
    
    int mapColorToShape(int colorId) {
        // Customize this mapping based on competition day rules.
        switch(colorId) {
            case 1: return 0; // Red -> I-shape
            case 2: return 1; // Blue -> O-shape
            case 3: return 2; // Green -> J-shape
            case 4: return 3; // Yellow -> L-shape
            default: return (currentShapeIndex + 1) % 4; // Cycle if color is unknown.
        }
    }
    
    void updateRobotDimensions(int shapeIndex) {
        // Update robot dimensions based on shape.
        // Adjust these values based on your robot's actual dimensions
        switch(shapeIndex) {
            case 0:  // I-shape
                robotWidthMM = 200; robotLengthMM = 400; break;
            case 1:  // O-shape
                robotWidthMM = 300; robotLengthMM = 300; break;
            case 2:  // J-shape
                robotWidthMM = 250; robotLengthMM = 350; break;
            case 3:  // L-shape
                robotWidthMM = 250; robotLengthMM = 350; break;
        }
    }
    
    // Public utility and debug functions
    float getCoveragePercentage() {
        return (float)totalCellsCovered / (float)(GRID_SIZE * GRID_SIZE) * 100.0;
    }
    
    void printDebugInfo() {
        Serial.print("State: "); Serial.print(getStateName(currentState));
        Serial.print(" | Pos: ("); Serial.print(currentPosition.x); Serial.print(","); Serial.print(currentPosition.y);
        Serial.print(") | H: "); Serial.print(heading);
        Serial.print(" | Cov: "); Serial.print(getCoveragePercentage()); Serial.println("%");
    }
    
    const char* getStateName(RobotState state) {
        switch(state) {
            case INITIALIZING: return "INIT"; case FORWARD_SWEEP: return "SWEEP";
            case LATERAL_SHIFT: return "SHIFT"; case TURNING: return "TURN";
            case OBSTACLE_DETECTED: return "OBSTACLE"; case COLOR_ZONE_DETECTED: return "COLOR";
            case SHAPE_CHANGING: return "SHAPE"; case EXPLORING_NEW_ROOM: return "NEW_ROOM";
            case BACKTRACKING: return "BACKTRACK"; case WALL_FOLLOWING: return "WALL_FOLLOW";
            case COMPLETED: return "DONE"; default: return "UNKNOWN";
        }
    }
};

// ============= ARDUINO-SPECIFIC IMPLEMENTATION =============

NavigationSystem navSystem;

// --- Your Hardware-Specific Functions Below ---

void smorphiStop() {
    my_robot.stopSmorphi();
    my_robot.sm_reset_M1(); my_robot.sm_reset_M2();
    my_robot.sm_reset_M3(); my_robot.sm_reset_M4();
}

void executeMovement(MovementCommand command) {
    switch(command) {
        case MOVE_FORWARD:
            my_robot.MoveForward(200); delay(250); smorphiStop(); break;
        case MOVE_BACKWARD:
            my_robot.MoveBackward(200); delay(250); smorphiStop(); break;
        case TURN_LEFT_90:
            my_robot.CenterPivotLeft(80); delay(600); smorphiStop(); break;
        case TURN_RIGHT_90:
            my_robot.CenterPivotRight(80); delay(600); smorphiStop(); break;
        case LATERAL_MOVE_LEFT:
            my_robot.MoveLeft(200); delay(500); smorphiStop(); break;
        case LATERAL_MOVE_RIGHT:
            my_robot.MoveRight(200); delay(500); smorphiStop(); break;
        case STOP:
            smorphiStop(); break;
    }
}

bool readFrontIR() { return !my_robot.module1_sensor_status(3); }
bool readBackIR() { return !my_robot.module3_sensor_status(4); }
bool readLeftIR() { return !my_robot.module4_sensor_status(3); }
bool readRightIR() { return !my_robot.module2_sensor_status(3); }
int readHuskyLens() { return 0; /* Placeholder */ }

void setup() {
    Serial.begin(115200);
    Serial.println("Navigation System Initialized");
    my_robot.BeginSmorphi();
    delay(3000);
    Serial.println("Starting navigation...");
}

void loop() {
    // 1. Read all sensors once.
    SensorData sensors;
    sensors.frontObstacle = readFrontIR();
    sensors.backObstacle = readBackIR();
    sensors.leftObstacle = readLeftIR();
    sensors.rightObstacle = readRightIR();
    sensors.colorDetected = readHuskyLens();
    
    // 2. Get the next logical command from the navigation system.
    MovementCommand command = navSystem.navigate(sensors);
    
    // 3. Execute the physical command.
    executeMovement(command);
    
    // 4. Print debug info periodically.
    static unsigned long lastDebugTime = 0;
    if(millis() - lastDebugTime > 500) {
        navSystem.printDebugInfo();
        lastDebugTime = millis();
    }
    
    delay(10);
}
