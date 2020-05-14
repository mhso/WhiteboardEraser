// First motor.
#define IN1 4
#define IN2 5
#define IN3 6
#define IN4 7

// Second motor.
#define IN5 A2
#define IN6 A3
#define IN7 A4
#define IN8 A5

// Amount of revolutions needed to move a leg the full length of the actuator.
#define REVOLUTIONS_PER_CYCLE 10
// Amount of times to move the actuator when moving vertically.
#define MAX_VERT_CYCLES 2

// Motor specific variables (28-BYJ stepper).
const int STEPS_PER_REV = 32;
const int GEAR_RED = 64;
const int MINI_STEPS = STEPS_PER_REV * GEAR_RED;
const int STEPS_PER_CALL = MINI_STEPS / 2;
const int STEPS_PER_CYCLE = MINI_STEPS * REVOLUTIONS_PER_CYCLE;

// Current moving directions.
int horizontalDirection = EAST;
int verticalDirection = STOPPED;
int currentStep = 0;
char currentVertCycles = 0;

int currentSpeed = STEPS_PER_CALL;
int directionArr[2] = {0, 0};

Stepper stepper(STEPS_PER_REV, IN1, IN3, IN2, IN4);
Stepper* activeStepper = &stepper;

void resetActuator() {
    Serial.println("Resetting actuator...");
    setRGBStatus(255, 255, 0); // Status code yellow = we hit the edge.
    int stepsNeeded = currentSpeed < 0 ? STEPS_PER_CYCLE - currentStep : currentStep;
    activeStepper->step(-stepsNeeded);
    Serial.println("Done resetting.");
}

/**
 * Change direction of the robot. This can either be triggered by having moved down
 * vertically a certain distance, or by having hit the edge of the board (as indicated
 * by a distance sensor). If triggered by a sensor, and we are moving vertically, this
 * method will return true (indicating we are done), otherwise return false.
 * 
 * @param triggeredBySensor Whether the change in direction is triggered by a distance sensor.
 * @return Boolean indicating whether we are done cleaning the board.
 */
bool changeDirection(bool triggeredBySensor) {
    Serial.println("Switching directions!");
    if (verticalDirection != STOPPED) {
        if (triggeredBySensor) { // If we are moving vertically and hit the edge of the board, we are done!
            setRGBStatus(255, 0, 0); // Status code red = we are done.
            return true;
        }
        verticalDirection = STOPPED;
        horizontalDirection = horizontalDirection == EAST ? WEST : EAST;
        if (horizontalDirection == WEST) {
            setRGBStatus(0, 255, 255); // Status code cyan = moving west.
            // switchMagnets(MAGNET_HORIZONTAL_ON);
        }
        else {
            setRGBStatus(0, 0, 255); // Status code blue = moving east.
        }
        // Start the horizontal actuator.
        activeStepper = new Stepper(STEPS_PER_REV, IN1, IN3, IN2, IN4);
        activeStepper->setSpeed(1000);
    }
    else {
        // Disable magnets for the horizontal actuator, activate for vertical.
        // switchMagnets(MAGNET_VERTICAL_ON);
        resetActuator(); // Reset the linear actuator to it's initial position.
        // switchMagnets(MAGNET_HORIZONTAL_ON);
        verticalDirection = SOUTH;
        setRGBStatus(255, 128, 0); // Status code orange = moving down.
        // Start the vertical actuator.
        activeStepper = new Stepper(STEPS_PER_REV, IN5, IN7, IN6, IN8);
        activeStepper->setSpeed(1000);
    }
    currentStep = 0;
    currentSpeed = STEPS_PER_CALL;
    delay(1000);
    return false;
}

/**
 * Moves the stepper for the specified amount of steps
 * (default 1024 steps = half a rotation).
 */
void runOnce(int steps=-STEPS_PER_CALL) {
    activeStepper->step(steps);
}

/**
 * Run the active stepper for STEPS_PER_CALL micro-steps. If this stepper has run
 * for STEPS_PER_REV * REVOLUTIONS_PER_CYCLE (i.e. it has moved the current actuator
 * for it's full length), then reverse the active stepper. If the vertical actuator
 * is active (we are moving down/up), and it has moved the actuator fully MAX_VERT_CYCLES
 * amount of times, then the robot swaps directions and runs horizontally.
 * 
 * @return Integer status code indicating which magnets should be switched on/off (if any).
 */
int runStepper() {
    activeStepper->setSpeed(1000);
    if (currentVertCycles == MAX_VERT_CYCLES) {
        // We have moved the desired distance vertically.
        Serial.println("We have moved vertically for long enough!");
        currentStep = 0;
        currentVertCycles = 0;
        resetDistanceReading();
        changeDirection(false);
        delay(1000);
    }
    else if (currentStep >= STEPS_PER_CYCLE) {
        Serial.println("We have moved full length of actuator!");
        // We have moved the full length of the actuator.
        currentStep = 0;
        currentSpeed = -currentSpeed;
        delay(1000);
        if (verticalDirection == STOPPED) { // We are moving horizontally 
            if (currentSpeed > 0) {
                // Stepper moving clockwise. If we are moving east, magnet is off
                // and should be turned on. Opposite if we are moving west.
                return horizontalDirection == EAST ? MAGNET_HORIZONTAL_ON : MAGNET_VERTICAL_ON;
            }
            else {
                // Stepper moving counter-clockwise. If we are moving east, magnet is on
                // and should be turned off. Opposite if we are moving west.
                return horizontalDirection == WEST ? MAGNET_VERTICAL_ON : MAGNET_HORIZONTAL_ON;
            }
        }
        else { // We are moving vertically.
            currentVertCycles++;
            if (currentSpeed > 0) {
                // Stepper moving clockwise. If we are moving north, magnet is off
                // and should be turned on. Opposite if we are moving north.
                return verticalDirection == NORTH ? MAGNET_VERTICAL_ON : MAGNET_HORIZONTAL_ON;
            }
            else {
                // Stepper moving counter-clockwise. If we are moving south, magnet is on
                // and should be turned off. Opposite if we are moving north.
                return verticalDirection == SOUTH ? MAGNET_HORIZONTAL_ON : MAGNET_VERTICAL_ON;
            }
        }
    }
    String dir = "";
    if (verticalDirection != STOPPED) {
        dir = verticalDirection == NORTH ? "North" : "South";
    }
    else {
        dir = horizontalDirection == WEST ? "West" : "East";
    }
    runOnce(currentSpeed); // Move the stepper (blocks until finished).
    currentStep += STEPS_PER_CALL;
    int progress = ((float)currentStep / (float)STEPS_PER_CYCLE) * 100;
    Serial.print("Heading " + dir);
    Serial.print(". Moved ");
    Serial.print(progress);
    Serial.println(" %");
    return 0;
}

void resetStepper() {
    currentStep = 0;
    currentSpeed = STEPS_PER_CALL;
}

int* getDirections() {
    directionArr[0] = horizontalDirection;
    directionArr[1] = verticalDirection;
    return directionArr;
}
