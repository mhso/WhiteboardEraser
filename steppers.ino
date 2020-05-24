// First motor.
#define IN1 9
#define IN2 4
#define IN3 13
#define IN4 A1

// Second motor.
#define IN5 A5
#define IN6 A4
#define IN7 A3
#define IN8 A2

// Amount of revolutions needed to move a leg the full length of the actuator.
#define REVOLUTIONS_PER_CYCLE 10
// Amount of times to move the actuator when moving vertically.
#define ACCELERATION 50

// Motor specific variables (28-BYJ stepper).
const int STEPS_PER_REV = 32;
const int GEAR_RED = 64;
const int MINI_STEPS = STEPS_PER_REV * GEAR_RED;

const int STEPS_PER_CALL = MINI_STEPS / 2;
const int STEPS_PER_CYCLE = MINI_STEPS * REVOLUTIONS_PER_CYCLE;
const int MAX_VERT_STEPS = 2;
const int TIMES_TO_MEASURE_DISTANCE = 4;
const int MEASURE_AT_STEP = STEPS_PER_CYCLE / TIMES_TO_MEASURE_DISTANCE;

// Current moving directions.
int horizontalDirection = EAST;
int verticalDirection = STOPPED;
int currentStep = 0;
int currentVertSteps = 0;
long currentHorizontalPosition = 0;
long totalHorizontalDistance = 0;

// Distance measurement things.
int timesMeasuredDist = 1;
// Boolean that is set to true when we have hit the bottom edge of the board.
bool lastHorizontalCycle = false;

int currentSpeed = STEPS_PER_CYCLE;
int directionArr[2] = {horizontalDirection, verticalDirection};

AccelStepper* activeStepper;

/**
 * Helper method for resetting the two stepper motors to their initial position.
 */
void resetSteppers() {
    int horizontalRot = 0;
    int verticalRot = 0;
    if (horizontalRot != 0) {
        AccelStepper stepper(AccelStepper::FULL4WIRE, IN1, IN3, IN2, IN4);
        stepper.setMaxSpeed(600);
        stepper.setAcceleration(50);
        stepper.runToNewPosition(MINI_STEPS * horizontalRot);
    }
    if (verticalRot != 0) {
        AccelStepper stepper(AccelStepper::FULL4WIRE, IN5, IN7, IN6, IN8);
        stepper.setMaxSpeed(500);
        stepper.setAcceleration(50);
        stepper.runToNewPosition(MINI_STEPS * verticalRot);
    }
}

/**
 * This method creates an AccelStepper object and starts the stepper
 * corresponding to the given direction.
 * 
 * @param horizontal Boolean indicating whether the stepper to starts
 * should be the horizontal or the vertical one.
 */
void startStepper(bool horizontal) {
    int speed;
    if (horizontal) {
        activeStepper = new AccelStepper(AccelStepper::FULL4WIRE, IN1, IN3, IN2, IN4);
        speed = 600;
    }
    else {
        activeStepper = new AccelStepper(AccelStepper::FULL4WIRE, IN5, IN7, IN6, IN8);
        speed = 500;
    }
    activeStepper->setMaxSpeed(speed);
    activeStepper->setAcceleration(ACCELERATION);
    activeStepper->moveTo(currentSpeed);
}

void resetActuator(int position) {
    Serial.println("Resetting actuator...");
    setRGBStatus(255, 255, 0); // Status code yellow = we hit the edge.
    int stepsNeeded = currentSpeed < 0 ? STEPS_PER_CYCLE - position : -position;
    Serial.print("Running: ");
    Serial.print(stepsNeeded);
    Serial.println(" steps.");
    runOnce(stepsNeeded);
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
    Serial.print("Switching directions ");
    if (triggeredBySensor) {
        Serial.println("because of sensor!");
    }
    else {
        Serial.println("because we have moved far enough vertically.");
    }
    currentSpeed = STEPS_PER_CYCLE;
    timesMeasuredDist = 1;
    resetDistanceReading();
    if (verticalDirection != STOPPED) {
        if (triggeredBySensor) { // We have hit the bottom of the board.
            lastHorizontalCycle = true;
        }
        verticalDirection = STOPPED;
        horizontalDirection = horizontalDirection == EAST ? WEST : EAST;
        currentHorizontalPosition = 0;
        if (horizontalDirection == WEST) {
            setRGBStatus(0, 255, 255); // Status code cyan = moving west.
            switchMagnets(MAGNET_HORIZONTAL_ON);
        }
        else {
            setRGBStatus(0, 0, 255); // Status code blue = moving east.
        }
        // Stop the vertical actuator.
        activeStepper->stop();
        activeStepper->runToPosition();
        // Start the horizontal actuator.
        startStepper(true);
    }
    else {
        if (totalHorizontalDistance == 0) { // We our on our first horizontal cycle.
            totalHorizontalDistance = currentHorizontalPosition + abs(activeStepper->currentPosition());
        }
        if (lastHorizontalCycle) { // If we have hit the bottom corner of the board, we are done!
            setRGBStatus(255, 0, 0); // Status code red = we are done.
            return true;
        }
        // Stop the horizontal actuator.
        int position = activeStepper->currentPosition();
        activeStepper->stop();
        activeStepper->runToPosition();
        // Disable magnets for the horizontal actuator, activate for vertical.
        switchMagnets(MAGNET_VERTICAL_ON);
        resetActuator(position); // Reset the linear actuator to it's initial position.
        switchMagnets(MAGNET_HORIZONTAL_ON);
        verticalDirection = SOUTH;
        setRGBStatus(255, 128, 0); // Status code orange = moving down.
        // Start the vertical actuator.
        startStepper(false);
    }
    return false;
}

/**
 * Moves the stepper for the specified amount of steps.
 */
void runOnce(int steps) {
    activeStepper->setCurrentPosition(0);
    activeStepper->setMaxSpeed(600);
    activeStepper->runToNewPosition(steps);
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
    if ((verticalDirection != STOPPED || totalHorizontalDistance == 0)
            && (timesMeasuredDist < TIMES_TO_MEASURE_DISTANCE
            && abs(activeStepper->currentPosition()) >= MEASURE_AT_STEP * timesMeasuredDist)) {
        // We are moving vertically or it is the first time we are moving horizontally,
        // AND we have moved far enough to measure distance.
        timesMeasuredDist++;
        return MEASURE_DISTANCE;
    }
    if (currentVertSteps == MAX_VERT_STEPS) {
        // We have moved the desired distance vertically.
        Serial.println("We have moved vertically for long enough!");
        currentVertSteps = 0;
        resetDistanceReading();
        changeDirection(false);
    }
    else if (currentHorizontalPosition >= totalHorizontalDistance) {
        changeDirection(false);
    }
    else if (activeStepper->distanceToGo() == 0) {
        Serial.println("We have moved full length of actuator!");
        // We have moved the full length of the actuator.
        timesMeasuredDist = 1;
        currentSpeed = -currentSpeed;
        int newPos = -activeStepper->currentPosition();
        activeStepper->setCurrentPosition(0);
        activeStepper->setMaxSpeed(verticalDirection == STOPPED ? 600 : 500);
        activeStepper->moveTo(newPos);
        resetDistanceReading();
        if (verticalDirection == STOPPED) { // We are moving horizontally
            currentHorizontalPosition += abs(activeStepper->currentPosition());
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
            currentVertSteps++;
            Serial.print("Vertical steps: ");
            Serial.println(currentVertSteps);
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
    activeStepper->run(); // Move the stepper a single step.
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
