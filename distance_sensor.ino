// Maximum value that measured distance must flunctuate with, in order to be ruled as the same surface.
#define MAX_DIST_VARIATION 35

const int pingPinEast = 8; // Trigger pin of east-facing Ultrasonic Sensor
const int echoPinEast = 2; // Echo pin of east-facing Ultrasonic Sensor
const int pingPinWest = 12; // Trigger pin of west-facing Ultrasonic Sensor
const int echoPinWest = 3; // Echo pin of west-facing Ultrasonic Sensor

volatile unsigned long lastReading = 0;
volatile unsigned long deviation = 0;
volatile int currentPingPin = 0;
volatile int currentEchoPin = 0;
volatile unsigned long pulseStart = 0;
volatile bool signalSent = false;
volatile bool signalReceived = false;

void setupDistSensors() {
    pinMode(pingPinEast, OUTPUT);
    pinMode(echoPinEast, INPUT);
    pinMode(pingPinWest, OUTPUT);
    pinMode(echoPinWest, INPUT);
    Serial.begin(9600);
    attachInterrupt(0, detectHorizontalChange, CHANGE);
    attachInterrupt(1, detectVerticalChange, CHANGE);
}

void resetDistanceReading() {
    lastReading = 0;
}

/**
 * Detect change in distance when moving vertically.
 * This uses the sensor that is opposite the previous horizontal distance.
 * F.x. if we were previously moving east, this uses the west-facing sensor.
 * If this method returns true, it means we are done cleaning the board.
 * 
 * @param horizontalDir The previous horizontal direction.
 * @return True if the distance from previous reading has changed substantially.
 */
void detectVerticalChange() {
    detectChange(currentEchoPin);
}

/**
 * Detect change in distance when moving horizontally.
 * This uses the sensor that is facing the way we are traveling.
 * If this method returns true, we should switch directions and move south.
 * 
 * @param direction The horizontal direction (EAST or WEST).
 * @return True if the distance from previous reading has changed substantially.
 */
void detectHorizontalChange() {
    detectChange(currentEchoPin);
}

/**
 * Start the process of measuring distance to the board.
 * This is called periodically in the main loop of the program.
 * 
 * @param directions The directions the robot is currently moving (or was moving).
 */
void startDistanceReading(int* directions) {
    int horizontalDir = directions[0];
    int verticalDir = directions[1];
    if (verticalDir == SOUTH || verticalDir == NORTH) { // Moving vertically.
        // Use the sensor that is opposite the previous horizontal distance.
        currentPingPin = horizontalDir == WEST ? pingPinEast : pingPinWest;
        currentEchoPin = horizontalDir == WEST ? echoPinEast : echoPinWest;
    }
    else {  // Moving horizontally.
        // Use the sensor in the direction we are travelling.
        currentPingPin = horizontalDir == EAST ? pingPinEast : pingPinWest;
        currentEchoPin = horizontalDir == EAST ? echoPinEast : echoPinWest;
    }
    digitalWrite(currentPingPin, LOW); // Send start of ping pulse.
    delayMicroseconds(2);
    digitalWrite(currentPingPin, HIGH); // Send peak of ping pulse.
    delayMicroseconds(10);
    digitalWrite(currentPingPin, LOW); // Send end of ping pulse.
    signalSent = true;
}

/**
 * Indicates whether the distance to the whiteboard has changed since last measure.
 * If it has, we have reached the end of the board and need to change direction
 * (or are done cleaning it). Uses the distance sensor that is facing the direction
 * the robot is currently moving. This method is called only after pollDistanceSensor()
 * returns true (i.e. the distance sensor has succesfully done a measurement).
 */
bool distanceChanged() {
    bool changed = deviation > MAX_DIST_VARIATION;
    if (changed) {
        resetDistanceReading();
    }
    String dir = currentPingPin == pingPinEast ? "East" : "West";
    Serial.print("Change of distance at " + dir + ": ");
    Serial.println(deviation);
    return changed;
}

/**
 * This method is called from an Arduino interrupt on either pin 2 or 3.
 * It is called whenenver these pins receive a change of voltage, which occurs
 * when the sensor has received it's own echo.
 * 
 * @param echoPin The echo pin of the distance sensor 2 for east-facing sensor,
 *                3 for the west-facing one).
 */
void detectChange(int echoPin) {
    if (signalSent) {
        if (digitalRead(echoPin) == HIGH) {
            pulseStart = micros();
        }
        else if (!signalReceived) {
            unsigned long duration = micros() - pulseStart;
            if (lastReading == 0) {
                deviation = 0;
            }
            else {
                unsigned long maxDuration = max(duration, lastReading);
                unsigned long minDuration = min(duration, lastReading);
                deviation = maxDuration - minDuration;
            }
            lastReading = duration;
            signalReceived = true;
        }
    }
}

/**
 * This method is called periodically in the main loop, between calls
 * to startDistanceReading() and distanceChanged(). It sends out ping
 * signals to the activate distance sensor at certain intervals and returns
 * true when a full pulse has been sent.
 * 
 * @returns Boolean indicating whether the measurement pulse has been sent out.
 */
bool newDistanceReading() {
    if (signalReceived) {
        signalReceived = false;
        signalSent = false;
        return true;
    }
    return false;
}