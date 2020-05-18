// Maximum value that measured distance must flunctuate with, in order to be ruled as the same surface.
#define MAX_DIST_VARIATION 30

const int pingPinEast = 12; // Trigger pin of east-facing Ultrasonic Sensor
const int echoPinEast = 13; // Echo pin of east-facing Ultrasonic Sensor
const int pingPinWest = 8; // Trigger pin of west-facing Ultrasonic Sensor
const int echoPinWest = 9; // Echo pin of west-facing Ultrasonic Sensor

int lastReading = 0;

void setupDistSensors() {
    pinMode(pingPinEast, OUTPUT);
    pinMode(echoPinEast, INPUT);
    pinMode(pingPinWest, OUTPUT);
    pinMode(echoPinWest, INPUT);
}

void resetDistanceReading() {
    lastReading = 0;
}

/**
 * Measure distance from sensor with the given pins.
 * Returns the change from previous reading.
 * 
 * @param pingPin The ping/trigger pin of the distance sensor.
 * @param echoPin The corresponding echo pin.
 * @returns The deviation/change of the distance measurement
 * done by the given sensor, compared to the previous measurement.
 */
int getDistanceDeviation(int pingPin, int echoPin) {
    digitalWrite(pingPin, LOW);
    delayMicroseconds(2);
    digitalWrite(pingPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(pingPin, LOW);

    int duration = pulseIn(echoPin, HIGH);
    int deviation = abs(duration - lastReading);
    lastReading = duration;
    return deviation;
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
bool detectVerticalChange(int horizontalDir) {
    int pingPin = horizontalDir == WEST ? pingPinEast : pingPinWest;
    int echoPin = horizontalDir == WEST ? echoPinEast : echoPinWest;
    int oldReading = lastReading;

    int deviation = getDistanceDeviation(pingPin, echoPin);

    Serial.print("Change of distance at South: ");
    Serial.println(deviation);

    return oldReading && deviation > MAX_DIST_VARIATION;
}

/**
 * Detect change in distance when moving horizontally.
 * This uses the sensor that is facing the way we are traveling.
 * If this method returns true, we should switch directions and move south.
 * 
 * @param direction The horizontal direction (EAST or WEST).
 * @return True if the distance from previous reading has changed substantially.
 */
bool detectHorizontalChange(int direction) {
    int pingPin = direction == EAST ? pingPinEast : pingPinWest;
    int echoPin = direction == EAST ? echoPinEast : echoPinWest;
    int oldReading = lastReading;

    int deviation = getDistanceDeviation(pingPin, echoPin);

    String dir = direction == EAST ? "East" : "West";

    Serial.print("Change of distance at " + dir + ": ");
    Serial.print(deviation);

    return oldReading && deviation > MAX_DIST_VARIATION;
}

/**
 * Indicates whether the distance to the whiteboard has changed since last measure.
 * If it has, we have reached the end of the board and need to change direction
 * (or are done cleaning it). Uses the distance sensor that is facing the direction
 * the robot is currently moving.
 * 
 * @param directions The directions the robot is currently moving (or was moving).
 */
bool distanceChanged(int* directions) {
    int horizontalDir = directions[0];
    int verticalDir = directions[1];
    bool changed = verticalDir == SOUTH || verticalDir == NORTH ?
                   detectVerticalChange(horizontalDir) : detectHorizontalChange(horizontalDir);
    if (changed) {
        resetDistanceReading();
    }
    return changed;
}