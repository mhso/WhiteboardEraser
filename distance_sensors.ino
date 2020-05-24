#include <NewPing.h>

// Maximum value that measured distance must flunctuate with, in order to be ruled as the same surface.
#define MAX_DIST_VARIATION 35

const int pingPinEast = 8; // Trigger pin of east-facing Ultrasonic Sensor
const int echoPinEast = 2; // Echo pin of east-facing Ultrasonic Sensor
const int pingPinWest = 12; // Trigger pin of west-facing Ultrasonic Sensor
const int echoPinWest = 3; // Echo pin of west-facing Ultrasonic Sensor

volatile unsigned long lastReading = 0;
volatile unsigned long deviation = 0;
volatile bool signalReceived = false;

NewPing eastSensor(pingPinEast, echoPinEast, 50);
NewPing westSensor(pingPinWest, echoPinWest, 50);
NewPing* activeSensor;

void resetDistanceReading() {
    lastReading = 0;
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
        // Use the sensor opposite to the previous horziontal direction.
        activeSensor = horizontalDir == WEST ? &eastSensor : &westSensor;
    }
    else {  // Moving horizontally.
        // Use the sensor in the direction of where we are currently moving.
        activeSensor = horizontalDir == EAST ? &eastSensor : &westSensor;
    }
    activeSensor->ping_timer(detectEcho); // Send out a measuring pin in the background.
}

/**
 * Indicates whether the distance to the whiteboard has changed since last measure.
 * If it has, we have reached the end of the board and need to change direction
 * (or are done cleaning it). Uses the distance sensor that is facing the direction
 * the robot is currently moving. This method is called only after newDistanceReading()
 * returns true (i.e. the distance sensor has succesfully done a measurement).
 */
bool distanceChanged() {
    bool changed = deviation > MAX_DIST_VARIATION;
    if (changed) {
        resetDistanceReading();
    }
    String dir = activeSensor == &eastSensor ? "East" : "West";
    Serial.print("Change of distance at " + dir + ": ");
    Serial.println(deviation);
    return changed;
}

/**
 * This method is called periodically by the NewPing library when the active sensor
 * has sent out a ping, and is waiting for an echo.
 */
void detectEcho() {
    if (activeSensor->check_timer()) { // Check if we have received an echo.
        unsigned long duration = activeSensor->ping_result;
        Serial.print("Signal received. Duration: ");
        Serial.println(duration);
        if (lastReading == 0) { // First time we do a distance reading.
            deviation = 0;
        }
        else {
            unsigned long maxDuration = max(duration, lastReading);
            unsigned long minDuration = min(duration, lastReading);
            deviation = maxDuration - minDuration;
        }
        Serial.print("Signal change: ");
        Serial.println(deviation);
        lastReading = duration;
        signalReceived = true;
    }
}

/**
 * This method is called periodically in the main loop, after calls to
 * startDistanceReading() to check whether an echo has been received by
 * the active distance sensor.
 * 
 * @returns Boolean indicating whether the measurement echo has been received.
 */
bool newDistanceReading() {
    if (signalReceived) {
        signalReceived = false;
        return true;
    }
    return false;
}