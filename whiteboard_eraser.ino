#include <AccelStepper.h>

// Direction flags for seeing where the robot is heading.
#define STOPPED 0
#define WEST 1
#define EAST 2
#define NORTH 3
#define SOUTH 4

// Status codes for indicating if vertical/horizontal magnets should turn on/off. 
#define MAGNET_HORIZONTAL_ON 1
#define MAGNET_VERTICAL_ON 2
#define MEASURE_DISTANCE 3

// Start switch pin.
#define START_SWITCH 7

// Pins for RGB LED
#define RED_STATUS_PIN 5
#define GREEN_STATUS_PIN 10
#define BLUE_STATUS_PIN 11

// --- RGB colors statuses ---
// Green: Nothing is going on.
// Blue: Robot is moving east.
// Cyan: Robot is moving west.
// Orange: Robot is moving south.
// Yellow: Robot has reached any edge of the board.

void setRGBStatus(int r, int g, int b) {
    analogWrite(RED_STATUS_PIN, r);
    analogWrite(GREEN_STATUS_PIN, g);
    analogWrite(BLUE_STATUS_PIN, b);
}

void setup() {
    pinMode(RED_STATUS_PIN, OUTPUT);
    pinMode(GREEN_STATUS_PIN, OUTPUT);
    pinMode(BLUE_STATUS_PIN, OUTPUT);
    pinMode(START_SWITCH, INPUT);
    setupMagnets();
    setRGBStatus(0, 255, 0);
    Serial.begin(9600);
    while (digitalRead(START_SWITCH) == LOW) {
        delay(100); // Sleep until start switch is pressed.
    }
    resetSteppers();
    startStepper(true);
}

void loop() {
    int stepperStatus = runStepper();

    if (newDistanceReading() && distanceChanged()) {
        bool endOfBoard = changeDirection(true);
        if (endOfBoard) {
            Serial.println("WE ARE DONE!!!");
            while (1) {
                // Infinite loop when we are done.
            }
        }
    }

    if (stepperStatus) {
        if (stepperStatus < MEASURE_DISTANCE) {
            switchMagnets(stepperStatus);
        }
        else {
            startDistanceReading(getDirections());
        }
    }
}