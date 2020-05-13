#include <Stepper.h>

// Direction flags for seeing where the robot is heading.
#define STOPPED 0
#define WEST 1
#define EAST 2
#define NORTH 3
#define SOUTH 4

// Status codes for indicating if vertical/horizontal magnets should turn on/off. 
#define MAGNET_HORIZONTAL_ON 1
#define MAGNET_VERTICAL_ON 2

// Reset switch pin.
#define RESET_SWITCH A0
// Kill switch pin.
#define KILL_SWITCH A1

// Pins for RGB LED
#define RED_STATUS_PIN 3
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
    pinMode(RESET_SWITCH, INPUT);
    pinMode(KILL_SWITCH, INPUT);
    pinMode(RED_STATUS_PIN, OUTPUT);
    pinMode(GREEN_STATUS_PIN, OUTPUT);
    pinMode(BLUE_STATUS_PIN, OUTPUT);
    setupDistSensors();
    //setupMagnets();
    setRGBStatus(0, 255, 0);
    Serial.begin(9600);
}

void loop() {
    bool distChanged = distanceChanged(getDirections());

    if (distChanged) { // Distance sensor detected a change. Hopefully means we hit edge of board.
        bool endOfBoard = changeDirection(true);
        if (endOfBoard) {
            Serial.println("WE ARE DONE!!!");
            while (1) {
                // Infinite loop when we are done.
            }
        }
    }
    int magnetStatus = runStepper();

    if (magnetStatus) {
        Serial.println("Switching magnets!");
        //switchMagnets(magnetStatus);
    }
    if (digitalRead(KILL_SWITCH) == HIGH) {
        delay(10);
        if (digitalRead(KILL_SWITCH) == HIGH) {
            Serial.println("Kill switch pressed! Terminating program...");
            delay(500);
            cli();
            while (true) {
                // Sleep, little one, sleep...
            }
        }
    }
    if (digitalRead(RESET_SWITCH) == HIGH) {
        Serial.println("Reset switch pressed. Resetting actuator...");
        delay(100);
        while (digitalRead(RESET_SWITCH) == HIGH) {
            runOnce();
            delay(100);
        }
        resetStepper();
    }
}