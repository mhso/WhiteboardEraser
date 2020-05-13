const int horizontalPin = 2; // Pin of horizontal actuator magnet.
const int verticalPin = 3; // Pin of vertical actuator magnet.

void setupMagnets() {
    pinMode(horizontalPin, OUTPUT);
    pinMode(verticalPin, OUTPUT);
}

/**
 * Switch the magnets on the linear actuators on/off.
 * 
 * @param status Status for which magnet to turn on, either
 * MAGNET_HORIZONTAL_ON or MAGNET_VERTICAL_ON. Turning one on
 * will turn the other off.
 */
void switchMagnets(int status) {
    int lowPin = status == MAGNET_HORIZONTAL_ON ? verticalPin : horizontalPin;
    int highPin = status == MAGNET_HORIZONTAL_ON ? horizontalPin : verticalPin;
    digitalWrite(lowPin, HIGH); // Turn on magnet indicated by status.
    digitalWrite(highPin, LOW); // Turn off the other magnet.
}