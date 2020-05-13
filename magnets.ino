const int horizontalPin = 2; // Pin of horizontal actuator magnet.
const int verticalPin = 3; // Pin of vertical actuator magnet.

void setupMagnets() {
    pinMode(horizontalPin, OUTPUT);
    pinMode(verticalPin, OUTPUT);
}

void switchMagnets(int status) {
    int voltage = status > 0 ? HIGH : LOW;
    int otherVoltage = status > 0 ? LOW : HIGH;
    int pin = abs(status) == MAGNET_HORIZONTAL_ON ? horizontalPin : verticalPin;
    int otherPin = pin == horizontalPin ? verticalPin : horizontalPin;
    digitalWrite(pin, voltage); // Turn on/off magnet indicated by status.
    digitalWrite(otherPin, otherVoltage); // Turn off/on the other magnet.
}