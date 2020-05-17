const int horizontalPin = 6; // Pin of horizontal actuator magnet.
const int verticalPin = 1337; // Pin of vertical actuator magnet.

void setupMagnets() {
    pinMode(horizontalPin, OUTPUT);
    //pinMode(verticalPin, OUTPUT);
}

/**
 * Switch the magnets on the linear actuators on/off.
 * 
 * @param status Status for which magnet to turn on, either
 * MAGNET_HORIZONTAL_ON or MAGNET_VERTICAL_ON. Turning one on
 * will turn the other off.
 */
void switchMagnets(int status) {
    String onMag = status == MAGNET_HORIZONTAL_ON ? "Horizontal on." : "Vertical on.";
    Serial.println("Switching magnets! " + onMag);
    int lowPin = status == MAGNET_HORIZONTAL_ON ? verticalPin : horizontalPin;
    int highPin = status == MAGNET_HORIZONTAL_ON ? horizontalPin : verticalPin;
    if (highPin == horizontalPin) { // Since we don't actually have two magnets :(
        digitalWrite(highPin, HIGH); // Turn on magnet indicated by status.
        setRGBStatus(0, 255, 255);
    }
    if (lowPin == horizontalPin) { // Since we don't actually have two magnets :(
        digitalWrite(lowPin, LOW); // Turn off the other magnet.
        setRGBStatus(0, 0, 0);
    }
}