#define PC_BAUDRATE 115200
#define GPS_BAUDRATE 115200

void setup() {
    Serial.begin(PC_BAUDRATE);
    Serial1.begin(GPS_BAUDRATE);
}

// If there is a data from the receiver, read it and send to the PC or vice versa.
void loop() {
    if (Serial1.available())
    {
        Serial.write(Serial1.read());
    }

    if (Serial.available())
    {
        Serial1.write(Serial.read());
    }
}
