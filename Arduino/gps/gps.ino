#include <TinyGPS++.h>
#include <HardwareSerial.h>

static const int RXPin = 16, TXPin = 17;  
static const uint32_t GPSBaud = 115200;     

TinyGPSPlus gps;
HardwareSerial gpsSerial(1);  

double lastLat = 0.0, lastLng = 0.0;  

void setup() {
    Serial.begin(115200);
    gpsSerial.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);  // Start GPS communication
}

void loop() {
    while (gpsSerial.available() > 0) {  
        if (gps.encode(gpsSerial.read())) {
            if (gps.location.isUpdated()) {
                // Update the last known position
                lastLat = gps.location.lat();
                lastLng = gps.location.lng();
                
                Serial.println("**** New GPS Data ****");
                Serial.print("New Latitude: ");
                Serial.print(lastLat, 6);
                Serial.print(", Longitude: ");
                Serial.println(lastLng, 6);
            }
        }
    }
}
