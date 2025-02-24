#define TINY_GSM_MODEM_SIM800  

#include <TinyGsmClient.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>

const int buzzerPin = 27;
const int switchPin = 14;
#define RX_PIN 3 
#define TX_PIN 1
#define BAUD_RATE 9600
#define GPS_RX 16
#define GPS_TX 17

HardwareSerial gsmSerial(2);
HardwareSerial gpsSerial(1);
TinyGsm modem(gsmSerial);
TinyGPSPlus gps;

bool buzzerState = false;
bool buttonPressed = false;

double latitude;
double longitude;

bool isFall() {
 
}

void getGPSLocation() {
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
        if (gps.location.isUpdated()) {
            latitude = gps.location.lat();
            longitude = gps.location.lng();
        }
    }
}

void sendSMS() {
    String phoneNumber = "+9779768432105";  
    // getGPSLocation();
    String message = "Fall Detected! Location: " + 
                     String(latitude, 6) + ", " + String(longitude, 6) + 
                     " (Google Maps: https://maps.google.com/?q=" + 
                     String(latitude, 6) + "," + String(longitude, 6) + ")";
    
    Serial.println("Sending SMS...");
    if (modem.sendSMS(phoneNumber, message)) {
        Serial.println("SMS Sent Successfully!");
    } else {
        Serial.println("SMS Failed!");
    }
}

void setup() {
    pinMode(buzzerPin, OUTPUT);
    pinMode(switchPin, INPUT_PULLUP);
    Serial.begin(115200);
    
    gsmSerial.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
    gpsSerial.begin(BAUD_RATE, SERIAL_8N1, GPS_RX, GPS_TX);
    Serial.println("Initializing SIM800L...");
    if (!modem.restart()) {
        Serial.println("Failed to restart SIM800L!");
        return;
    }
    Serial.println("SIM800L Ready!");
}

void loop() {
    bool condition = isFall();
    // getGPSLocation();
    
    if (condition && !buzzerState) {
        digitalWrite(buzzerPin, HIGH);
        buzzerState = true;
        Serial.println("Buzzer ON");

        unsigned long startTime = millis();  
        
        while (millis() - startTime < 10000) {
            if (digitalRead(switchPin) == LOW) {
                delay(200);
                if (digitalRead(switchPin) == LOW) {
                    digitalWrite(buzzerPin, LOW);
                    Serial.println("Buzzer turned OFF - No SMS sent");
                    buttonPressed = true;
                    while (digitalRead(switchPin) == LOW);
                    delay(200);
                    break;
                }
            }
        }

        if (!buttonPressed) {
            Serial.println("Buzzer continues ringing - Sending SMS");
            sendSMS();  
        }

        buzzerState = false;
        buttonPressed = false;
    }
    delay(1000);
}
