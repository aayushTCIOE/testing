#define TINY_GSM_MODEM_SIM800  

#include <TinyGsmClient.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

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
Adafruit_MPU6050 mpu;

bool buzzerState = false;
bool buttonPressed = false;
bool trigger1 = false, trigger2 = false, trigger3 = false, fall = false;
int trigger1count = 0, trigger2count = 0, trigger3count = 0;
double latitude;
double longitude;

  bool isFall() {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      
      float Amp = sqrt(a.acceleration.x * a.acceleration.x +
                      a.acceleration.y * a.acceleration.y +
                      a.acceleration.z * a.acceleration.z);
      float gx = g.gyro.x;
      float gy = g.gyro.y;
      float gz = g.gyro.z;
      float angleChange;
      
      if (Amp <= 2 && !trigger2) {
          trigger1 = true;
          Serial.println("TRIGGER 1 ACTIVATED");
      }
      
      if (trigger1) {
          trigger1count++;
          if (Amp >= 12) {
              trigger2 = true;
              Serial.println("TRIGGER 2 ACTIVATED");
              trigger1 = false;
              trigger1count = 0;
          }
      }
      
      if (trigger2) {
          trigger2count++;
          angleChange = sqrt(gx * gx + gy * gy + gz * gz);
          Serial.println(angleChange);
          if (angleChange >= 30 && angleChange <= 400) {
              trigger3 = true;
              trigger2 = false;
              trigger2count = 0;
              Serial.println("TRIGGER 3 ACTIVATED");
          }
      }
      
      if (trigger3) {
          trigger3count++;
          if (trigger3count >= 10) {
              angleChange = sqrt(gx * gx + gy * gy + gz * gz);
              Serial.println(angleChange);
              if (angleChange >= 0 && angleChange <= 10) {
                  fall = true;
                  trigger3 = false;
                  trigger3count = 0;
                  Serial.println("FALL DETECTED");
                  return true;
              } else {
                  trigger3 = false;
                  trigger3count = 0;
                  Serial.println("TRIGGER 3 DEACTIVATED");
              }
          }
      }
      return false;
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
    
    Serial.println("Initializing MPU6050...");
    if (!mpu.begin()) {
        Serial.println("MPU6050 initialization failed!");
        while (1);
    }
    Serial.println("MPU6050 Ready!");

    Serial.println("Initializing SIM800L...");
    if (!modem.restart()) {
        Serial.println("Failed to restart SIM800L!");
        return;
    }
    Serial.println("SIM800L Ready!");
}

void loop() {
    bool condition = isFall();
    
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
