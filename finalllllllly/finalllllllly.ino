#define TINY_GSM_MODEM_SIM800  

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGsmClient.h>
#include <HardwareSerial.h>

const int buzzerPin = 27;
const int switchPin = 14;
#define RX_PIN 3 
#define TX_PIN 1
#define BAUD_RATE 9600

bool buzzerState = false;
bool buttonPressed = false;

HardwareSerial gsmSerial(2);
TinyGsm modem(gsmSerial);
Adafruit_MPU6050 mpu;

double latitude=27.672561464117486;
double longitude=85.35202087747004;


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

// yesle forward fall detect garucha
bool isForwardFall(float ax,float ay, float az,float gx,float gy,float gz){
  if((ax >= -2 && ax <= 5) && (ay >= -1.5 && ay <= 1.7) && (az >= 6 && az <= 9.4) ){
    // if((gx >= -1 && gx <= 1) && (gy >=  -1 && gy <= 1) && (gz >= -0.5 && gz <= 0.5) ) {return true;}
   if((gx >= -1 && gx <= 1) && (gy >=  -1 && gy <= 1) && (gz >=10) ) {return true;}
  }
  return false;
}


// yesle backward fall detect garucha
bool isBackwardFall(float ax,float ay, float az,float gx,float gy,float gz){
  if((ax >= -4.5 && ax <= 5) && (ay >=  -5 && ay <= 3.3) && (az >= 5 && az <= 9) ){
    // if((gx >= -1 && gx <= 1) && (gy >=  -1 && gy <= 1) && (gz >= -0.5 && gz <= 0.5) ){
         if((gx >= -1 && gx <= 1) && (gy >=  -1 && gy <= 1) && (gz >=10) ) {return true;}

   
  }
  return false;
}


// yesle lateral fall detect garucha
bool islateralFall(float ax,float ay, float az,float gx,float gy,float gz){
  if((ax >= -6 && ax <= 2) && (ay >=  -1.5 && ay <= 6) && (az >= 5.8 && az <= 9) ){
    // if((gx >= -1 && gx <= 1) && (gy >=  -1 && gy <= 1) && (gz >= -0.5 && gz <= 0.5) ){
               if((gx >= -1 && gx <= 1) && (gy >=  -1 && gy <= 1) && (gz >=10) ) {return true;}

     
  }
  return false;
}

void setup() {
   pinMode(buzzerPin, OUTPUT);
    pinMode(switchPin, INPUT_PULLUP);
    Serial.begin(115200);

   gsmSerial.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);

   
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) delay(10);
    }
    Serial.println("MPU6050 Initialized!");

    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void loop() {
sensors_event_t a, g, temp;
mpu.getEvent(&a, &g, &temp);

   float gx = g.gyro.x;
    float gy = g.gyro.y;
    float gz = g.gyro.z;

    float ax = a.acceleration.x;
    float ay = a.acceleration.y;
    float az = a.acceleration.z;

      bool fallDetected=false ;

if ((gx >= 100) && 
    (gy >= 100) && 
    (gz >= 100)) {
    
    Serial.println("Stable condition detected, skipping fall detection.");
    fallDetected = false;
} else {
    fallDetected = isForwardFall(ax, ay, az, gx, gy, gz) || 
                   isBackwardFall(ax, ay, az, gx, gy, gz) || 
                   islateralFall(ax, ay, az, gx, gy, gz);
}

if (fallDetected && !buzzerState) {
// if ( (isForwardFall(ax,ay,az,gx,gy,gz)) && !buzzerState ) {  
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
            // Serial.println("sms is sent");
             delay(30000); 
        }

        buzzerState = false;
        buttonPressed = false;
    }
 delay(800);
     
}
