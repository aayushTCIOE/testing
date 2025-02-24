// #define TINY_GSM_MODEM_SIM800  

// #include <TinyGsmClient.h>
// #include <HardwareSerial.h>

// HardwareSerial gsmSerial(2);
// #define RX_PIN 16  
// #define TX_PIN 17  
// #define BAUD_RATE 9600

// TinyGsm modem(gsmSerial);

// void setup() {
//     Serial.begin(115200);
//     gsmSerial.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);

//     Serial.println("Initializing SIM800L...");
//     if (!modem.restart()) {
//         Serial.println("Failed to restart SIM800L!");
//         return;
//     }
//     Serial.println("SIM800L Ready!");


//     String phoneNumber = "+9779768432105";  
//     String message = "Hello from ESP32!";
//     if (modem.sendSMS(phoneNumber, message)) {
//         Serial.println("SMS Sent Successfully!");
//     } else {
//         Serial.println("SMS Failed!");
//     }
// }

// void loop() {
 
// }


#define TINY_GSM_MODEM_SIM800  

#include <TinyGsmClient.h>
#include <HardwareSerial.h>

HardwareSerial gsmSerial(2);
#define RX_PIN 16  
#define TX_PIN 17  
#define BAUD_RATE 9600

TinyGsm modem(gsmSerial);

void setup() {
    Serial.begin(115200);
    gsmSerial.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);

    Serial.println("Initializing SIM800L...");
    if (!modem.restart()) {
        Serial.println("Failed to restart SIM800L!");
        return;
    }
    Serial.println("SIM800L Ready!");
double latitude= 38.8951;
double longitude = -77.0364; 

    String phoneNumber = "+9779768432105";  
String message = "Fall Detected! Location: " + 
                     String(latitude, 6) + ", " + String(longitude, 6) + 
                     " (Google Maps: https://maps.google.com/?q=" + 
                     String(latitude, 6) + "," + String(longitude, 6) + ")";
    if (modem.sendSMS(phoneNumber, message)) {
        Serial.println("SMS Sent Successfully!");
    } else {
        Serial.println("SMS Failed!");
    }
}

void loop() {
 
}



// void sendSMS(float latitude, float longitude) {
//     String phoneNumber = "+977xxxxxxxxxx"; //number of the registerd caretaker  
//     String message = "Fall Detected! Location: " + 
//                      String(latitude, 6) + ", " + String(longitude, 6) + 
//                      " (Google Maps: https://maps.google.com/?q=" + 
//                      String(latitude, 6) + "," + String(longitude, 6) + ")";

//     if (modem.sendSMS(phoneNumber, message)) {
//         Serial.println("SMS Sent ");
//     } else {
//         Serial.println("SMS Not sent");
//     }
// }

// void buzzerAndPushButton(){
//   bool condition = getConditionFromModel();  
//   if (condition && !buzzerState) {
//     digitalWrite(buzzerPin, HIGH);  
//     buzzerState = true;
//     Serial.println("Buzzer ON");

//     unsigned long startTime = millis();  
    
//     while (millis() - startTime < 10000) {  
//       if (digitalRead(switchPin) == LOW) {  
//         delay(200);  
//         if (digitalRead(switchPin) == LOW) {  
//           digitalWrite(buzzerPin, LOW);  
//           Serial.println("Switch is pressed and the buzzer does not ring anymore and the sms is not sent");
//           buttonPressed = true;         
         
//           while (digitalRead(switchPin) == LOW);
//           delay(200); 
//           break; 
//         }
//       }
//     }

//     if (!buttonPressed) {
//       Serial.println("buzzer is still ringing");
//        }
   
//     if (buttonPressed) {
//       sendSMS();  
//     } 
// }

// void GPSandMPU(){
//   while (GPS_Serial.available() > 0) {
//     char c = GPS_Serial.read();
//     gps.encode(c);
//   }

//   //  MPU6050 data
//   sensors_event_t accel, gyro, temp;
//   mpu.getEvent(&accel, &gyro, &temp);

//   ax = accel.acceleration.x;
//   ay = accel.acceleration.y;
//   az = accel.acceleration.z;

//   gx = gyro.gyro.x - gyroXOffset;
//   gy = gyro.gyro.y - gyroYOffset;
//   gz = gyro.gyro.z - gyroZOffset;

//   // Print MPU6050 data
//   Serial.print("Accel: ");
//   Serial.print(ax); Serial.print(", ");
//   Serial.print(ay); Serial.print(", ");
//   Serial.print(az);
//   Serial.print(" | Gyro: ");
//   Serial.print(gx); Serial.print(", ");
//   Serial.print(gy); Serial.print(", ");
//   Serial.print(gz);

//   // Print GPS data 
//   if (gps.location.isUpdated()) {
//     Serial.print(" | GPS: ");
//     Serial.print(gps.location.lat(), 6);
//     Serial.print(", ");
//     Serial.print(gps.location.lng(), 6);
//     Serial.print(" | Alt: ");
//     Serial.print(gps.altitude.meters(), 2);
//   }
//   Serial.println();
//   delay(100);
// }



// void calibrateGyro() {
//   for (int i = 0; i < 1000; i++) {
//     sensors_event_t a, g, temp;
//     mpu.getEvent(&a, &g, &temp);
//     gyroXOffset += g.gyro.x;
//     gyroYOffset += g.gyro.y;
//     gyroZOffset += g.gyro.z;
//   }
//   gyroXOffset /= 1000;
//   gyroYOffset /= 1000;
//   gyroZOffset /= 1000;
// }
