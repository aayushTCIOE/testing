#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>


Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10); 
  }

  
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  
  
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  
  
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(100);
}

void loop() {  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  
  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("---------------------------------------");
  delay(500);
}

#define TINY_GSM_MODEM_SIM800  

#include <TinyGsmClient.h>
#include <HardwareSerial.h>

#define RX_PIN 16  
#define TX_PIN 17  
#define BAUD_RATE 9600
#define BUZZER_PIN 4      
#define SWITCH_PIN 5      

HardwareSerial gsmSerial(2);
TinyGsm modem(gsmSerial);

bool buzzerState = false;
bool buttonPressed = false;
bool smsSent = false;

void sendSMS() {
    String phoneNumber = "+9779768432105";  
    String message = "Hello from ESP32!";
    Serial.println("Sending SMS...");
    if (modem.sendSMS(phoneNumber, message)) {
        Serial.println("SMS Sent Successfully!");
    } else {
        Serial.println("SMS Failed!");
    }
}

bool getConditionFromApp() {
    return random(0, 2);  // Random true or false
}

void setup() {
    Serial.begin(115200);
    gsmSerial.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);

    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(SWITCH_PIN, INPUT_PULLUP);

    Serial.println("Initializing SIM800L...");
    if (!modem.restart()) {
        Serial.println("Failed to restart SIM800L!");
        return;
    }
    Serial.println("SIM800L Ready!");
}

void loop() {
    bool condition = getConditionFromApp();  
    if (condition && !buzzerState) {
        digitalWrite(BUZZER_PIN, HIGH);  
        buzzerState = true;
        Serial.println("Buzzer ON");

        unsigned long startTime = millis();  
        while (millis() - startTime < 10000) {  
            if (digitalRead(SWITCH_PIN) == LOW) {  
                delay(200);  
                if (digitalRead(SWITCH_PIN) == LOW) {  
                    digitalWrite(BUZZER_PIN, LOW);  
                    Serial.println("Buzzer turned OFF - No SMS sent");
                    buttonPressed = true;
                    smsSent = false;
                    while (digitalRead(SWITCH_PIN) == LOW);
                    delay(200); 
                    break; 
                }
            }
        }

        if (!buttonPressed && !smsSent) {
            Serial.println("Buzzer continues ringing - Sending SMS");
            sendSMS();  
            smsSent = true;
        }
    
        buzzerState = false;
        buttonPressed = false;
    }  
    delay(1000);  
}
