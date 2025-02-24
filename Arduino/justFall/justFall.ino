// #include <Wire.h>
// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>

// // Create MPU6050 object
// Adafruit_MPU6050 mpu;

// // Fall detection variables
// bool trigger1 = false, trigger2 = false, trigger3 = false, fall = false;
// int trigger1count = 0, trigger2count = 0, trigger3count = 0;

// bool isFall() {
//     sensors_event_t a, g, temp;
//     mpu.getEvent(&a, &g, &temp);
    
//     float Amp = sqrt(a.acceleration.x * a.acceleration.x +
//                      a.acceleration.y * a.acceleration.y +
//                      a.acceleration.z * a.acceleration.z);
//     float gx = g.gyro.x;
//     float gy = g.gyro.y;
//     float gz = g.gyro.z;
//     float angleChange;
    
//     if (Amp <= 2 && !trigger2) {
//         trigger1 = true;
//         Serial.println("TRIGGER 1 ACTIVATED");
//     }
    
//     if (trigger1) {
//         trigger1count++;
//         if (Amp >= 12) {
//             trigger2 = true;
//             Serial.println("TRIGGER 2 ACTIVATED");
//             trigger1 = false;
//             trigger1count = 0;
//         }
//     }
    
//     if (trigger2) {
//         trigger2count++;
//         angleChange = sqrt(gx * gx + gy * gy + gz * gz);
//         Serial.println(angleChange);
//         if (angleChange >= 30 && angleChange <= 400) {
//             trigger3 = true;
//             trigger2 = false;
//             trigger2count = 0;
//             Serial.println("TRIGGER 3 ACTIVATED");
//         }
//     }
    
//     if (trigger3) {
//         trigger3count++;
//         if (trigger3count >= 10) {
//             angleChange = sqrt(gx * gx + gy * gy + gz * gz);
//             Serial.println(angleChange);
//             if (angleChange >= 0 && angleChange <= 10) {
//                 fall = true;
//                 trigger3 = false;
//                 trigger3count = 0;
//                 Serial.println("FALL DETECTED");
//                 return true;
//             } else {
//                 trigger3 = false;
//                 trigger3count = 0;
//                 Serial.println("TRIGGER 3 DEACTIVATED");
//             }
//         }
//     }
//     return false;
// }

// void setup() {
//     Serial.begin(115200);
//     while (!Serial);

//     // Initialize MPU6050
//     if (!mpu.begin()) {
//         Serial.println("Failed to find MPU6050 chip");
//         while (1) {
//             delay(10);
//         }
//     }

//     Serial.println("MPU6050 Found!");
//     mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
//     mpu.setGyroRange(MPU6050_RANGE_500_DEG);
//     mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

//     delay(100);
// }

// void loop() {
//     if (isFall()) {
//         Serial.println("FALL ALERT: Fall detected!");
//         delay(5000); // Delay before next detection
//     }
//     delay(50); // Small delay to prevent excessive sensor polling
// }









// #include <Wire.h>
// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>

// // Create MPU6050 object
// Adafruit_MPU6050 mpu;

// // Fall detection variables
// bool trigger1 = false, trigger2 = false, trigger3 = false, fall = false;
// int trigger1count = 0, trigger2count = 0, trigger3count = 0;

// bool isFall(float ax, float ay, float az, float gx, float gy, float gz) {
//     float Amp = sqrt(ax * ax + ay * ay + az * az);
//     float angleChange = sqrt(gx * gx + gy * gy + gz * gz);
    
//     if (Amp <= 2 && !trigger2) {
//         trigger1 = true;
//         Serial.println("TRIGGER 1 ACTIVATED");
//     }

//     if (trigger1) {
//         trigger1count++;
//         if (Amp >= 12) {
//             trigger2 = true;
//             Serial.println("TRIGGER 2 ACTIVATED");
//             trigger1 = false;
//             trigger1count = 0;
//         }
//     }

//     if (trigger2) {
//         trigger2count++;
//         Serial.println(angleChange);
//         if (angleChange >= 30 && angleChange <= 400) {
//             trigger3 = true;
//             trigger2 = false;
//             trigger2count = 0;
//             Serial.println("TRIGGER 3 ACTIVATED");
//         }
//     }

//     if (trigger3) {
//         trigger3count++;
//         if (trigger3count >= 10) {
//             angleChange = sqrt(gx * gx + gy * gy + gz * gz);
//             Serial.println(angleChange);
//             if (angleChange >= 0 && angleChange <= 10) {
//                 fall = true;
//                 trigger3 = false;
//                 trigger3count = 0;
//                 Serial.println("FALL DETECTED");
//                 return true;
//             } else {
//                 trigger3 = false;
//                 trigger3count = 0;
//                 Serial.println("TRIGGER 3 DEACTIVATED");
//             }
//         }
//     }
//     return false;
// }

// void setup() {
//     Serial.begin(115200);
//     while (!Serial);

//     // Initialize MPU6050
//     if (!mpu.begin()) {
//         Serial.println("Failed to find MPU6050 chip");
//         while (1) {
//             delay(10);
//         }
//     }

//     Serial.println("MPU6050 Found!");
//     mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
//     mpu.setGyroRange(MPU6050_RANGE_500_DEG);
//     mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

//     delay(100);
// }

// void loop() {
//     // Read sensor values
//     sensors_event_t a, g, temp;
//     mpu.getEvent(&a, &g, &temp);

//     float ax = a.acceleration.x;
//     float ay = a.acceleration.y;
//     float az = a.acceleration.z;
//     float gx = g.gyro.x;
//     float gy = g.gyro.y;
//     float gz = g.gyro.z;

//     // Pass values to the function
//     if (isFall(ax, ay, az, gx, gy, gz)) {
//         Serial.println("FALL ALERT: Fall detected!");
//         delay(5000); // Delay before next detection
//     }
//     delay(50); // Small delay to prevent excessive sensor polling
// }







// #include <Wire.h>
// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>

// Adafruit_MPU6050 mpu;

// void setup() {
//     Serial.begin(115200);
//     while (!Serial);

//     if (!mpu.begin()) {
//         Serial.println("Failed to find MPU6050 chip");
//         while (1) { delay(10); }
//     }

//     Serial.println("MPU6050 Initialized!");
//     mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
//     mpu.setGyroRange(MPU6050_RANGE_500_DEG);
//     mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
//     delay(100);
// }

// bool detectFall() {
//     sensors_event_t a, g, temp;
//     mpu.getEvent(&a, &g, &temp);

//     // Calculate total acceleration magnitude
//     float acceleration = sqrt(a.acceleration.x * a.acceleration.x + 
//                               a.acceleration.y * a.acceleration.y + 
//                               a.acceleration.z * a.acceleration.z);

//     // Calculate total gyroscope movement
//     float gyro = sqrt(g.gyro.x * g.gyro.x + g.gyro.y * g.gyro.y + g.gyro.z * g.gyro.z);

//     Serial.print("Acc: "); Serial.print(acceleration);
//     Serial.print(" | Gyro: "); Serial.println(gyro);

//     // Check for Free Fall (low acceleration)
//     if (acceleration < 2) {
//         Serial.println("Free Fall Detected...");
//         delay(100); // Wait for impact
//         mpu.getEvent(&a, &g, &temp);
//         acceleration = sqrt(a.acceleration.x * a.acceleration.x + 
//                             a.acceleration.y * a.acceleration.y + 
//                             a.acceleration.z * a.acceleration.z);
//         if (acceleration > 12) { // Impact detected
//             Serial.println("Impact Detected...");
//             delay(500); // Wait to check for stability
//             mpu.getEvent(&a, &g, &temp);
//             gyro = sqrt(g.gyro.x * g.gyro.x + g.gyro.y * g.gyro.y + g.gyro.z * g.gyro.z);
//             if (gyro < 5) { // No movement after impact
//                 Serial.println("FALL DETECTED!");
//                 return true;
//             }
//         }
//     }
//     return false;
// }

// void loop() {
//     if (detectFall()) {
//         Serial.println("FALL ALERT! Person has fallen.");
//         Serial.println("ok to the next iteration"); // Delay before next detection
//     }
//     delay(50); // Small delay to prevent excessive sensor polling
// }


#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Create MPU6050 object
Adafruit_MPU6050 mpu;

// Fall detection variables
bool trigger1 = false, trigger2 = false, trigger3 = false, fall = false;
int trigger1count = 0, trigger2count = 0, trigger3count = 0;

bool isFall(float ax, float ay, float az, float gx, float gy, float gz) {
    float Amp = sqrt(ax * ax + ay * ay + az * az);
    float angleChange = sqrt(gx * gx + gy * gy + gz * gz);

    Serial.print("Acceleration Magnitude: ");
    Serial.print(Amp);
    Serial.print(" | Gyro Magnitude: ");
    Serial.println(angleChange);

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
        Serial.print("Gyro Change: ");
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
            Serial.print("Final Gyro Check: ");
            Serial.println(angleChange);
            if (angleChange >= 0 && angleChange <= 10) {
                fall = true;
                trigger3 = false;
                trigger3count = 0;
                Serial.println("FALL DETECTED!");
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

void setup() {
    Serial.begin(115200);
    while (!Serial);

    // Initialize MPU6050
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
    // Read sensor values
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float ax = a.acceleration.x;
    float ay = a.acceleration.y;
    float az = a.acceleration.z;
    float gx = g.gyro.x;
    float gy = g.gyro.y;
    float gz = g.gyro.z;

    // Print raw sensor data
    Serial.print("Ax: "); Serial.print(ax);
    Serial.print(" | Ay: "); Serial.print(ay);
    Serial.print(" | Az: "); Serial.print(az);
    Serial.print(" || Gx: "); Serial.print(gx);
    Serial.print(" | Gy: "); Serial.print(gy);
    Serial.print(" | Gz: "); Serial.println(gz);

    // Pass values to the function
    if (isFall(ax, ay, az, gx, gy, gz)) {
        Serial.println("FALL ALERT: Fall detected!");
        Serial.println("fall detected.........................................");
        Serial.println("/........................................................................");
        Serial.println("/........................................................................");
        Serial.println("/........................................................................");
        Serial.println("/........................................................................");
        Serial.println("/........................................................................");
      
    }
    delay(50); // Small delay to prevent excessive sensor polling
}
