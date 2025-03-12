// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>
// #include <Wire.h>

// Adafruit_MPU6050 mpu;


// const float ACCEL_THRESHOLD = 11;  
// const float GYRO_CHANGE_THRESHOLD = 4; 
// const int TIME_WINDOW = 500;              

// void setup() {
//   Serial.begin(115200);
//   while (!Serial) delay(10); 
 
//   if (!mpu.begin()) {
//     Serial.println("Failed to find MPU6050 chip");
//     while (1) delay(10);
//   }
//   Serial.println("MPU6050 Found!");

//   // Configure MPU6050
//   mpu.setAccelerometerRange(MPU6050_RANGE_2_G); 
//   mpu.setGyroRange(MPU6050_RANGE_500_DEG);       
//   mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);    
// }

// void loop() {
 
//   sensors_event_t a, g, temp;
//   mpu.getEvent(&a, &g, &temp);

//   float ax = a.acceleration.x;
//   float ay = a.acceleration.y;
//   float az = a.acceleration.z;
//   float totalAccel = sqrt(ax * ax + ay * ay + az * az);


//   Serial.print("Total Acceleration (m/s²): ");
//   Serial.println(totalAccel);


//   if (totalAccel > ACCEL_THRESHOLD) {
//     Serial.println("Acceleration Threshold Crossed! Checking orientation...");

  
//     float initialGx = g.gyro.x;
//     float initialGy = g.gyro.y;
//     float initialGz = g.gyro.z;

   
//     delay(TIME_WINDOW);

   
//     mpu.getEvent(&a, &g, &temp);

  
//     float deltaGx = abs(g.gyro.x - initialGx) * (TIME_WINDOW / 1000.0); 
//     float deltaGy = abs(g.gyro.y - initialGy) * (TIME_WINDOW / 1000.0);
//     float deltaGz = abs(g.gyro.z - initialGz) * (TIME_WINDOW / 1000.0);

 
//     Serial.print("Gyro Change (deg) - X: ");
//     Serial.print(deltaGx);
//     Serial.print(" Y: ");
//     Serial.print(deltaGy);
//     Serial.print(" Z: ");
//     Serial.println(deltaGz);

//     if (deltaGx > GYRO_CHANGE_THRESHOLD || deltaGy > GYRO_CHANGE_THRESHOLD || deltaGz > GYRO_CHANGE_THRESHOLD) {
//       Serial.println("Fall Detected!");
//     } else {
//       Serial.println("No significant orientation change. Looping back.");
//     }
//   }

//   delay(100); // Sampling rate
// }












#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

const char* ssid = "";
const char* password = "";
const char* mqtt_server = "broker.emqx.io";  
const int mqtt_port = 1883;
const char* mqtt_topic_pub = "sensor_data";

WiFiClient espClient;
PubSubClient client(espClient);
Adafruit_MPU6050 mpu;

const float ACCEL_THRESHOLD = 11.0;  
const float GYRO_CHANGE_THRESHOLD = 4.0;  
const int TIME_WINDOW = 500;  

void setup_wifi() {
    Serial.print("Connecting to WiFi: ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected!");
}

void reconnect() {
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        String clientId = "ESP32Client-" + String(random(0xffff), HEX);
        if (client.connect(clientId.c_str())) {
            Serial.println("Connected to MQTT Broker!");
        } else {
            Serial.print("Failed, rc=");
            Serial.print(client.state());
            Serial.println(" Retrying in 5 seconds...");
            delay(5000);
        }
    }
}

void setup() {
    Serial.begin(115200);
    setup_wifi();
    client.setServer(mqtt_server, mqtt_port);
    
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) delay(10);
    }
    Serial.println("MPU6050 Initialized!");

    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    delay(100);
}

void loop() {
    if (!client.connected()) {
        reconnect();
    }
    client.loop();

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float ax = a.acceleration.x;
    float ay = a.acceleration.y;
    float az = a.acceleration.z;
    float totalAccel = sqrt(ax * ax + ay * ay + az * az);

    bool fallDetected = false;

    if (totalAccel > ACCEL_THRESHOLD) {
        Serial.println("Acceleration Threshold Crossed! Checking orientation...");
        
        float initialGx = g.gyro.x;
        float initialGy = g.gyro.y;
        float initialGz = g.gyro.z;

        delay(TIME_WINDOW);

        mpu.getEvent(&a, &g, &temp);

        float deltaGx = abs(g.gyro.x - initialGx) * (TIME_WINDOW / 1000.0); 
        float deltaGy = abs(g.gyro.y - initialGy) * (TIME_WINDOW / 1000.0);
        float deltaGz = abs(g.gyro.z - initialGz) * (TIME_WINDOW / 1000.0);

        Serial.print("Gyro Change (deg) - X: ");
        Serial.print(deltaGx);
        Serial.print(" Y: ");
        Serial.print(deltaGy);
        Serial.print(" Z: ");
        Serial.println(deltaGz);

        if (deltaGx > GYRO_CHANGE_THRESHOLD || deltaGy > GYRO_CHANGE_THRESHOLD || deltaGz > GYRO_CHANGE_THRESHOLD) {
            Serial.println("Fall Detected!");
            fallDetected = true;
        } else {
            Serial.println("No significant orientation change. Looping back.");
        }
    }

    // Construct JSON message
    String message = "{";
    message += "\"Ax\": " + String(ax) + ", ";
    message += "\"Ay\": " + String(ay) + ", ";
    message += "\"Az\": " + String(az) + ", ";
    message += "\"Gx\": " + String(g.gyro.x) + ", ";
    message += "\"Gy\": " + String(g.gyro.y) + ", ";
    message += "\"Gz\": " + String(g.gyro.z) + ", ";
    message += "\"TotalAccel\": " + String(totalAccel) + ", ";
    message += "\"FallDetected\": \"" + String(fallDetected ? "YES" : "NO") + "\"";
    message += "}";

    client.publish(mqtt_topic_pub, message.c_str());
    Serial.println("Sensor data sent to MQTT Broker");

    delay(1000);
}
