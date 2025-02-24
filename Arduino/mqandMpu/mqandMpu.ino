#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

const char* ssid = "dipeshdb_2";
const char* password = "9867795383@db";
const char* mqtt_server = "broker.emqx.io";
const int mqtt_port = 1883;
const char* mqtt_topic_pub = "testy";

WiFiClient espClient;
PubSubClient client(espClient);
Adafruit_MPU6050 mpu;

unsigned long lastMsg = 0;
float ax{},ay{},az{},gx{},gy{},gz{};
float gyroXOffset{}, gyroYOffset{}, gyroZOffset{};

void setup_wifi() {
    delay(10);
    Serial.println();
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
        String clientId = "ESP32Client-";
        clientId += String(random(0xffff), HEX);
        if (client.connect(clientId.c_str())) {
            Serial.println("Connected to MQTT Broker");
        } else {
            Serial.print("Failed, rc=");
            Serial.print(client.state());
            Serial.println(" Retrying in 5 seconds");
            delay(5000);
        }
    }
}

void calibrateGyro() {
    for (int i = 0; i < 1000; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        gyroXOffset += g.gyro.x;
        gyroYOffset += g.gyro.y;
        gyroZOffset += g.gyro.z;
    }
    gyroXOffset /= 1000;
    gyroYOffset /= 1000;
    gyroZOffset /= 1000;
}

void setup() {
    Serial.begin(115200);
    setup_wifi();
    client.setServer(mqtt_server, mqtt_port);
    
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) {
            delay(10);
        }
    }
    Serial.println("MPU6050 Found!");
    calibrateGyro();

    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    delay(100);
}

void loop() {
    if (!client.connected()) {
        reconnect();
    }
    client.loop();

    unsigned long now = millis();
    if (now - lastMsg > 20) {
        lastMsg = now;

        sensors_event_t accel, gyro, temp;
        mpu.getEvent(&accel, &gyro, &temp);

        ax = accel.acceleration.x;
        ay = accel.acceleration.y;
        az = accel.acceleration.z;
        gx = gyro.gyro.x - gyroXOffset;
        gy = gyro.gyro.y - gyroYOffset;
        gz = gyro.gyro.z - gyroZOffset;

float timestamp = (now / 1000.0); 
 Serial.print(timestamp);Serial.print(","); Serial.print(ax); Serial.print(","); Serial.print(ay); Serial.print(","); Serial.print(az);  
 Serial.print(",");Serial.print(gx);  Serial.print(","); Serial.print(gy);Serial.print(","); Serial.println(gz, 4);  
        String message =  String(timestamp) + "," + String(ax,4) + "," + String(ay,4) + "," + String(az,4) + "," + String(gx,4) + "," + String(gy,4) + "," + String(gz,4) ;
        
        client.publish(mqtt_topic_pub, message.c_str());
    }
}
