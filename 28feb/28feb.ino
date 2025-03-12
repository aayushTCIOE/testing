#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

const char* ssid = "dipeshdb_2";
const char* password = "9867795383@db";
const char* mqtt_server = "broker.emqx.io";  
const int mqtt_port = 1883;
const char* mqtt_topic_pub = "sensor_datas";

WiFiClient espClient;
PubSubClient client(espClient);
Adafruit_MPU6050 mpu;


// yesle forward fall detect garucha
bool isForwardFall(float ax,float ay, float az,float gx,float gy,float gz){
 if((ax >= -2 && ax <= 5) && (ay >= -1.5 && ay <= 1.7) && (az >= 6 && az <= 9.4) ){
    if((gx >= -1 && gx <= 1) && (gy >=  -1 && gy <= 1) && (gz >= -0.5 && gz <= 0.5) ) {return true;}   
  }
 return false;
}


// yesle backward fall detect garucha
bool isBackwardFall(float ax,float ay, float az,float gx,float gy,float gz){
 if((ax >= -4.5 && ax <= 5) && (ay >=  -5 && ay <= 3.3) && (az >= 5 && az <= 9) ){
    if((gx >= -1 && gx <= 1) && (gy >=  -1 && gy <= 1) && (gz >= -0.5 && gz <= 0.5) ){
     return true;}
  }
  return false;
}


// yesle lateral fall detect garucha
bool islateralFall(float ax,float ay, float az,float gx,float gy,float gz){
 if((ax >= -6 && ax <= 2) && (ay >=  -1.5 && ay <= 6) && (az >= 5.8 && az <= 9) ){
    if((gx >= -1 && gx <= 1) && (gy >=  -1 && gy <= 1) && (gz >= -0.5 && gz <= 0.5) ){
     return true;}
  }
   return false;
}


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

    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
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

   float gx = g.gyro.x;
   float gy = g.gyro.y;
   float gz = g.gyro.z;

    float ax = a.acceleration.x;
    float ay = a.acceleration.y;
    float az = a.acceleration.z;

  bool fallDetected=false ;

if ((gx >= -0.13 && gx <= -0.10) && 
    (gy >= -0.04 && gy <= -0.02) && 
    (gz >= 0.01 && gz <= 0.02)) {
    
    Serial.println("Stable condition detected, skipping fall detection.");
    fallDetected = false;
} else {
    fallDetected = isForwardFall(ax, ay, az, gx, gy, gz) || 
                   isBackwardFall(ax, ay, az, gx, gy, gz) || 
                   islateralFall(ax, ay, az, gx, gy, gz);
}

    // Construct JSON message
    String message = "{";
    message += "\"Ax\": " + String(ax) + ", ";
    message += "\"Ay\": " + String(ay) + ", ";
    message += "\"Az\": " + String(az) + ", ";
    message += "\"Gx\": " + String(gx) + ", ";
    message += "\"Gy\": " + String(gy) + ", ";
    message += "\"Gz\": " + String(gz) + ", ";
    message += "\"FallDetected\": \"" + String(fallDetected ? "YES" : "NO") + "\"";
    message += "}";

    client.publish(mqtt_topic_pub, message.c_str());
    Serial.println("Sensor data sent to MQTT Broker");

    delay(800);
}
