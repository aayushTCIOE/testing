#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid = "test_Ehall";
const char* password = "@dmin@123";

const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* mqtt_topic_pub = "test";      



WiFiClient espClient;
PubSubClient client(espClient);


unsigned long lastMsg = 0;
// const long interval = 2000; 

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

void reconnect() {
    while (!client.connected()) {
    String clientId = "ESP32222Client-";
    clientId += String(random(0xffff), HEX);   
    if (client.connect(clientId.c_str())) {
            Serial.println("connected");             
            client.publish(mqtt_topic_pub, "tomorraom is my project");
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" retrying in 5 seconds");
            delay(5000);
        }
    }
}

void setup() {
    Serial.begin(115200);    
    setup_wifi();   
    client.setServer(mqtt_server, mqtt_port); 
}

void loop() {   
    if (!client.connected()) {
        reconnect();
    }
    client.loop();   
    unsigned long now = millis();
    if (now - lastMsg > 20) {
        lastMsg = now;      
        String message = "Temperature: " + String(random(20, 30)); 
        client.publish(mqtt_topic_pub, message.c_str());
    }
}