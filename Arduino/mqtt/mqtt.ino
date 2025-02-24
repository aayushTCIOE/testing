// #include <WiFi.h>
// #include <PubSubClient.h>

// const char* ssid = "Who-Dat_2.4";
// const char* password = "@class@object@18";

// const char* mqttServer = "broker.emqx.io"; 
// const int mqttPort = 1883;

// WiFiClient espClient;
// PubSubClient client(espClient);

// void setup() {
//   Serial.begin(115200);


//   WiFi.begin(ssid, password);
//   while (WiFi.status() != WL_CONNECTED) {
//     delay(1000);
//     Serial.println("Connecting to WiFi...");
//   }
//   Serial.println("Connected to WiFi");


//   client.setServer(mqttServer, mqttPort);
//   while (!client.connected()) {
//     Serial.println("Connecting to MQTT...");
//     if (client.connect("ESP32_Client")) {
//       Serial.println("Connected to MQTT Broker");
//     } else {
//       Serial.print("Failed, rc=");
//       Serial.print(client.state());
//       Serial.println(" Retrying in 5 seconds...");
//       delay(5000);
//     }
//   }
// }

// void loop() {

//   Serial.println("Publishing 'Hello World'...");
//   client.publish("test/topic", "Hello World");

//   delay(5000);
//   client.loop();  
// }




#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid = "Who-Dat_2.4";
const char* password = "@class@object@18";
const char* mqtt_server = "broker.mqtt-dashboard.com";

WiFiClient espClient;
PubSubClient client(espClient);


void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);   
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");

      client.publish("outTopic", "this is me and only me");
      } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
       delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
}

void loop() {

}
