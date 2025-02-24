// #include <ESP8266WiFi.h>
// #include <PubSubClient.h>


// const char* ssid = "Who-Dat_2.4";
// const char* password = "@class@object@18";

// const char* mqtt-server = "broker.hivemq.com"; 


// WiFiClient espClient;
// PubSubClient client(espClient);

// unsigned long lastMsg = 0;
// #define MSG_BUFFER_SIZE	(50)
// char msg[MSG_BUFFER_SIZE];
// // int value = 0;

// void setup_wifi() {
//   delay(10);
//   Serial.println();
//   Serial.print("Connecting to ");
//   Serial.println(ssid);

//   WiFi.mode(WIFI_STA);
//   WiFi.begin(ssid, password);

//   while (WiFi.status() != WL_CONNECTED) {
//     delay(500);
//     Serial.print(".");
//   }
//   randomSeed(micros());
//   Serial.println("");
//   Serial.println("WiFi connected");
//   Serial.println("IP address: ");
//   Serial.println(WiFi.localIP());
// }


// void reconnect() {
//   // Loop until we're reconnected
//   while (!client.connected()) {
//     Serial.print("Attempting MQTT connection...");
//     // Create a random client ID
//     String clientId = "ESP8266Client-";
//     clientId += String(random(0xffff), HEX);
//     // Attempt to connect
//     if (client.connect(clientId.c_str())) {
//       Serial.println("connected");
//       // Once connected, publish an announcement...
//       client.publish("outTopic", "hello world");
//       // ... and resubscribe
//       client.subscribe("inTopic");
//     } else {
//       Serial.print("failed, rc=");
//       Serial.print(client.state());
//       Serial.println(" try again in 5 seconds");
//       // Wait 5 seconds before retrying
//       delay(5000);
//     }
//   }
// }

// void setup() {
//   Serial.begin(115200);
//   setup_wifi();
//   client.setServer(mqtt_server, 1883);
//   client.setCallback(callback);
// }

// void loop() {

//   if (!client.connected()) {
//     reconnect();
//   }
//   client.loop();

//   unsigned long now = millis(); 
//     snprintf (msg, MSG_BUFFER_SIZE, "hello world #%ld", value);
//     Serial.print("Publish message: ");
//     Serial.println(msg);
//     client.publish("outTopic", msg);
  
// }
#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid = "Who-Dat_2.4";
const char* password = "@class@object@18";

const char* mqtt_server = "192.168.1.68"; 

WiFiClient espClient;
PubSubClient client(espClient);

#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);

    if (client.connect(clientId.c_str())) {
      Serial.println("connected");

      
      client.publish("outTopic", "Hello World");
      Serial.println("Published: Hello World");
      
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" trying again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  reconnect();  // ✅ Send "Hello World" only once when connected
}

void loop() {
 
}
