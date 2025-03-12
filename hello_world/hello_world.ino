// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>
// #include <Wire.h>

// #include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
// #include "tensorflow/lite/micro/micro_interpreter.h"
// #include "tensorflow/lite/micro/system_setup.h"
// #include "tensorflow/lite/schema_schema_generated.h"

// #include "model.h" 

// Adafruit_MPU6050 mpu;

// namespace {
// const tflite::Model *model = nullptr;
// tflite::MicroInterpreter *interpreter = nullptr;
// TfLiteTensor *input = nullptr;
// TfLiteTensor *output = nullptr;
// constexpr int kTensorArenaSize = 4000; 
// uint8_t tensor_arena[kTensorArenaSize];
// }  

// #define BATCH_SIZE 20  

// float input_buffer[BATCH_SIZE][6]; 
// int sample_count = 0;  

// void setup() {
//   Serial.begin(115200);
//   while (!Serial);

//   if (!mpu.begin()) {
//     Serial.println("Failed to find MPU6050 chip!");
//     while (1);
//   }
 
//   mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
//   mpu.setGyroRange(MPU6050_RANGE_250_DEG);
//   mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

//   delay(100);

//   model = tflite::GetModel(model_tflite); 
//   if (model->version() != TFLITE_SCHEMA_VERSION) {
//     Serial.println("Model schema version mismatch!");
//     return;
//   }

//   static tflite::MicroMutableOpResolver<1> resolver;
//   if (resolver.AddFullyConnected() != kTfLiteOk) {
//     return;
//   }

//   static tflite::MicroInterpreter static_interpreter(model, resolver, tensor_arena, kTensorArenaSize);
//   interpreter = &static_interpreter;

//   if (interpreter->AllocateTensors() != kTfLiteOk) {
//     Serial.println("AllocateTensors() failed!");
//     return;
//   }

//   input = interpreter->input(0);
//   output = interpreter->output(0);
// }

// bool isFall() {
//   sensors_event_t a, g, temp;
//   mpu.getEvent(&a, &g, &temp);

//   float norm_ax = a.acceleration.x / 16384.0f;
//   float norm_ay = a.acceleration.y / 16384.0f;
//   float norm_az = a.acceleration.z / 16384.0f;
//   float norm_gx = g.gyro.x / 131.0f;
//   float norm_gy = g.gyro.y / 131.0f;
//   float norm_gz = g.gyro.z / 131.0f;

//   input_buffer[sample_count][0] = norm_ax;
//   input_buffer[sample_count][1] = norm_ay;
//   input_buffer[sample_count][2] = norm_az;
//   input_buffer[sample_count][3] = norm_gx;
//   input_buffer[sample_count][4] = norm_gy;
//   input_buffer[sample_count][5] = norm_gz;

//   sample_count++;

//   if (sample_count >= BATCH_SIZE) {
//     Serial.println("Processing batch...");
//     for (int i = 0; i < BATCH_SIZE; i++) {
//       for (int j = 0; j < 6; j++) {
//         input->data.f[j] = input_buffer[i][j];
//       }

//       if (interpreter->Invoke() != kTfLiteOk) {
//         Serial.println("Model inference failed!");
//         return false;
//       }

//       float output_value = output->data.f[0];
//       bool fall_detected = output_value > 0.7;

//       if (fall_detected) {
//         return true;
//       }
//     }
//     sample_count = 0;
//   }
//   return false;
// }

// void loop() {
//   bool fall = mpuAndModel();
//   if (fall) {
//     Serial.println("Fall detected! Taking action...");
//   }
//   delay(100);
// }



#define TINY_GSM_MODEM_SIM800  
#include <TinyGsmClient.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "model.h"

const int buzzerPin = 27;
const int switchPin = 14;
#define RX_PIN 3 
#define TX_PIN 1
#define BAUD_RATE 9600
#define GPS_RX 16
#define GPS_TX 17

HardwareSerial gsmSerial(2);
HardwareSerial gpsSerial(1);
TinyGsm modem(gsmSerial);
TinyGPSPlus gps;
Adafruit_MPU6050 mpu;

bool buzzerState = false;
bool buttonPressed = false;
double latitude;
double longitude;

namespace {
const tflite::Model *model = nullptr;
tflite::MicroInterpreter *interpreter = nullptr;
TfLiteTensor *input = nullptr;
TfLiteTensor *output = nullptr;
constexpr int kTensorArenaSize = 4000; 
uint8_t tensor_arena[kTensorArenaSize];
}

#define BATCH_SIZE 20  
float input_buffer[BATCH_SIZE][6]; 
int sample_count = 0;

bool mpuAndModel() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float norm_ax = a.acceleration.x / 16384.0f;
    float norm_ay = a.acceleration.y / 16384.0f;
    float norm_az = a.acceleration.z / 16384.0f;
    float norm_gx = g.gyro.x / 131.0f;
    float norm_gy = g.gyro.y / 131.0f;
    float norm_gz = g.gyro.z / 131.0f;

    input_buffer[sample_count][0] = norm_ax;
    input_buffer[sample_count][1] = norm_ay;
    input_buffer[sample_count][2] = norm_az;
    input_buffer[sample_count][3] = norm_gx;
    input_buffer[sample_count][4] = norm_gy;
    input_buffer[sample_count][5] = norm_gz;

    sample_count++;
    if (sample_count >= BATCH_SIZE) {
        for (int i = 0; i < BATCH_SIZE; i++) {
            for (int j = 0; j < 6; j++) {
                input->data.f[j] = input_buffer[i][j];
            }
            if (interpreter->Invoke() != kTfLiteOk) {
                return false;
            }
            if (output->data.f[0] > 0.5) {
                return true;
            }
        }
        sample_count = 0;
    }
    return false;
}

void getGPSLocation() {
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
        if (gps.location.isUpdated()) {
            latitude = gps.location.lat();
            longitude = gps.location.lng();
        }
    }
}

void sendSMS() {
    String phoneNumber = "+9779768432105";
    getGPSLocation();
    String message = "Fall Detected! Location: " + 
                     String(latitude, 6) + ", " + String(longitude, 6) + 
                     " (Google Maps: https://maps.google.com/?q=" + 
                     String(latitude, 6) + "," + String(longitude, 6) + ")";
    modem.sendSMS(phoneNumber, message);
}

void setup() {
    pinMode(buzzerPin, OUTPUT);
    pinMode(switchPin, INPUT_PULLUP);
    Serial.begin(115200);
    gsmSerial.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
    gpsSerial.begin(BAUD_RATE, SERIAL_8N1, GPS_RX, GPS_TX);
    
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip!");
        while (1);
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    model = tflite::GetModel(model_tflite);
    static tflite::MicroMutableOpResolver<1> resolver;
    resolver.AddFullyConnected();
    static tflite::MicroInterpreter static_interpreter(model, resolver, tensor_arena, kTensorArenaSize);
    interpreter = &static_interpreter;
    interpreter->AllocateTensors();
    input = interpreter->input(0);
    output = interpreter->output(0);
    
    if (!modem.restart()) {
        Serial.println("Failed to restart SIM800L!");
        return;
    }
}

void loop() {
    if (mpuAndModel() && !buzzerState) {
        digitalWrite(buzzerPin, HIGH);
        buzzerState = true;

        unsigned long startTime = millis();  
        while (millis() - startTime < 10000) {
            if (digitalRead(switchPin) == LOW) {
                delay(200);
                if (digitalRead(switchPin) == LOW) {
                    digitalWrite(buzzerPin, LOW);
                    buttonPressed = true;
                    while (digitalRead(switchPin) == LOW);
                    delay(200);
                    break;
                }
            }
        }
        if (!buttonPressed) {
            sendSMS();  
        }
        buzzerState = false;
        buttonPressed = false;
    }
    delay(1000);
}


