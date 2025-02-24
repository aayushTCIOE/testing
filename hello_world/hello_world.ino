

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"

#include "model.h"  // Include your trained model

Adafruit_MPU6050 mpu;

// Globals
namespace {
const tflite::Model *model = nullptr;
tflite::MicroInterpreter *interpreter = nullptr;
TfLiteTensor *input = nullptr;
TfLiteTensor *output = nullptr;

constexpr int kTensorArenaSize = 4000;  // Adjust for your model size
uint8_t tensor_arena[kTensorArenaSize];
}  // namespace

#define BATCH_SIZE 20  // Number of samples to store

// Buffer to store 50 sets of accelerometer + gyroscope data
float input_buffer[BATCH_SIZE][6];  // 50 samples of 6 values (ax, ay, az, gx, gy, gz)
int sample_count = 0;  // Tracks number of collected samples

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip!");
    while (1);
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(100);

  // Load model
  model = tflite::GetModel(model_tflite);  // model_data should be the byte array of the converted model
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("Model schema version mismatch!");
    return;
  }

  static tflite::MicroMutableOpResolver<1> resolver;
  if (resolver.AddFullyConnected() != kTfLiteOk) {
    return;
  }

  static tflite::MicroInterpreter static_interpreter(model, resolver, tensor_arena, kTensorArenaSize);
  interpreter = &static_interpreter;

  if (interpreter->AllocateTensors() != kTfLiteOk) {
    Serial.println("AllocateTensors() failed!");
    return;
  }

  input = interpreter->input(0);
  output = interpreter->output(0);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Normalize accelerometer values (±2g -> 16384 LSB)
  float norm_ax = a.acceleration.x / 16384.0f;
  float norm_ay = a.acceleration.y / 16384.0f;
  float norm_az = a.acceleration.z / 16384.0f;

  // Normalize gyroscope values (±250°/s -> 131 LSB)
  float norm_gx = g.gyro.x / 131.0f;
  float norm_gy = g.gyro.y / 131.0f;
  float norm_gz = g.gyro.z / 131.0f;

  // Store values in buffer
  input_buffer[sample_count][0] = norm_ax;
  input_buffer[sample_count][1] = norm_ay;
  input_buffer[sample_count][2] = norm_az;
  input_buffer[sample_count][3] = norm_gx;
  input_buffer[sample_count][4] = norm_gy;
  input_buffer[sample_count][5] = norm_gz;

  sample_count++;

  // Check if we have collected 50 samples
  if (sample_count >= BATCH_SIZE) {
    Serial.println("Processing batch...");

    // Process each sample sequentially
    for (int i = 0; i < BATCH_SIZE; i++) {
      // Store data in the input tensor
      input->data.f[0] = input_buffer[i][0];
      input->data.f[1] = input_buffer[i][1];
      input->data.f[2] = input_buffer[i][2];
      input->data.f[3] = input_buffer[i][3];
      input->data.f[4] = input_buffer[i][4];
      input->data.f[5] = input_buffer[i][5];

      // Run inference
      TfLiteStatus invoke_status = interpreter->Invoke();
      if (invoke_status != kTfLiteOk) {
        Serial.print("Model inference failed with error code: ");
        Serial.println(invoke_status);
        return;
      }

      // Obtain output
      float output_value = output->data.f[0];

      // Convert to boolean (adjust threshold if needed)
      bool fall_detected = output_value > 0.5;

      // Print output
      Serial.print("Sample ");
      Serial.print(i);
      Serial.print(" | Model Output: ");
      Serial.print(output_value);
      Serial.print(" | Fall Detected: ");
      Serial.println(fall_detected ? "YES" : "NO");
    }

    // Reset sample counter for the next batch
    sample_count = 0;
  }

  delay(100);  // Adjust delay as needed
}
