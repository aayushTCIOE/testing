#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"

#include "model.h"  // Your trained TFLite model
#include "constants.h"
#include "output_handler.h"

// MPU6050 instance
Adafruit_MPU6050 mpu;

// TensorFlow Lite Globals
namespace {
const tflite::Model *model = nullptr;
tflite::MicroInterpreter *interpreter = nullptr;
TfLiteTensor *input = nullptr;
TfLiteTensor *output = nullptr;

constexpr int kTensorArenaSize = 2000;
uint8_t tensor_arena[kTensorArenaSize];
}  // namespace

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("MPU6050 initialization failed!");
    while (1);
  }
  Serial.println("MPU6050 initialized.");

  // Load the TensorFlow Lite model
  model = tflite::GetModel(g_model);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    Serial.printf("Model schema mismatch! Expected %d but got %d\n", TFLITE_SCHEMA_VERSION, model->version());
    return;
  }

  // Set up TensorFlow Lite resolver
  static tflite::MicroMutableOpResolver<1> resolver;
  if (resolver.AddFullyConnected() != kTfLiteOk) {
    Serial.println("Failed to add FullyConnected layer.");
    return;
  }

  // Create interpreter
  static tflite::MicroInterpreter static_interpreter(model, resolver, tensor_arena, kTensorArenaSize);
  interpreter = &static_interpreter;

  // Allocate memory for model
  if (interpreter->AllocateTensors() != kTfLiteOk) {
    Serial.println("Failed to allocate tensors.");
    return;
  }

  // Get input and output tensors
  input = interpreter->input(0);
  output = interpreter->output(0);

  Serial.println("Setup complete.");
}

void loop() {
  // Read MPU6050 sensor data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Prepare input for the model (modify according to your model's expected input format)
  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;
  float gx = g.gyro.x;
  float gy = g.gyro.y;
  float gz = g.gyro.z;

  Serial.printf("MPU6050 Data -> Ax: %.2f Ay: %.2f Az: %.2f | Gx: %.2f Gy: %.2f Gz: %.2f\n", ax, ay, az, gx, gy, gz);

  // Convert to model's input format (assuming float input, modify if quantized)
  input->data.f[0] = ax;
  input->data.f[1] = ay;
  input->data.f[2] = az;
  input->data.f[3] = gx;
  input->data.f[4] = gy;
  input->data.f[5] = gz;

  // Run inference
  if (interpreter->Invoke() != kTfLiteOk) {
    Serial.println("Model inference failed!");
    return;
  }

  // Get the output (modify this based on your model's output format)
  float prediction = output->data.f[0];

  // Print output
  Serial.printf("Model Output: %.2f\n", prediction);

  // Handle output (for example, detect falls)
  if (prediction > 0.5) {  
    Serial.println("Fall detected!");
    // Add buzzer or SMS alert logic here
  }

  delay(500);  // Delay before next reading
}
