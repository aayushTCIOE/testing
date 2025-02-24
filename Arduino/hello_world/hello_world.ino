#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"

#include "model.h"

Adafruit_MPU6050 mpu;


namespace {
const tflite::Model *model = nullptr;
tflite::MicroInterpreter *interpreter = nullptr;
TfLiteTensor *input = nullptr;
TfLiteTensor *output = nullptr;

constexpr int kTensorArenaSize = 2000;
uint8_t tensor_arena[kTensorArenaSize];
}  

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
  model = tflite::GetModel(g_model);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    Serial.printf("Model schema version mismatch!\n");
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

  // Normalize accelerometer values
  float norm_ax = a.acceleration.x / 9.81f;  // Convert to G
  float norm_ay = a.acceleration.y / 9.81f;
  float norm_az = a.acceleration.z / 9.81f;

  // Quantize input
  input->data.int8[0] = norm_ax / input->params.scale + input->params.zero_point;
  input->data.int8[1] = norm_ay / input->params.scale + input->params.zero_point;
  input->data.int8[2] = norm_az / input->params.scale + input->params.zero_point;


  if (interpreter->Invoke() != kTfLiteOk) {
    Serial.println("Model inference failed!");
    return;
  }


  int8_t output_value = output->data.int8[0];
  float result = (output_value - output->params.zero_point) * output->params.scale;

 
  Serial.print("X: ");
  Serial.print(norm_ax);
  Serial.print(", Y: ");
  Serial.print(norm_ay);
  Serial.print(", Z: ");
  Serial.print(norm_az);
  Serial.print(" -> Model Output: ");
  Serial.println(result);

  delay(100);
}
