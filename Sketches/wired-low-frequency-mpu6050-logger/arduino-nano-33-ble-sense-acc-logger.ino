/*** Magic Wand Data Collection (continuous) ***/

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Settings
#define LED_R_PIN 22 // Red LED pin
#define DELAY_COLLECTION 50 // Delay (ms) between collections

// Constants
#define SAMPLING_FREQ_HZ 1000 // Sampling frequency (Hz)
#define SAMPLING_PERIOD_MS 1000/SAMPLING_FREQ_HZ // Sampling period (ms)
#define NUM_SAMPLES 200 // 1000 samples at 1000 Hz is 1 sec window

// Instantiate MPU6050 object
Adafruit_MPU6050 mpu;

void setup() {
  // Enable LED pin (RGB LEDs are active low)
  pinMode(LED_R_PIN, OUTPUT);
  digitalWrite(LED_R_PIN, HIGH);

  // Start serial
  Serial.begin(115200);

  // Initialize MPU6050 sensor
  if (mpu.begin()) {
    Serial.println("Initialised MPU6050 chip");
    // while (1);
  }

  Serial.println(SAMPLING_PERIOD_MS);
  

  // Set accelerometer range
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  // Set gyroscope range
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
}

void loop() {
  float mpu_acc_x, mpu_acc_y, mpu_acc_z;
  float mpu_gyr_x, mpu_gyr_y, mpu_gyr_z;
  unsigned long timestamp;
  unsigned long start_timestamp;

  // Turn on LED to show we're recording (RGB LED is active low)
  pinMode(LED_R_PIN, OUTPUT);
  digitalWrite(LED_R_PIN, LOW);

  // Print header
  Serial.println("timestamp,accX,accY,accZ,gyrX,gyrY,gyrZ");
  
  // Record samples in buffer
  start_timestamp = millis();
  for (int i = 0; i < NUM_SAMPLES; i++) {
    // Take timestamp so we can hit our target frequency
    timestamp = millis();

    // Read accelerometer and gyroscope data from the MPU6050 sensor
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    mpu_acc_x = a.acceleration.x;
    mpu_acc_y = a.acceleration.y;
    mpu_acc_z = a.acceleration.z;

    mpu_gyr_x = g.gyro.x;
    mpu_gyr_y = g.gyro.y;
    mpu_gyr_z = g.gyro.z;

    // Print CSV data with timestamp
    Serial.print(timestamp - start_timestamp);
    Serial.print(",");
    Serial.print(mpu_acc_x);
    Serial.print(",");
    Serial.print(mpu_acc_y);
    Serial.print(",");
    Serial.print(mpu_acc_z);
    Serial.print(",");
    Serial.print(mpu_gyr_x);
    Serial.print(",");
    Serial.print(mpu_gyr_y);
    Serial.print(",");
    Serial.println(mpu_gyr_z);

    // Wait just long enough for our sampling period
    while (millis() < timestamp + SAMPLING_PERIOD_MS);
  }

  // Print empty line to transmit termination of recording
  Serial.println();

  // Turn off LED to show we're done
  digitalWrite(LED_R_PIN, HIGH);

  // Wait before repeating the collection process
  delay(DELAY_COLLECTION);
}