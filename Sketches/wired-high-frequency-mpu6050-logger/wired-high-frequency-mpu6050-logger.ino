#include<Wire.h>

// MPU6050 registers
#define MPU6050_ADDR         0x68
#define ACCEL_CONFIG        0x1C
#define GYRO_CONFIG        0x1B
#define ACCEL_XOUT_H       0x3B
#define TEMP_OUT_H         0x41
#define GYRO_XOUT_H        0x43
#define PWR_MGMT_1        0x6B

// Scale factors
const float ACCEL_SCALE = 8192.0;  // ±4g range (2^15 / 4)

// Settings
#define LED_R_PIN 22 // Red LED pin
#define DELAY_COLLECTION 50 // Delay (ms) between collections

// Constants
#define SAMPLING_FREQ_HZ 1000 // Sampling frequency (Hz)
#define SAMPLING_PERIOD_MS 1000 / SAMPLING_FREQ_HZ // Sampling period (ms)
#define NUM_SAMPLES 200 // 1000 samples at 1000 Hz is 1 sec window

unsigned long timestamp;
unsigned long start_timestamp;

void setup() {
  // Enable LED pin (RGB LEDs are active low)
  pinMode(LED_R_PIN, OUTPUT);
  digitalWrite(LED_R_PIN, HIGH);

  Wire.begin();
  Wire.setClock(3400000);  // Set I2C clock to 400kHz for faster communication
  Serial.begin(115200);   // High baud rate for faster data transfer

  // Wake up MPU6050 and set clock source
  writeMPU6050(PWR_MGMT_1, 0x00);

  // Set accelerometer range to ±4g
  writeMPU6050(ACCEL_CONFIG, 0x08);

  delay(100);  // Wait for the sensor to stabilize
}

void loop() {

  // Turn off LED to show we're recording (RGB LED is active low)
  pinMode(LED_R_PIN, OUTPUT);
  digitalWrite(LED_R_PIN, LOW);

  // Print header
  Serial.println("timestamp,accX,accY,accZ");
  
  // Record samples in buffer
  start_timestamp = millis();
  for (int i = 0; i < NUM_SAMPLES; i++) {
    
    // Take timestamp so we can hit our target frequency
    timestamp = millis();

    // Read accelerometer data
    int16_t ax = readMPU6050(ACCEL_XOUT_H);
    int16_t ay = readMPU6050(ACCEL_XOUT_H + 2);
    int16_t az = readMPU6050(ACCEL_XOUT_H + 4);

    // // Convert accelerometer data to g's
    // float gx = ax / ACCEL_SCALE;
    // float gy = ay / ACCEL_SCALE;
    // float gz = az / ACCEL_SCALE;

    // Print data
    // Serial.println(String(timestamp - start_timestamp, DEC) + "," + String(gx,2) + "," + String(gy,2) + "," + String(gz,2));
    Serial.println(String(timestamp - start_timestamp) + "," + String(ax) + "," + String(ay) + "," + String(az));

    // Wait just long enough for our sampling period
    while (millis() < timestamp + SAMPLING_PERIOD_MS);
  }

  // Print empty line to transmit termination of recording
  Serial.println();

  // Turn on LED to show we're done
  digitalWrite(LED_R_PIN, HIGH);

  // Wait before repeating the collection process
  delay(DELAY_COLLECTION);
}

void writeMPU6050(byte reg, byte data) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission(true);
}

int16_t readMPU6050(byte reg) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  
  Wire.requestFrom(MPU6050_ADDR, 2, true);
  return (Wire.read() << 8) | Wire.read();
}

