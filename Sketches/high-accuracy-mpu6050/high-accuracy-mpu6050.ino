#include <Wire.h>

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
const float GYRO_SCALE = 65.5;     // ±500°/s range (32768 / 500)

void setup() {
  Wire.begin();
  Wire.setClock(400000);  // Set I2C clock to 400kHz for faster communication
  Serial.begin(115200);   // High baud rate for faster data transfer

  // Wake up MPU6050 and set clock source
  writeMPU6050(PWR_MGMT_1, 0x00);

  // Set accelerometer range to ±4g
  writeMPU6050(ACCEL_CONFIG, 0x08);

  // Set gyroscope range to ±500°/s
  writeMPU6050(GYRO_CONFIG, 0x08);

  delay(100);  // Wait for the sensor to stabilize
}

void loop() {
  // Read accelerometer data
  int16_t ax = readMPU6050(ACCEL_XOUT_H);
  int16_t ay = readMPU6050(ACCEL_XOUT_H + 2);
  int16_t az = readMPU6050(ACCEL_XOUT_H + 4);

  // Convert accelerometer data to g's
  float gx = ax / ACCEL_SCALE;
  float gy = ay / ACCEL_SCALE;
  float gz = az / ACCEL_SCALE;

  // Read gyroscope data
  int16_t gyrox = readMPU6050(GYRO_XOUT_H);
  int16_t gyroy = readMPU6050(GYRO_XOUT_H + 2);
  int16_t gyroz = readMPU6050(GYRO_XOUT_H + 4);

  // Convert gyroscope data to °/s
  float wx = gyrox / GYRO_SCALE;
  float wy = gyroy / GYRO_SCALE;
  float wz = gyroz / GYRO_SCALE;

  // Print data
  Serial.print("Accel (g): ");
  Serial.print(gx, 4); Serial.print(", ");
  Serial.print(gy, 4); Serial.print(", ");
  Serial.println(gz, 4);

  Serial.print("Gyro (deg/s): ");
  Serial.print(wx, 2); Serial.print(", ");
  Serial.print(wy, 2); Serial.print(", ");
  Serial.println(wz, 2);

  Serial.println();

  delay(50);  // Adjust delay based on your needs
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