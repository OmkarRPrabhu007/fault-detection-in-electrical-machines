/*
  BLE_peripheral_Device_Accelerometer_Raw.ino

  This program uses the ArduinoBLE library to set-up an Arduino Nano 33 BLE Sense as a peripheral device
  and looks for a central device seeking a specified service and three characteristics.
  If found, it writes the raw accelerometer data (AccX, AccY, AccZ) from the on-board LSM9DS1 sensor 
  to these characteristics.

  The circuit:
  - Arduino Nano 33 BLE Sense. 

  This example code is in the public domain.
*/

#include <ArduinoBLE.h>
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
#define SAMPLING_PERIOD_MS 1000/SAMPLING_FREQ_HZ // Sampling period (ms)
#define NUM_SAMPLES 200 // 1000 samples at 1000 Hz is 1 sec window

unsigned long timestamp;
unsigned long start_timestamp;

const char* deviceServiceUuid = "19b10000-e8f2-537e-4f6c-d104768a1214";
const char* accXCharacteristicUuid = "19b10001-e8f2-537e-4f6c-d104768a1214";
const char* accYCharacteristicUuid = "19b10002-e8f2-537e-4f6c-d104768a1214";
const char* accZCharacteristicUuid = "19b10003-e8f2-537e-4f6c-d104768a1214";

void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  if (!BLE.begin()) {
    Serial.println("* Starting Bluetooth® Low Energy module failed!");
    while (1);
  }
  
  BLE.setLocalName("Nano 33 BLE (peripheral)"); 
  BLE.advertise();

  Serial.println("Arduino Nano 33 BLE Sense (peripheral Device)");
  Serial.println(" ");

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
  connectTocentral();
}

void connectTocentral(){
  BLEDevice central;
  
  Serial.println("- Discovering central device...");

  do
  {
    BLE.scanForUuid(deviceServiceUuid);
    central = BLE.available();
  } while (!central);
  
  if (central) {
    Serial.println("* central device found!");
    Serial.print("* Device MAC address: ");
    Serial.println(central.address());
    Serial.print("* Device name: ");
    Serial.println(central.localName());
    Serial.print("* Advertised service UUID: ");
    Serial.println(central.advertisedServiceUuid());
    Serial.println(" ");
    BLE.stopScan();
    controlcentral(central);
  }
}

void controlcentral(BLEDevice central) {
  Serial.println("- Connecting to central device...");

  if (central.connect()) {
    Serial.println("* Connected to central device!");
    Serial.println(" ");
  } else {
    Serial.println("* Connection to central device failed!");
    Serial.println(" ");
    return;
  }

  Serial.println("- Discovering central device attributes...");
  if (central.discoverAttributes()) {
    Serial.println("* central device attributes discovered!");
    Serial.println(" ");
  } else {
    Serial.println("* central device attributes discovery failed!");
    Serial.println(" ");
    central.disconnect();
    return;
  }

  BLECharacteristic accXChar = central.characteristic(accXCharacteristicUuid);
  BLECharacteristic accYChar = central.characteristic(accYCharacteristicUuid);
  BLECharacteristic accZChar = central.characteristic(accZCharacteristicUuid);
    
  if (!accXChar || !accYChar || !accZChar) {
    Serial.println("* central device does not have all required characteristics!");
    central.disconnect();
    return;
  } else if (!accXChar.canWrite() || !accYChar.canWrite() || !accZChar.canWrite()) {
    Serial.println("* central does not have writable characteristics!");
    central.disconnect();
    return;
  }
  
  while (central.connected()) {
  
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

      accXChar.writeValue(ax);
      accYChar.writeValue(ay);
      accZChar.writeValue(az);
      
      // Print data
      // Serial.println(String(timestamp - start_timestamp, DEC) + "," + String(gx,2) + "," + String(gy,2) + "," + String(gz,2));
      Serial.println(String(timestamp - start_timestamp) + "," + String(ax) + "," + String(ay) + "," + String(az));

      // Wait just long enough for our sampling period
      while (millis() < timestamp + SAMPLING_PERIOD_MS);
  }

  // Print empty line to transmit termination of recording
  // Serial.println();

  // Turn on LED to show we're done
  digitalWrite(LED_R_PIN, HIGH);

  // Wait before repeating the collection process
  delay(DELAY_COLLECTION);
  }
  
  Serial.println("- central device disconnected!");
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