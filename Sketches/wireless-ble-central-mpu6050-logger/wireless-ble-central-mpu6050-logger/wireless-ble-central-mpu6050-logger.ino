#include "arduino_secrets.h"

/*
  BLE_central_Accelerometer_Display.ino
  This program uses the ArduinoBLE library to set-up an Arduino Nano 33 BLE 
  as a central device and specifies a service and three characteristics 
  to receive raw accelerometer data (AccX, AccY, AccZ). The received data 
  is then printed to the Serial Monitor in a comma-separated format.

  The circuit:
  - Arduino Nano 33 BLE. 

  This example code is in the public domain.
*/

#include <ArduinoBLE.h>

const char* deviceServiceUuid = "19b10000-e8f2-537e-4f6c-d104768a1214";
const char* accXCharacteristicUuid = "19b10001-e8f2-537e-4f6c-d104768a1214";
const char* accYCharacteristicUuid = "19b10002-e8f2-537e-4f6c-d104768a1214";
const char* accZCharacteristicUuid = "19b10003-e8f2-537e-4f6c-d104768a1214";


BLEService accelerometerService(deviceServiceUuid); 

BLEIntCharacteristic accXCharacteristic(accXCharacteristicUuid, BLERead | BLEWrite);
BLEIntCharacteristic accYCharacteristic(accYCharacteristicUuid, BLERead | BLEWrite);
BLEIntCharacteristic accZCharacteristic(accZCharacteristicUuid, BLERead | BLEWrite);

void setup() {
  Serial.begin(115200);
  while (!Serial);  
  
  if (!BLE.begin()) {
    Serial.println("- Starting BluetoothÂ® Low Energy module failed!");
    while (1);
  }

  BLE.setLocalName("Arduino Nano 33 BLE (central)");
  BLE.setAdvertisedService(accelerometerService);

  accelerometerService.addCharacteristic(accXCharacteristic);
  accelerometerService.addCharacteristic(accYCharacteristic);
  accelerometerService.addCharacteristic(accZCharacteristic);

  BLE.addService(accelerometerService);

  accXCharacteristic.writeValue(0);
  accYCharacteristic.writeValue(0);
  accZCharacteristic.writeValue(0);

  BLE.advertise();
  Serial.println("Nano 33 BLE (central Device)");
  Serial.println("Waiting for connection...");
}

void loop() {
  BLEDevice peripheral = BLE.central(); // Some weird naming convention in arduino

  if (peripheral) {
    Serial.println("* Connected to peripheral device!");
    Serial.print("* Device MAC address: ");
    Serial.println(peripheral.address());
    Serial.println("* Accelerometer data (X, Y, Z):");

    while (peripheral.connected()) {
      if (accXCharacteristic.written() || accYCharacteristic.written() || accZCharacteristic.written()) {
        displayAccelerometerData();
      }
    }
    
    Serial.println("* Disconnected from peripheral device!");
    Serial.println("Waiting for connection...");
  }
}

void displayAccelerometerData() {
  // Read the raw int16_t values
  int16_t xRaw = accXCharacteristic.value();
  int16_t yRaw = accYCharacteristic.value();
  int16_t zRaw = accZCharacteristic.value();

  // Convert back to float (remember, we multiplied by 1000 on the peripheral side)
  // float x = xRaw / 1000.0;
  // float y = yRaw / 1000.0;
  // float z = zRaw / 1000.0;

  // Print the values in the desired format
  Serial.print(xRaw);  // 2 decimal places
  Serial.print(",");
  Serial.print(yRaw);
  Serial.print(",");
  Serial.println(zRaw);
}