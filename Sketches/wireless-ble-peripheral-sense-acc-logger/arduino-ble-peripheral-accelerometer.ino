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
#include <Arduino_LSM9DS1.h>

const char* deviceServiceUuid = "19b10000-e8f2-537e-4f6c-d104768a1214";
const char* accXCharacteristicUuid = "19b10001-e8f2-537e-4f6c-d104768a1214";
const char* accYCharacteristicUuid = "19b10002-e8f2-537e-4f6c-d104768a1214";
const char* accZCharacteristicUuid = "19b10003-e8f2-537e-4f6c-d104768a1214";

void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  if (!IMU.begin()) {
    Serial.println("* Error initializing IMU sensor!");
  } 
  
  if (!BLE.begin()) {
    Serial.println("* Starting Bluetooth® Low Energy module failed!");
    while (1);
  }
  
  BLE.setLocalName("Nano 33 BLE (peripheral)"); 
  BLE.advertise();

  Serial.println("Arduino Nano 33 BLE Sense (peripheral Device)");
  Serial.println(" ");
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
    float x, y, z;
    
    if (IMU.accelerationAvailable()) {
      IMU.readAcceleration(x, y, z);

      // Convert float values to int16_t (assuming we want to keep some decimal precision)
      int16_t xInt = x * 1000;
      int16_t yInt = y * 1000;
      int16_t zInt = z * 1000;

      // Write the values to their respective characteristics
      accXChar.writeValue(xInt);
      accYChar.writeValue(yInt);
      accZChar.writeValue(zInt);

      Serial.print("Acceleration: ");
      Serial.print(x);
      Serial.print(", ");
      Serial.print(y);
      Serial.print(", ");
      Serial.println(z);
    }
    
    delay(100); // Send data every 100ms (adjust as needed)
  }
  
  Serial.println("- central device disconnected!");
}