/*
 * Code adapted from MPU6050_DMP6_ESP32
 * 
 * Author: Ethan Joyce
 */

#define BLUETOOTH_EN

#include "Wire.h"
#include "MPU6050_6Axis_MotionApps20.h"

#ifdef BLUETOOTH_EN
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#endif // BLUETOOTH_EN


#define SDA_PIN         4
#define SCL_PIN         5
#define INTERRUPT_PIN   15

#define DT              0.01 /* 10ms aka 100Hz */

// Rep detection
#define REP_DETECT_V_THRESHOLD   0.10 /* m/s, used for -ve and +ve velocities */


#ifdef BLUETOOTH_EN

#define MAX_MTU                 517

#define SERVICE_UUID            "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX  "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;

uint8_t macAddress[] = {0x5C, 0x60, 0xD3, 0x6C, 0x82, 0x1C - 0x02}; // Subtract two because actual address is +2 for some reason?

#endif // BLUETOOTH_EN

MPU6050 mpu;

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint8_t packetSize;
uint8_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

float velocity = 0; // Cumulative sum from integration, units of m/s

// Rep detection algorithm
uint8_t rep_stage = 0; // Rep stages: 0 (no rep), 1 (-ve vel), 2 (+ve vel)

float max_accel = 0;
float min_accel = 0;
float max_velocity = 0;
float min_velocity = 0;


#ifdef BLUETOOTH_EN
  /*
   * Bluetooth Callbacks
   */
  class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer *pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer *pServer) {
      deviceConnected = false;
    };
  };

  class MyCallbacks: public BLECharacteristicCallbacks {
      void onWrite(BLECharacteristic *pCharacteristic) {
        std::string rxValue = pCharacteristic->getValue();

        if (rxValue.length() > 0) {
          Serial.println("*********");
          Serial.print("Received Value: ");
          for (int i = 0; i < rxValue.length(); i++)
            Serial.print(rxValue[i]);

          Serial.println();
          Serial.println("*********");
        }
      }
  };

#endif // BLUETOOTH_EN

/*
 * Interrupt Detection Routine
 */
volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}

#ifdef BLUETOOTH_EN
  void setupBLE() {
    esp_base_mac_addr_set(macAddress);
    
    BLEDevice::init("OpenBST UART Service");

    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);

    pTxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_TX,
        BLECharacteristic::PROPERTY_NOTIFY
      );
    pTxCharacteristic->addDescriptor(new BLE2902());

    BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_RX,
        BLECharacteristic::PROPERTY_WRITE
      );
    pRxCharacteristic->setCallbacks(new MyCallbacks());

    pService->start();
    pServer->getAdvertising()->start();
    Serial.println("Waiting for a BLE client connection...");
  }
#endif // BLUETOOTH_EN

void setup() {
  Serial.begin(115200);
  Serial.println("Serial connection initialized");

  #ifdef BLUETOOTH_EN
    setupBLE();
  #endif // BLUETOOTH_EN

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000); // 400 kHz I2C clock

  Serial.println("Initializing I2C devices...");
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  Serial.println("Testing device connections...");
  Serial.println(mpu.testConnection() ? "MPU6050 connection successfull" : "MPU6050 connection failed");

  Serial.println("Initializing DMP...");
  devStatus = mpu.dmpInitialize();

  // Offsets found using calibration sketch
  mpu.setXAccelOffset(-6479);
  mpu.setYAccelOffset(579);
  mpu.setZAccelOffset(1596);
  mpu.setXGyroOffset(45);
  mpu.setYGyroOffset(-28);
  mpu.setZGyroOffset(-17);

  if (devStatus == 0) {
    Serial.println("Enabling DMP...");
    mpu.setDMPEnabled(true);

    Serial.println("Enabling interrupt detection...");
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    Serial.println("DMP ready! Waiting for first interrupt...");
    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();
  } else { // Error occurred
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    Serial.print("DMP Initialization failed (code ");
    Serial.print(devStatus);
    Serial.println(")");
  }
}

#ifdef BLUETOOTH_EN
  void ble_transmitrep(char *str) {
    pTxCharacteristic->setValue(str);
    pTxCharacteristic->notify();
  }
  /*void ble_println(String str) {
    char buff[1024];
    str.toCharArray(buff, 1024);

    pTxCharacteristic->setValue(buff);
    pTxCharacteristic->notify();
  }

  void ble_println(char *str) {
    pTxCharacteristic->setValue(str);
    pTxCharacteristic->notify();
  }*/
#endif // BLUETOOTH_EN

void loop() {
  #ifdef BLUETOOTH_EN
    /*
     * BLE Checking Logic
     */
    if (!deviceConnected && oldDeviceConnected) {
      delay(500);
      pServer->startAdvertising(); // restart advertising
      Serial.println("Restarted advertising...");
      oldDeviceConnected = deviceConnected;
      return;
    } else if (!deviceConnected) {
      return; // Wait for device to be connected
    }
  
    if (deviceConnected && !oldDeviceConnected) {
      Serial.println("Device connected.");
      oldDeviceConnected = deviceConnected;
      
      // Set our MTU to max to allow notify messages > 20 chars
      pServer->updatePeerMTU(pServer->getConnId(), MAX_MTU);

      //ble_println("Connected."); // NECESSARY to send something to init connection
    }
  #endif // BLUETOOTH_EN


  /*
   * DMP Logic
   */
  
  if (!dmpReady) return; // Wait for DMP

  // Wait for interrupt or packet
  if (!mpuInterrupt && fifoCount < packetSize) return;

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  fifoCount = mpu.getFIFOCount();

  // Check for overflow
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    Serial.println("FIFO overflow!");

    // Otherwise, check for DMP data ready interrupt
  } else if (mpuIntStatus & 0x02) {
    // Wait for 
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    mpu.getFIFOBytes(fifoBuffer, packetSize);

    fifoCount -= packetSize;

    static int i = 300;
    if (i-- > 0) return; // Wait for short delay

    // Read direction of gravity, and raw acceleration
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetAccel(&aa, fifoBuffer);

    // Apply offsets for X and Y accel -- IF READINGS ARE WRONG, CHECK THESE OFFSETS
    // OLD: aa.x -= 877;
    // OLD: aa.y -= -777;
    aa.x -= -380;
    aa.y -= -870;

    /*Serial.print("aa\t");
    Serial.print(aa.x * 9.81/8192.0);
    Serial.print("\t");
    Serial.print(aa.y * 9.81/8192.0);
    Serial.print("\t");
    Serial.println(aa.z * 9.81/8192.0);*/

    // Convert to "true world" z acceleration
    // OLD formula: float accelZ = (aa.z*cos(euler[2]) - aa.y*sin(euler[2])) * 9.81/8192.0 - 9.81;
    float accelZ = (aa.x*gravity.x + aa.y*gravity.y + aa.z*gravity.z) * 9.81/8192.0 - 9.81;
    //Serial.println(accelZ);

    // Next up, filter the accelerometer data (note, our polling operates at 100Hz or T=10ms)
    float b[] = { 0.0445, 0.0891, 0.0445 };
    float a[] = { 1.3208, -0.4989 };
    
    static float y1 = 0, y2 = 0, x1 = 0, x2 = 0;

    float x = accelZ;
    float y = a[0]*y1 + a[1]*y2 + b[0]*x + b[1]*x1 + b[2]*x2;

    y2 = y1;
    y1 = y;
    x2 = x1;
    x1 = x;

    //Serial.print(y);       // filtered
    //Serial.print("\t");
    //Serial.println(x);   // unfiltered

    // Greatly reduce steady state error by applying gradient function (this is kinda hacky)
    float y_processed;
    if (y < 0)
      y_processed = -(sqrt(y*y + 0.2*0.2) - 0.2);
    else
      y_processed = sqrt(y*y + 0.2*0.2) - 0.2;

    // Integrate to find velocity
    velocity += y_processed * DT;
    
    // Leak our cumulative sum to prevent steady state error
    velocity -= velocity*0.01*1.0/(1.0+abs(y_processed));

    Serial.print(y_processed);
    Serial.print("\t");
    Serial.println(velocity);

    // Rep starts when velocity goes negative
    if (rep_stage == 0 && velocity < -REP_DETECT_V_THRESHOLD) rep_stage = 1;

    if (rep_stage != 0) {
      // Update rep statistics
      if (y_processed > max_accel) max_accel = y_processed;
      if (y_processed < min_accel) min_accel = y_processed;
      if (velocity > max_velocity) max_velocity = velocity;
      if (velocity < min_velocity) min_velocity = velocity;

      if (rep_stage == 1 && velocity > REP_DETECT_V_THRESHOLD)
        rep_stage = 2; // Move to next stage when thresh passed

      if (rep_stage == 2 && velocity < REP_DETECT_V_THRESHOLD) {
        rep_stage = 0; // Rep is over when velocity settles back

        // Notify our client device
#ifdef BLUETOOTH_EN
        char strBuff[MAX_MTU - 3];

        sprintf(strBuff, "%.02f %0.02f %0.02f %0.02f",
          max_velocity,
          min_velocity,
          max_accel,
          min_accel
        );
        ble_transmitrep(strBuff);
        /*char strBuff[512];

        ble_println("---- REP ----");
        
        sprintf(strBuff, "Max accel: %f", max_accel);
        ble_println(strBuff);

        sprintf(strBuff, "Min accel: %f", min_accel);
        ble_println(strBuff);

        sprintf(strBuff, "Max vel: %f", max_velocity);
        ble_println(strBuff);

        sprintf(strBuff, "Min vel: %f", min_velocity);
        ble_println(strBuff);

        ble_println("----====----");*/
#endif // BLUETOOTH_EN

        // Reset rep-specific params
        max_accel = 0;
        min_accel = 0;
        max_velocity = 0;
        min_velocity = 0;
      }
    }
  }
}

/*#ifdef BLUETOOTH_EN
        ble_println("----------");

        char strBuff[512];
      
        sprintf(strBuff, "Largest accel: %f", largestAccel);
        ble_println(strBuff);
      
        sprintf(strBuff, "Net accel: %f", netAccel);
        ble_println(strBuff);
      
        sprintf(strBuff, "Largest vel: %f", largestVel);
        ble_println(strBuff);
      
        sprintf(strBuff, "Time passed: %d", timePassed);
        ble_println(strBuff);

        sprintf(strBuff, "Rep or not?: %s", largestAccel > 2.0 ? "true" : "false");
        ble_println(strBuff);

        ble_println("----------");
      #endif // BLUETOOTH_EN*/
