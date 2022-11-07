// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu[] = {0x68, 0x69};
//MPU6050 mpu(0x69); // <-- use for AD0 high

#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus[] = {0, 0};   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize[] = {0, 0};    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  for (byte b = 0; b < 2; b++)
  {
    mpu[b].initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    Serial.print("MPU#");
    Serial.print(b);
    Serial.println(":");
    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu[b].testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu[b].dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu[b].setXGyroOffset(220);
    mpu[b].setYGyroOffset(76);
    mpu[b].setZGyroOffset(-85);
    mpu[b].setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
      // Calibration Time: generate offsets and calibrate our MPU6050
      mpu[b].CalibrateAccel(6);
      mpu[b].CalibrateGyro(6);
      mpu[b].PrintActiveOffsets();
      // turn on the DMP, now that it's ready
      Serial.println(F("Enabling DMP ..."));
      mpu[b].setDMPEnabled(true);

      // enable Arduino interrupt detection
      Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
      Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
      Serial.println(F(")..."));
      attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
      mpuIntStatus[b] = mpu[b].getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      Serial.println(F("DMP ready! Waiting for first interrupt..."));
      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize[b] = mpu[b].dmpGetFIFOPacketSize();
    } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
    }
  }
  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
}


void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  for (byte b = 0; b < 2; b++) {
    // read a packet from FIFO
    if (mpu[b].dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
#ifdef OUTPUT_READABLE_YAWPITCHROLL
      // display Euler angles in degrees
      mpu[b].dmpGetQuaternion(&q, fifoBuffer);
      mpu[b].dmpGetGravity(&gravity, &q);
      mpu[b].dmpGetYawPitchRoll(ypr, &q, &gravity);
      Serial.print("mpu#");
      Serial.print(b);
      Serial.print("\t");
      Serial.print(ypr[0] * 180 / M_PI);
      Serial.print("\t");
      Serial.print(ypr[1] * 180 / M_PI);
      Serial.print("\t");
      Serial.println(ypr[2] * 180 / M_PI);
#endif
    }
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}
