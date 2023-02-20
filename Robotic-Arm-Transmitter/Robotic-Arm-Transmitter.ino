// By Ashish Agrahari: https://github.com/AshishA26

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>

MPU6050 mpu[] = {0x68, 0x69};// AD0 low = 0x68, AD0 high = 0x69

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
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

float MPU_y[] = {0, 0};
float MPU_p[] = {0, 0};
float MPU_r[] = {0, 0};

RF24 radio(7, 8);   // nRF24L01 (CE, CSN)
const byte address[6] = "00001"; // Address

// Max size of this struct is 32 bytes - NRF24L01 buffer limit
struct Data_Package {
  byte MPU0_y;
  byte MPU0_p;
  byte MPU0_r;
  byte MPU1_y;
  byte MPU1_p;
  byte MPU1_r;
};
Data_Package data; //Create a variable with the above structure

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  Wire.setWireTimeout(3000, true); // VERY IMPORTANT!!! THIS FIXES THE I2C BUS FROM HANGING
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200);
  while (!Serial);

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

  // Define the radio communication
  radio.begin();
  radio.openWritingPipe(address);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MIN); // It was originally LOW

  data.MPU0_y = 0;
  data.MPU0_p = 0;
  data.MPU0_r = 0;
  data.MPU1_y = 0;
  data.MPU1_p = 0;
  data.MPU1_r = 0;
}

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  for (byte b = 0; b < 2; b++) {
    // read a packet from FIFO
    if (mpu[b].dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
      // display Euler angles in degrees
      mpu[b].dmpGetQuaternion(&q, fifoBuffer);
      mpu[b].dmpGetGravity(&gravity, &q);
      mpu[b].dmpGetYawPitchRoll(ypr, &q, &gravity);
      //      Serial.print("mpu#");
      //      Serial.print(b);
      //      Serial.print("\t");
      //      Serial.print(ypr[0] * 180 / M_PI);
      //      Serial.print("\t");
      //      Serial.print(ypr[1] * 180 / M_PI);
      //      Serial.print("\t");
      //      Serial.println(ypr[2] * 180 / M_PI);

      MPU_y[b] = ypr[0] * 180 / M_PI;
      MPU_p[b] = ypr[1] * 180 / M_PI;
      MPU_r[b] = ypr[2] * 180 / M_PI;
    }
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }

  //Map the data from MPU to 0-255 range and then round it to be able to send in bytes, and set it to the data variables being sent
  data.MPU0_y = round(map(MPU_y[0], -180, 180, 0, 255));
  data.MPU0_p = round(map(MPU_p[0], -180, 180, 0, 255));
  data.MPU0_r = round(map(MPU_r[0], -180, 180, 0, 255));
  data.MPU1_y = round(map(MPU_y[1], -180, 180, 0, 255));
  data.MPU1_p = round(map(MPU_p[1], -180, 180, 0, 255));
  data.MPU1_r = round(map(MPU_r[1], -180, 180, 0, 255));

  Serial.print("MPU0_y: ");
  Serial.print(data.MPU0_y);
  Serial.print("; MPU0_p: ");
  Serial.print(data.MPU0_p);
  Serial.print("; MPU0_r: ");
  Serial.print(data.MPU0_r);
  Serial.print("; MPU1_y: ");
  Serial.print(data.MPU1_y);
  Serial.print("; MPU1_p: ");
  Serial.print(data.MPU1_p);
  Serial.print("; MPU1_r: ");
  Serial.print(data.MPU1_r);
  Serial.println("");

  // Send the whole data from the structure to the receiver
  radio.write(&data, sizeof(Data_Package));
}
