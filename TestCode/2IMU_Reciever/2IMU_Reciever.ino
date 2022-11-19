#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(32, 33);   // nRF24L01 (CE, CSN), can be any digital pin on the arduino

// called this way, it uses the default address 0x40
// pwm is adafruits library meant to mimic things like servo.writeMicroseconds in order for them to use it on the dedicated servo driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  600 // This is the rounded 'minimum' microsecond length, in this case the servo can have a minimum of 600 (600-2400 for 270 degrees rotation)
#define SERVOMAX  2400 // This is the rounded 'maximum' microsecond length, in this case the servo can have a maximum of 2400
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// our servo # counter
uint8_t servo[] = {0, 1, 2, 3, 4, 5};
float servoMap[] = {0, 0, 0, 0, 0, 0};

const byte address[6] = "00001";
unsigned long lastReceiveTime = 0;
unsigned long currentTime = 0;
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

void resetData() {
  // Reset the values when there is no radio connection - Set initial default values
  data.MPU0_y = 0;
  data.MPU0_p = 0;
  data.MPU0_r = 0;
  data.MPU1_y = 0;
  data.MPU1_p = 0;
  data.MPU1_r = 0;
  for (byte b = 0; b < 5; b++) {
    servoMap[b] = 0;
  }
}

void setup() {
  Serial.begin(9600);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MIN); // It was originally LOW
  radio.startListening(); //  Set the module as receiver
  resetData();
}

void loop() {

  // Check whether there is data to be received
  if (radio.available()) {
    radio.read(&data, sizeof(Data_Package)); // Read the whole data and store it into the 'data' structure
    lastReceiveTime = millis(); // At this moment we have received the data

    servoMap[0] = map(data.MPU0_y, 180, 0, SERVOMIN, SERVOMAX);
    pwm.writeMicroseconds(servo[0], servoMap[0]);
    servoMap[1] = map(data.MPU0_p, 180, 0, SERVOMIN, SERVOMAX);
    pwm.writeMicroseconds(servo[1], servoMap[1]);
    servoMap[2] = map(data.MPU0_r, 180, 0, SERVOMIN, SERVOMAX);
    pwm.writeMicroseconds(servo[2], servoMap[2]);

    servoMap[3] = map(data.MPU1_y, 180, 0, SERVOMIN, SERVOMAX);
    pwm.writeMicroseconds(servo[3], servoMap[3]);
    servoMap[4] = map(data.MPU1_p, 180, 0, SERVOMIN, SERVOMAX);
    pwm.writeMicroseconds(servo[4], servoMap[4]);
    servoMap[5] = map(data.MPU1_r, 180, 0, SERVOMIN, SERVOMAX);
    pwm.writeMicroseconds(servo[5], servoMap[5]);
  }
  // Check whether we keep receving data, or we have a connection between the two modules
  currentTime = millis();
  if ( currentTime - lastReceiveTime > 1000 ) { // If current time is more then 1 second since we have recived the last data, that means we have lost connection
    resetData(); // If connection is lost, reset the data. It prevents unwanted behavior, for example if a drone has a throttle up and we lose connection, it can keep flying unless we reset the values
  }
}
