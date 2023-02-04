
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(7, 8);   // nRF24L01 (CE, CSN), can be any digital pin on the arduino

// called this way, it uses the default address 0x40
// pwm is adafruits library meant to mimic things like servo.writeMicroseconds in order for them to use it on the dedicated servo driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  600 // This is the rounded 'minimum' microsecond length, in this case the servo can have a minimum of 600 (600-2400 for 270 degrees rotation)
#define SERVOMAX  2400 // This is the rounded 'maximum' microsecond length, in this case the servo can have a maximum of 2400
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// our servo # counter
uint8_t servo[] = {0, 1, 2, 3, 4, 5};
float servoMap[] = {0, 0, 0, 0, 0, 0};
float servoPos[] = {127, 127, 127, 127, 127, 127};
float servoMapR = 0;

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
  // 127 because byte value is 0-255, so 127 is middle
  data.MPU0_y = 127;
  data.MPU0_p = 127;
  data.MPU0_r = 127;
  data.MPU1_y = 127;
  data.MPU1_p = 127;
  data.MPU1_r = 127;
  for (byte b = 0; b < 6; b++) {
    servoMapR = map(127, 255, 0, SERVOMIN, SERVOMAX);
    pwm.writeMicroseconds(servo[b], servoMapR);
    servoPos[b] = 127;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Beginning...");

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

int i = 127;

void loop() {

  // Check whether there is data to be received
  if (radio.available()) {
    radio.read(&data, sizeof(Data_Package)); // Read the whole data and store it into the 'data' structure
    lastReceiveTime = millis(); // At this moment we have received the data
    //    Serial.println("Available");
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
    Serial.print("; ServoPos[4]: ");
    Serial.print(servoPos[4]);
    Serial.println("");

    // Map byte value to servo range, for each servo. Servos are numbered from bottom to top.
    //    servoMap[0] = map(data.MPU0_y, 255, 0, SERVOMIN, SERVOMAX); // Base
    //    servoMap[1] = map(data.MPU0_p, 255, 0, SERVOMIN, SERVOMAX); // Shoulder
    //    servoMap[2] = map(data.MPU1_p, 255, 0, SERVOMIN, SERVOMAX); // Elbow
    //    servoMap[3] = map(data.MPU1_r, 255, 0, SERVOMIN, SERVOMAX); // Wrist
    //    servoMap[4] = map(data.MPU0_r, 255, 0, SERVOMIN, SERVOMAX); // Claw
    // Note: There is only 5 servos, so MPU1_y isn't in use.

    // Base
    if (data.MPU0_y > 132 and servoPos[0] < 205) {
      servoPos[0] = servoPos[0] + 1;
      servoMap[0] = map(servoPos[0], 255, 0, SERVOMIN, SERVOMAX);
      delay(15);
    }
    if (data.MPU0_y < 122 and servoPos[0] > 50) {
      servoPos[0] = servoPos[0] - 1;
      servoMap[0] = map(servoPos[0], 255, 0, SERVOMIN, SERVOMAX);
      delay(15);
    }

    // Shoulder
    if (data.MPU0_p > 132 and servoPos[1] < 205) {
      servoPos[1] = servoPos[1] + 1;
      servoMap[1] = map(servoPos[1], 255, 0, SERVOMIN, SERVOMAX);
      delay(15);
    }
    if (data.MPU0_p < 122 and servoPos[1] > 50) {
      servoPos[1] = servoPos[1] - 1;
      servoMap[1] = map(servoPos[1], 255, 0, SERVOMIN, SERVOMAX);
      delay(15);
    }

    // Elbow
    if (data.MPU1_p > 132 and servoPos[2] < 205) {
      servoPos[2] = servoPos[2] + 1;
      servoMap[2] = map(servoPos[2], 255, 0, SERVOMIN, SERVOMAX);
      delay(15);
    }
    if (data.MPU1_p < 122 and servoPos[2] > 50) {
      servoPos[2] = servoPos[2] - 1;
      servoMap[2] = map(servoPos[2], 255, 0, SERVOMIN, SERVOMAX);
      delay(15);
    }

    // Wrist
    if (data.MPU1_r > 132 and servoPos[3] < 255) {
      servoPos[3] = servoPos[3] + 1;
      servoMap[3] = map(servoPos[3], 255, 0, SERVOMIN, SERVOMAX);
      delay(15);
    }
    if (data.MPU1_r < 122 and servoPos[3] > 0) {
      servoPos[3] = servoPos[3] - 1;
      servoMap[3] = map(servoPos[3], 255, 0, SERVOMIN, SERVOMAX);
      delay(15);
    }

    // Claw
    if (data.MPU0_r > 132 and servoPos[4] < 171) {
      servoPos[4] = servoPos[4] + 1;
      servoMap[4] = map(servoPos[4], 255, 0, SERVOMIN, SERVOMAX);
      delay(15);
    }
    if (data.MPU0_r < 122 and servoPos[4] > 111) {
      servoPos[4] = servoPos[4] - 1;
      servoMap[4] = map(servoPos[4], 255, 0, SERVOMIN, SERVOMAX);
      delay(15);
    }

    for (byte b = 0; b < 6; b++) {
      pwm.writeMicroseconds(servo[b], servoMap[b]);
    }

    //    // Limit the movement each of the servos movement so they doesn't hit any other part
    //    if (data.MPU0_y > 50 and data.MPU0_y < 205) {
    //      pwm.writeMicroseconds(servo[0], servoMap[0]);
    //    }
    //    if (data.MPU0_p > 50 and data.MPU0_p < 205) {
    //      pwm.writeMicroseconds(servo[1], servoMap[1]);
    //    }
    //    if (data.MPU1_p > 50 and data.MPU1_p < 205) {
    //      pwm.writeMicroseconds(servo[2], servoMap[2]);
    //    }
    //    if (data.MPU1_r > 0 and data.MPU1_r < 255) {
    //      pwm.writeMicroseconds(servo[3], servoMap[3]);
    //    }
    //    if (data.MPU0_r > 111 and data.MPU0_r < 171) {
    //      pwm.writeMicroseconds(servo[4], servoMap[4]);
    //    }
  }

  // Check whether we keep receving data, or we have a connection between the two modules
  currentTime = millis();
  if ( currentTime - lastReceiveTime > 1000 ) { // If current time is more then 1 second since we have recived the last data, that means we have lost connection
    resetData(); // If connection is lost, reset the data. It prevents unwanted behavior, for example if a drone has a throttle up and we lose connection, it can keep flying unless we reset the values
  }
}
