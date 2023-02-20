// By Ashish Agrahari: https://github.com/AshishA26

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

#define MPUBeginHigh  137
#define MPUBeginLow  117
#define MPUCheckHigh  140
#define MPUCheckLow  114
#define delayTime  20

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

/*
 * Function to check if other axis' are centered (or close to their center positions) as 2 axis' should not be moving at the same time. 
 * 2 axis motion at the same time has been stopped because, for example, if the MPU had a yaw of 0 and a roll of 0, and then the yaw 
 * was moved to say 65, then the roll would for no reason change to something like 180. This is most likely due to the programmed 
 * mathematics in the MPU itself. So this is just to prevent any weird motion.
*/
int checkOtherAxis(int axisNumber) {
  if (axisNumber == 0) {
    if (data.MPU0_p > MPUCheckHigh or data.MPU0_p < MPUCheckLow or data.MPU0_r > MPUCheckHigh or data.MPU0_r < MPUCheckLow) {
      return 0;
    }
    else {
      return 1;
    }
  }
  else if (axisNumber == 4) {
    if (data.MPU0_y > MPUCheckHigh or data.MPU0_y < MPUCheckLow or data.MPU0_r > MPUCheckHigh or data.MPU0_r < MPUCheckLow) {
      return 0;
    }
    else {
      return 1;
    }
  }
  else if (axisNumber == 3) {
    if (data.MPU1_r > MPUCheckHigh or data.MPU1_r < MPUCheckLow or data.MPU1_y > MPUCheckHigh or data.MPU1_y < MPUCheckLow) {
      return 0;
    }
    else {
      return 1;
    }
  }
  else if (axisNumber == 2) {
    if (data.MPU1_p > MPUCheckHigh or data.MPU1_p < MPUCheckLow or data.MPU1_y > MPUCheckHigh or data.MPU1_y < MPUCheckLow) {
      return 0;
    }
    else {
      return 1;
    }
  }
  else if (axisNumber == 1) {
    if (data.MPU0_y > MPUCheckHigh or data.MPU0_y < MPUCheckLow or data.MPU0_p > MPUCheckHigh or data.MPU0_p < MPUCheckLow) {
      return 0;
    }
    else {
      return 1;
    }
  }
}

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
    Serial.println("");

    // Note: Servos are numbered from bottom to top. Also, there is only 5 servos, so I have decided to not use the axis MPU1_y.
    // Below if statements are to limit servo motion to prevent parts from hitting each other. Also checks whether MPU has been tilted above or below a certain number, and checks if other axises are (almost) centered.
    
    // Base
    if (data.MPU0_y > MPUBeginHigh and servoPos[0] < 255 and checkOtherAxis(0) == 1) {
      servoPos[0] = servoPos[0] + 1;
      servoMap[0] = map(servoPos[0], 255, 0, SERVOMIN, SERVOMAX);
      delay(delayTime);
    }
    else if (data.MPU0_y < MPUBeginLow and servoPos[0] > 0 and checkOtherAxis(0) == 1) {
      servoPos[0] = servoPos[0] - 1;
      servoMap[0] = map(servoPos[0], 255, 0, SERVOMIN, SERVOMAX);
      delay(delayTime);
    }

    // Shoulder
    else if (data.MPU0_r > MPUBeginHigh and servoPos[1] < 205 and checkOtherAxis(1) == 1) {
      servoPos[1] = servoPos[1] + 1;
      servoMap[1] = map(servoPos[1], 255, 0, SERVOMIN, SERVOMAX);
      delay(delayTime);
    }
    else if (data.MPU0_r < MPUBeginLow and servoPos[1] > 50 and checkOtherAxis(1) == 1) {
      servoPos[1] = servoPos[1] - 1;
      servoMap[1] = map(servoPos[1], 255, 0, SERVOMIN, SERVOMAX);
      delay(delayTime);
    }

    // Elbow
    else if (data.MPU1_r > MPUBeginHigh and servoPos[2] < 205 and checkOtherAxis(2) == 1) {
      servoPos[2] = servoPos[2] + 1;
      servoMap[2] = map(servoPos[2], 255, 0, SERVOMIN, SERVOMAX);
      delay(delayTime);
    }
    else if (data.MPU1_r < MPUBeginLow and servoPos[2] > 50 and checkOtherAxis(2) == 1) {
      servoPos[2] = servoPos[2] - 1;
      servoMap[2] = map(servoPos[2], 255, 0, SERVOMIN, SERVOMAX);
      delay(delayTime);
    }

    // Wrist
    else if (data.MPU1_p > MPUBeginHigh and servoPos[3] < 255 and checkOtherAxis(3) == 1) {
      servoPos[3] = servoPos[3] + 1;
      servoMap[3] = map(servoPos[3], 255, 0, SERVOMIN, SERVOMAX);
      delay(delayTime);
    }
    else if (data.MPU1_p < MPUBeginLow and servoPos[3] > 0 and checkOtherAxis(3) == 1) {
      servoPos[3] = servoPos[3] - 1;
      servoMap[3] = map(servoPos[3], 255, 0, SERVOMIN, SERVOMAX);
      delay(delayTime);
    }

    // Claw
    else if (data.MPU0_p > MPUBeginHigh and servoPos[4] < 171 and checkOtherAxis(4) == 1) {
      servoPos[4] = servoPos[4] + 1;
      servoMap[4] = map(servoPos[4], 255, 0, SERVOMIN, SERVOMAX);
      delay(delayTime);
    }
    else if (data.MPU0_p < MPUBeginLow and servoPos[4] > 111 and checkOtherAxis(4) == 1) {
      servoPos[4] = servoPos[4] - 1;
      servoMap[4] = map(servoPos[4], 255, 0, SERVOMIN, SERVOMAX);
      delay(delayTime);
    }

    // Writes values to each servo
    for (byte b = 0; b < 6; b++) {
      pwm.writeMicroseconds(servo[b], servoMap[b]);
    }
  }

  // Check whether we keep receving data, or we have a connection between the two modules
  currentTime = millis();
  if ( currentTime - lastReceiveTime > 1000 ) { // If current time is more then 1 second since we have recived the last data, that means we have lost connection
    //resetData(); // This is commented out because if the nRF does disconnect, we don't want the arm to fling into its default position, we want to to stay in its previous position.
    Serial.println("nRF DISCONNECTED");
  }
}
