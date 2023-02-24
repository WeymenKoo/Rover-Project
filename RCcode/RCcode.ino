/*
  Connections:
  BTS7960 -> Arduino Mega 2560 / Teensy 4.1 / Waifuduino
  MotorRight_R_EN - 22 - Teensy 4.1 pin 6 - 3
  MotorRight_L_EN - 23 - Teensy 4.1 pin 7 - 4
  MotorLeft_R_EN - 24 - Teensy 4.1 pin 8 - 12
  MotorLeft_L_EN - 25 - Teensy 4.1 pin 9 - 13
  Rpwm1 - 2 - Teensy 4.1 pin 2 - 5
  Lpwm1 - 3 - Teensy 4.1 pin 3 - 6
  Rpwm2 - 4 - Teensy 4.1 pin 4 - 10
  Lpwm2 - 5 - Teensy 4.1 pin 5 - 11

  ​FlySky FS 2.4GHz Receiver -> Arduino Mega 2560 / Teensy 4.1 / Waifuduino
  ch2 - 7 // Aileron  - Teensy 4.1 pin 36 - 8
  ch3 - 8 // Elevator  - Teensy 4.1 pin 37 - 9
*/

/*Defining the upper and lower threshold in order for the motor to start moving*/
#define LOWER_STOP_RANGE_MOVE -20
#define UPPER_STOP_RANGE_MOVE 20
#define LOWER_STOP_RANGE_TURN -20
#define UPPER_STOP_RANGE_TURN 20

#define HazardLight 11 // pin number where the neopixel strip is connected
#define HeadLight 12
#define NUMPIXELS 6
#define CALIBRATIONTIME 20000

#include <Adafruit_NeoPixel.h>
unsigned long pixelsInterval = 50; // the time we need to wait
unsigned long theaterChasePreviousMillis = 0;
unsigned long colorWipePreviousMillis = 0;
uint16_t currentPixel = 0;// what pixel are we operating on
int theaterChaseQ = 0;
int theaterChaseRainbowQ = 0;
// create a neopixel strip object
Adafruit_NeoPixel strip(NUMPIXELS, HazardLight, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip2(NUMPIXELS, HeadLight, NEO_GRB + NEO_KHZ800);

#include <Servo.h>
Servo servoMotor;

/*BTS7960 Motor Driver Carrier*/
const int MotorRight_R_EN = 22;
const int MotorRight_L_EN = 23;

const int MotorLeft_R_EN = 24;
const int MotorLeft_L_EN = 25;

const int Rpwm1 = 3;
const int Lpwm1 = 2;
const int Rpwm2 = 5;
const int Lpwm2 = 4;

long pwmLvalue = 255;
long pwmRvalue = 255;
byte pwmChannel;
int robotControlState;
int last_mspeed;
boolean stop_state = true;

// MODE2
const int Elevator = 6; // Elevator input pin on the Arduino
const int Aileron = 7; // Aileron input pin on the Arduino
const int Rudder = 8; // Rudder input pin on the Arduino

int moveValue;
int turnValue;

void setup() {
  Serial.begin(115200);
  currentPixel = 0;
  strip.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  strip2.begin();
  strip.show(); // Turn OFF all pixels ASAP
  strip2.show();
  strip.setBrightness(255); // Set BRIGHTNESS to about 1/5 (max = 255)
  strip2.setBrightness(255);

  //Setup Right Motors
  pinMode(MotorRight_R_EN, OUTPUT); //Initiates Motor Channel A1 pin
  pinMode(MotorRight_L_EN, OUTPUT); //Initiates Motor Channel A2 pin

  //Setup Left Motors
  pinMode(MotorLeft_R_EN, OUTPUT); //Initiates Motor Channel B1 pin
  pinMode(MotorLeft_L_EN, OUTPUT); //Initiates Motor Channel B2 pin

  //Setup PWM pins as Outputs
  pinMode(Rpwm1, OUTPUT);
  pinMode(Lpwm1, OUTPUT);
  pinMode(Rpwm2, OUTPUT);
  pinMode(Lpwm2, OUTPUT);

  servoMotor.attach(9); // attach servo to pin 9
  stop_Robot();
}

void loop() {
  // read the pulse width of the input signal
  int pulseWidthElv = pulseIn(Elevator, HIGH);
  int pulseWidthAil = pulseIn(Aileron, HIGH);
  int pulseWidthRud = pulseIn(Rudder, HIGH); // read PWM signal from pin 8 with a timeout of 25ms

  //Defining movement speed
  moveValue = map(pulseWidthElv, 1119, 1870, -255, 255); // Convert the raw signal range of the receiver into [-255,255]
  moveValue = constrain(moveValue, -255, 255); // Change the number from 20 to 255 to change speed
  Serial.print("Move Value: ");
  Serial.print(moveValue);

  //Defining turning speed
  turnValue = map(pulseWidthAil, 1119, 1870, -255, 255); // Convert the raw signal range of the receiver into [-255,255]
  turnValue = constrain(turnValue, -255, 255); // Change the number from 20 to 255 to change speed
  Serial.print("  Turn Value: ");
  Serial.println(turnValue);

  //Serial.println("moveValue: "+String(moveValue)+ ", turnValue: "+String(turnValue));
  if (moveValue > LOWER_STOP_RANGE_MOVE && moveValue < UPPER_STOP_RANGE_MOVE && turnValue > LOWER_STOP_RANGE_TURN && turnValue < UPPER_STOP_RANGE_TURN) {
    if (stop_state == false) {
      stop_Robot();
      stop_state = true;
      Serial.println("Stop Robot");
    }
  }

  //GO FORWARD & BACKWARD
  else if (turnValue > LOWER_STOP_RANGE_TURN && turnValue < UPPER_STOP_RANGE_TURN) {
    if (moveValue > UPPER_STOP_RANGE_MOVE) {
      go_Forward(moveValue);
      stop_state = false;
      Serial.println("Go Forward");
    }
    else if (moveValue < LOWER_STOP_RANGE_MOVE) {
      go_Backwad(abs(moveValue));
      stop_state = false;
      Serial.println("Go Backward");
    }
  }

  //TURN RIGHT & LEFT
  else if (moveValue > LOWER_STOP_RANGE_MOVE && moveValue < UPPER_STOP_RANGE_MOVE) {
    if (turnValue > UPPER_STOP_RANGE_TURN) {
      turn_Right(turnValue);
      stop_state = false;
      Serial.println("Turn Right");
    }
    else if (turnValue < LOWER_STOP_RANGE_TURN) {
      turn_Left(abs(turnValue));
      stop_state = false;
      Serial.println("Turn Left");
    }
  }

  ///////NEOPIXEL//////
  if ((unsigned long)(millis() - theaterChasePreviousMillis) >= pixelsInterval) {
    theaterChasePreviousMillis = millis();
    theaterChase(strip.Color(255, 60, 0));
  }
  if ((unsigned long)(millis() - colorWipePreviousMillis) >= pixelsInterval) {
    colorWipePreviousMillis = millis();
    colorWipe(strip2.Color(125, 125, 100));
  }
}



//Theatre-style crawling lights.
void theaterChase(uint32_t c) {
  for (int i = 0; i < strip.numPixels(); i = i + 3) {
    strip.setPixelColor(i + theaterChaseQ, c);  //turn every third pixel on
  }
  strip.show();
  for (int i = 0; i < strip.numPixels(); i = i + 3) {
    strip.setPixelColor(i + theaterChaseQ, 0);      //turn every third pixel off
  }
  theaterChaseQ++;
  if (theaterChaseQ >= 3) theaterChaseQ = 0;
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c) {
  strip2.setPixelColor(currentPixel, c);
  strip2.show();
  currentPixel++;
  if (currentPixel == NUMPIXELS) {
    currentPixel = 0;
  }
}

void stop_Robot() { // robotControlState = 0
  if (robotControlState != 0) {
    //SetMotors(2);
    analogWrite(Rpwm1, 0);
    analogWrite(Lpwm1, 0);
    analogWrite(Rpwm2, 0);
    analogWrite(Lpwm2, 0);
    robotControlState = 0;
  }
}// void stopRobot()

void turn_Right(int mspeed) { // robotControlState = 1
  if (robotControlState != 1 || last_mspeed != mspeed) {
    SetMotors(1);
    analogWrite(Rpwm1, 0);
    analogWrite(Lpwm1, mspeed);
    analogWrite(Rpwm2, mspeed);
    analogWrite(Lpwm2, 0);
    robotControlState = 1;
    last_mspeed = mspeed;
  }
}// void turn_Right(int mspeed)

void turn_Left(int mspeed) { // robotControlState = 2
  if (robotControlState != 2 || last_mspeed != mspeed) {
    SetMotors(1);
    analogWrite(Rpwm1, mspeed);
    analogWrite(Lpwm1, 0);
    analogWrite(Rpwm2, 0);
    analogWrite(Lpwm2, mspeed);
    robotControlState = 2;
    last_mspeed = mspeed;
  }
}// void turn_Left(int mspeed)

void go_Forward(int mspeed) { // robotControlState = 3
  if (robotControlState != 3 || last_mspeed != mspeed) {
    SetMotors(1);
    analogWrite(Rpwm1, mspeed);
    analogWrite(Lpwm1, 0);
    analogWrite(Rpwm2, mspeed);
    analogWrite(Lpwm2, 0);
    robotControlState = 3;
    last_mspeed = mspeed;
  }
}// void goForward(int mspeed)

void go_Backwad(int mspeed) { // robotControlState = 4
  if (robotControlState != 4 || last_mspeed != mspeed) {
    SetMotors(1);
    analogWrite(Rpwm1, 0);
    analogWrite(Lpwm1, mspeed);
    analogWrite(Rpwm2, 0);
    analogWrite(Lpwm2, mspeed);
    robotControlState = 4;
    last_mspeed = mspeed;
  }
}// void goBackwad(int mspeed)

/*
void move_RightForward(int mspeed) { // robotControlState = 5
  if (robotControlState != 5 || last_mspeed != mspeed) {
    SetMotors(1);
    analogWrite(Rpwm1, mspeed * 0.4);
    analogWrite(Lpwm1, 0);
    analogWrite(Rpwm2, mspeed);
    analogWrite(Lpwm2, 0);
    robotControlState = 5;
    last_mspeed = mspeed;
  }
}// void move_RightForward(int mspeed)

void move_LeftForward(int mspeed) { // robotControlState = 6
  if (robotControlState != 6 || last_mspeed != mspeed) {
    SetMotors(1);
    analogWrite(Rpwm1, mspeed);
    analogWrite(Lpwm1, 0);
    analogWrite(Rpwm2, mspeed * 0.4);
    analogWrite(Lpwm2, 0);
    robotControlState = 6;
    last_mspeed = mspeed;
  }
}// move_LeftForward(int mspeed)

void move_RightBackward(int mspeed) { // robotControlState = 7
  if (robotControlState != 7 || last_mspeed != mspeed) {
    SetMotors(1);
    analogWrite(Rpwm1, 0);
    analogWrite(Lpwm1, mspeed * 0.4);
    analogWrite(Rpwm2, 0);
    analogWrite(Lpwm2, mspeed);
    robotControlState = 7;
    last_mspeed = mspeed;
  }
}// void move_RightBackward(int mspeed)

void move_LeftBackward(int mspeed) { // robotControlState = 8
  if (robotControlState != 8 || last_mspeed != mspeed) {
    SetMotors(1);
    analogWrite(Rpwm1, 0);
    analogWrite(Lpwm1, mspeed);
    analogWrite(Rpwm2, 0);
    analogWrite(Lpwm2, mspeed * 0.4);
    robotControlState = 8;
    last_mspeed = mspeed;
  }
}// void move_LeftBackward(int mspeed)*/

void stopRobot(int delay_ms) {
  SetMotors(2);
  analogWrite(Rpwm1, 0);
  analogWrite(Lpwm1, 0);
  analogWrite(Rpwm2, 0);
  analogWrite(Lpwm2, 0);
  delay(delay_ms);
}// void stopRobot(int delay_ms)

void SetPWM(const long pwm_num, byte pwm_channel) {
  if (pwm_channel == 1) { // DRIVE MOTOR
    analogWrite(Rpwm1, 0);
    analogWrite(Rpwm2, 0);
    analogWrite(Lpwm1, pwm_num);
    analogWrite(Lpwm2, pwm_num);
    pwmRvalue = pwm_num;
  }
  else if (pwm_channel == 2) { // STEERING MOTOR
    analogWrite(Lpwm1, 0);
    analogWrite(Lpwm2, 0);
    analogWrite(Rpwm1, pwm_num);
    analogWrite(Rpwm2, pwm_num);
    pwmLvalue = pwm_num;
  }
}// void SetPWM (const long pwm_num, byte pwm_channel)

void SetMotors(int controlCase) {
  switch (controlCase) {
    case 1:
      digitalWrite(MotorRight_R_EN, HIGH);
      digitalWrite(MotorRight_L_EN, HIGH);
      digitalWrite(MotorLeft_R_EN, HIGH);
      digitalWrite(MotorLeft_L_EN, HIGH);
      break;
    case 2:
      digitalWrite(MotorRight_R_EN, LOW);
      digitalWrite(MotorRight_L_EN, LOW);
      digitalWrite(MotorLeft_R_EN, LOW);
      digitalWrite(MotorLeft_L_EN, LOW);
      break;
  }
}// void SetMotors(int controlCase)
