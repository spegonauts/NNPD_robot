#include <LiquidCrystal.h>
#include <Servo.h>


// pin variable assignment
const byte STR_PIN = 3;
const byte THR_PIN = 2;

const byte LEFT_1 = 4;
const byte LEFT_2 = 5;
const byte LEFT_3 = 6;
const byte LEFT_4 = 7;

const byte RIGHT_1 = 8;
const byte RIGHT_2 = 9;
const byte RIGHT_3 = 10;
const byte RIGHT_4 = 11;

const long interval = 500;

unsigned long preMilSTR, preMilTHR;
unsigned long startH, startL, pulseH, pulseL;

// used to record times
volatile unsigned long timerStartSTR, timerStartTHR;
volatile int lastInterruptSTR, lastInterruptTHR; 
volatile int pulseTimeSTR, pulseTimeTHR;

//STR and THR variable initialization
int dispSTR = 1500;
int recentSTR = 0;
int firstSTR = 0;
int secondSTR = 0;
int thirdSTR = 0;
int fourthSTR = 0;
int fifthSTR = 0;

int dispTHR = 1500;
int recentTHR = 0;
int firstTHR = 0;
int secondTHR = 0;
int thirdTHR = 0;
int fourthTHR = 0;
int fifthTHR = 0;

// Variables used to recieve the current value of the STR and THR
int pwmSTR;
int pwmTHR;

// Declare Servos; the motors use the arduino servo library
Servo left1;
Servo left2;
Servo left3;
Servo left4;

Servo right1;
Servo right2;
Servo right3;
Servo right4;

// Initializes the lcd screen, values are for pins
LiquidCrystal lcd(22, 23, 24, 25, 26, 27);

//ISR function for steering
void calcSignalSTR() {

  lastInterruptSTR = micros();

  if (digitalRead(STR_PIN) == HIGH) {
    timerStartSTR = micros();
  } else {

    if (timerStartSTR != 0) {
      pulseTimeSTR = ((volatile int)micros() - timerStartSTR);
      timerStartSTR = 0;
    }
  }
}

//ISR function for throttle
void calcSignalTHR() {
  lastInterruptTHR = micros();

  if (digitalRead(THR_PIN) == HIGH) {
    timerStartTHR = micros();
  } else {

    if (timerStartTHR != 0) {
      pulseTimeTHR = ((volatile int)micros() - timerStartTHR);
      timerStartTHR = 0;
    }
  }
}

void setup() {
//set pins to input
  pinMode(STR_PIN, INPUT);
  pinMode(THR_PIN, INPUT);

//ISR for steering and throttle 
// Interrupts stop other processing tasks in order to recieve high priority information, in this case the PWM values of the controller
  attachInterrupt(digitalPinToInterrupt(3), calcSignalSTR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(2), calcSignalTHR, CHANGE);

//Setup for lcd and Serial monitor
  lcd.begin(16, 2);

  lcd.print("LEFT:");

  lcd.setCursor(0, 1);

  lcd.print("Right:");
  
  Serial.begin(9600);

//preMil variable set to 0
  preMilSTR = 0;
  preMilTHR = 0;  
  
//victor setup
// Victors are common FRC motor controllers, used to recieve and distribute PWM or CAN signals
// This code uses PWM for simplicity, CAN is far more complicated 

// The pins are first 'attached' to the motor controllers (tells the servo to look at a specific pin for info)
// The pin is then set to output mode (the pin will only ever be able to send outputs, never recieve information)
  left1.attach(LEFT_1);
  pinMode(LEFT_1, OUTPUT);
  
  left2.attach(LEFT_2);
  pinMode(LEFT_2, OUTPUT);
  
  left3.attach(LEFT_3);
  pinMode(LEFT_3, OUTPUT);
  
  left4.attach(LEFT_4);
  pinMode(LEFT_4, OUTPUT);
  

  right1.attach(RIGHT_1);
  pinMode(RIGHT_1, OUTPUT);
  
  right2.attach(RIGHT_2);
  pinMode(RIGHT_2, OUTPUT);
  
  right3.attach(RIGHT_3);
  pinMode(RIGHT_3, OUTPUT);
  
  right4.attach(RIGHT_4);
  pinMode(RIGHT_4, OUTPUT);
}


// Because both sets of wheels (2 left, 2 right) will always move at the same speed, functions make writiing the same value quicker
void writeLeft(int val) {
  left1.writeMicroseconds(val);
  left2.writeMicroseconds(val);
  left3.writeMicroseconds(val);
  left4.writeMicroseconds(val);
}

void writeRight(int val) {
  right1.writeMicroseconds(val);
  right2.writeMicroseconds(val);
  right3.writeMicroseconds(val);
  right4.writeMicroseconds(val);
}

void loop() {

  unsigned long currentMilSTR, currentMilTHR;

  currentMilSTR = millis();
  currentMilTHR = millis();

  //pwmSTR = pulseIn(STR_PIN, HIGH);
  //pwmTHR = pulseIn(THR_PIN, HIGH);

  lcd.setCursor(0, 1);

//update STR
  if (currentMilSTR - preMilSTR >= interval) {
    preMilSTR = currentMilSTR;

    dispSTR = pulseTimeSTR;

    recentSTR = dispSTR;

  }

//update THR
  if (currentMilTHR - preMilTHR >= interval) {
    preMilTHR = currentMilTHR;

    dispTHR = pulseTimeTHR;

    recentTHR = dispTHR;
    
  }

 //rolling average calculation
  dispSTR = ((recentSTR + firstSTR + secondSTR + thirdSTR + fourthSTR + fifthSTR) / 6);

  dispTHR = ((recentTHR + firstTHR + secondTHR + thirdTHR + fourthTHR + fifthTHR) / 6);
  
//  
//map values
// Values used were found based off of experimentation, may not be %100 accurate
  dispSTR = map(dispSTR, 1080, 1876, 1000, 2000);
  dispTHR = map(dispTHR, 1100, 1888, 1000, 2000);

  dispSTR = constrain(dispSTR, 1000, 2000);
  dispTHR = constrain(dispTHR, 1000, 2000);
  
//print values from rc
  lcd.setCursor(7, 0);
  
// These no longer display accurate information becuase these were being used to test the state of the robot as a whole
if (dispSTR  >= 1440 && dispSTR <= 1560) {
  lcd.print(dispSTR);
  lcd.setCursor(15, 0);
  lcd.print("1");
} else if ((dispSTR <= 1440 || dispSTR >=1560) && (dispTHR <= 1440 || dispTHR >= 1560)) {
  lcd.print(dispSTR);
  lcd.setCursor(15, 0);
  lcd.print("2");
} else if ((dispTHR >= 1440 || dispTHR <= 1560) && (dispSTR <= 1440 || dispSTR >= 1560)) {
  lcd.print(dispSTR);
  lcd.setCursor(15, 0);
  lcd.print("3");
}

  lcd.setCursor(7, 1);
  
// Comment from line 217 applies here too
if (dispSTR  >= 1440 && dispSTR <= 1560) {
  // Just use throttle
  lcd.print(dispSTR);
  lcd.setCursor(15, 1);
  lcd.print("1");
} else if ((dispSTR <= 1440 || dispSTR >=1560) && (dispTHR <= 1440 || dispTHR >= 1560)) {
  // Steer and throttle
  lcd.print(dispSTR);
  lcd.setCursor(15, 1);
  lcd.print("2");
} else if ((dispTHR >= 1440 || dispTHR <= 1560) && (dispSTR <= 1440 || dispSTR >= 1560)) {
  // No throttle, Steer in place
  lcd.print(dispSTR);
  lcd.setCursor(15, 1);
  lcd.print("3");
}

//drive
if ((dispSTR >= 1440 && dispSTR <= 1560) && (dispTHR >=1440 && dispTHR <= 1440)) {
  writeLeft(1500);
  writeRight(1500);
} else {
  if (dispSTR  >= 1440 && dispSTR <= 1560) {
    if (dispTHR >= 1560) {
      writeLeft(dispTHR);
      writeRight((2000 - dispTHR) + 1000);
   } else if (dispTHR <= 1440) {
     writeLeft(dispTHR);
     writeRight(2000 - (dispTHR - 1000));
   }
  } else if ((dispSTR <= 1440 || dispSTR >=1560) && (dispTHR <= 1440 || dispTHR >= 1560)) {
   writeLeft(constrain((dispTHR + (dispSTR - 1500)), 1000, 2000));
   writeRight(constrain((dispTHR + -(dispSTR - 1500)), 1000, 2000));
  } else if ((dispTHR >= 1440 || dispTHR <= 1560) && (dispSTR <= 1440 || dispSTR >= 1560)) {
   writeLeft(dispSTR);
   writeRight(dispSTR);
  }
}

  

//rolling average variable assignment
  if (recentSTR != 0) {
    fifthSTR = recentSTR;
  }
  if (fifthSTR != 0) {
    fourthSTR = fifthSTR;
  }
  if (fourthSTR != 0) {
    thirdSTR = fourthSTR;
  }
  if (thirdSTR != 0) {
    secondSTR = thirdSTR;
  }
  if (secondSTR != 0) {
    firstSTR = secondSTR;
  }


  if (recentTHR != 0) {
    fifthTHR = recentTHR;
  }
  if (fifthTHR != 0) {
    fourthTHR = fifthTHR;
  }
  if (fourthTHR != 0) {
    thirdTHR = fourthTHR;
  }
  if (thirdTHR != 0) {
    secondTHR = thirdTHR;
  }
  if (secondTHR != 0) {
    firstTHR = secondTHR;
  }
}
