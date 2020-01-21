//----------------------------------------------------------------------------
// NNPD Robot Project
//
// 2020-01-16  Jacob King      - Original version
// 2020-01-17  David Lawrence  - Added comments. Fix some indentation.
// 2020-01-21  David Lawrence  - Added code for RESET button for target
//
// Requires: 
//    PinChangeInterrupt
//
// This program implements the drive control for the robot via
// an Arduino board. The basic function is to read the PWN signals
// from the reciever representing the steering and throttle positions
// of the controller and then set the drive motors accordingly.
//
//    STR = steering
//    THR = throttle
//
// Details:
// The arduino reads the PWM signal widths by setting up interrupts
// on the pins they are connected to. The interrupt service routines
// (ISR) are called whenever the PWM signal transitions. On a transition
// to a high state, the global ardunio system clock is recorded. On
// a transition to a low state, the time passed is recorded as the pulse
// width to a global variable.
//
// The measured pulse width values are mapped to different PWM settings
// to send to the motor controllers. The motors themselves are DC motors,
// but are driven by use Victor controllers which use a PWM signal to
// determine voltage and polarity. Thus, arduino Servo objects are used
// to represent the DC motors.
// 
// The reset button for the target is implemented in a similar fashion
// using the AUX1 output of the receiver. Here though, the state is
// either on or off and the corresponding PWM from AUX1 is either
// ~1860 (not pressed) or ~1060(pressed). The output pin on the arduino
// used for the reset is set high when the button is activate and low
// when it is not.

#include <LiquidCrystal.h>
#include <Servo.h>
#include <PinChangeInterrupt.h>



// pin variable assignment
const byte STR_PIN = 3;
const byte THR_PIN = 2;
const byte AUX_PIN = 12;
const byte RESET_PIN = 13;

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

volatile unsigned long timerAUX1Start=0;
volatile unsigned long Ncalls_ISR_AUX1=0; // for debugging


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
unsigned long int pwmAUX1 = 0;

// Variables used to remember the last setting of the left and right PWM
int pwmLEFT;
int pwmRIGHT;

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

//----------------------------------------
// calcSignalSTR
//
// ISR function for steering
//----------------------------------------
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

//----------------------------------------
// calcSignalTHR
//
// ISR function for throttle
//----------------------------------------
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

//----------------------------------------
// ISR_AUX1
//
// ISR function for reset
//----------------------------------------
void ISR_AUX1(void) {

  volatile unsigned long now = micros();
 
  if (digitalRead(AUX_PIN) == HIGH) {
    timerAUX1Start = now;
  } else {
    if( now > timerAUX1Start ) {
      pwmAUX1 = now - timerAUX1Start;
    }
    timerAUX1Start = 0;
  }

  Ncalls_ISR_AUX1++;
}

//----------------------------------------
// setup
//----------------------------------------
void setup() {

  //set pins to input
  pinMode(STR_PIN, INPUT);
  pinMode(THR_PIN, INPUT);
  pinMode(AUX_PIN, INPUT_PULLUP);
  pinMode(RESET_PIN, OUTPUT);

  //ISR for steering and throttle 
  // Interrupts stop other processing tasks in order to recieve high priority information, in this case the PWM values of the controller
  attachInterrupt(digitalPinToInterrupt(3), calcSignalSTR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(2), calcSignalTHR, CHANGE);
  attachPCINT(digitalPinToPCINT(AUX_PIN), ISR_AUX1, CHANGE);

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

} // end setup()


//----------------------------------------
// writeLeft
//
// Because both sets of wheels (2 left, 2 right) will always move at the same speed, functions make writiing the same value quicker
//----------------------------------------
void writeLeft(int val) {
  left1.writeMicroseconds(val);
  left2.writeMicroseconds(val);
  left3.writeMicroseconds(val);
  left4.writeMicroseconds(val);
  pwmLEFT = val; // remember for display later
}

//----------------------------------------
// writeRight
//----------------------------------------
void writeRight(int val) {
  right1.writeMicroseconds(val);
  right2.writeMicroseconds(val);
  right3.writeMicroseconds(val);
  right4.writeMicroseconds(val);
  pwmRIGHT = val; // remember for display later
}

//----------------------------------------
// loop
//----------------------------------------
void loop() {

  unsigned long currentMilSTR, currentMilTHR;

  currentMilSTR = millis();
  currentMilTHR = millis();

  //pwmSTR = pulseIn(STR_PIN, HIGH);
  //pwmTHR = pulseIn(THR_PIN, HIGH);

//  lcd.setCursor(0, 1);

  // *** It is not clear what the purpose of the following 2  ***
  // *** if statements is. They appear to limit updating of   ***
  // *** the "recentSTR" and "recentTHR" variables to 500     ***
  // *** millisecond boundaries.  They also update the        ***
  // *** "dispSTR" and "dispTHR" variables, but these are     ***
  // *** immediately overwritten right after in the rolling   ***
  // *** average calculation. Perhaps this is a relic fromw   ***
  // *** an earlier attempt to smooth the controller values   ***
  // *** over time that was eventually replaced by the        ***
  // *** rolling average (??)                                 ***
  // *** NOTE: On further thinking, this seems to defeat the  ***
  // *** rolling average if loop() is called at least 6 times ***
  // *** in 500 milliseconds. Only at the boundaries will the ***
  // *** averaging have any (albeit brief) effect.            ***
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
  //map values  -  convert from receiver PWM to motor PWM
  // Values used were found based off of experimentation, may not be %100 accurate
  dispSTR = map(dispSTR, 1080, 1876, 1000, 2000);
  dispTHR = map(dispTHR, 1100, 1888, 1000, 2000);

  dispSTR = constrain(dispSTR, 1000, 2000);
  dispTHR = constrain(dispTHR, 1000, 2000);

  // Print last PWM values written to drive motors to lcd screen
  lcd.setCursor(7, 0);
  lcd.print(pwmLEFT);
  lcd.setCursor(7, 1);
  lcd.print(pwmRIGHT);

//*** The code below was in the original version and wrote the measured ***
//*** PWM values to the screen. It also included information about each ***
//*** of the STR and THR values being in the deadband in the form of a  ***
//*** 1,2,or 3. I'm not sure that was still working though since I did  ***
//*** not see them change from "1". Regardless, This has been commented ***
//*** out in preference for the above which displays the PWM motor      ***
//*** settings and the code at the bottom which displays if the reset   ***
//*** button is being pressed.                                          ***
//  lcd.setCursor(0, 0);
//  lcd.print("AUX 1: ");
//  lcd.print(pwmAUX1);
//
//  lcd.setCursor(0, 1);
//  lcd.print("Ncalls: ");
//  lcd.print(Ncalls_ISR_AUX1);

//  //print values from rc
//  lcd.setCursor(7, 0);
//  
//  // These no longer display accurate information becuase these were being used to test the state of the robot as a whole
//  if (dispSTR  >= 1440 && dispSTR <= 1560) {
//    lcd.print(dispSTR);
//    lcd.setCursor(15, 0);
//    lcd.print("1");
//  } else if ((dispSTR <= 1440 || dispSTR >=1560) && (dispTHR <= 1440 || dispTHR >= 1560)) {
//    lcd.print(dispSTR);
//    lcd.setCursor(15, 0);
//    lcd.print("2");
//  } else if ((dispTHR >= 1440 || dispTHR <= 1560) && (dispSTR <= 1440 || dispSTR >= 1560)) {
//    lcd.print(dispSTR);
//    lcd.setCursor(15, 0);
//    lcd.print("3");
//  }
//
//  lcd.setCursor(7, 1);
//  
//  // Comment from above applies here too
//  if (dispSTR  >= 1440 && dispSTR <= 1560) {
//    // Just use throttle
//    lcd.print(dispSTR);
//    lcd.setCursor(15, 1);
//    lcd.print("1");
//  } else if ((dispSTR <= 1440 || dispSTR >=1560) && (dispTHR <= 1440 || dispTHR >= 1560)) {
//    // Steer and throttle
//    lcd.print(dispSTR);
//    lcd.setCursor(15, 1);
//    lcd.print("2");
//  } else if ((dispTHR >= 1440 || dispTHR <= 1560) && (dispSTR <= 1440 || dispSTR >= 1560)) {
//    // No throttle, Steer in place
//    lcd.print(dispSTR);
//    lcd.setCursor(15, 1);
//    lcd.print("3");
//  }

  //drive
  //
  // Here values are written to the drive motors based on the rolling averages.
  // - A deadband is implmented for both the STR and THR.
  // - Values written are in the standard servo range of 1000-2000 with 1500 representing no motion.
  // - The right side motors are reversed relative to the left (i.e. we must write 1-x instead of x).
  // - The motors are set using a linear sum of the two controls (when both active).
  // - If only the STR is active, then the robot will spin at a rate proportional to the STR.
  
  if ((dispSTR >= 1440 && dispSTR <= 1560) && (dispTHR >=1440 && dispTHR <= 1560)) {
    // Both STR and THR in deadband. Stop all motors.
    writeLeft(1500);
    writeRight(1500);
  } else {
    // At least one of STR or THR are outside the deadband.
    if (dispSTR  >= 1440 && dispSTR <= 1560) {
      // STR is in deadband (THR is not)
      // *** The if statement below is not needed since the writeLeft ***
      // *** and writeRight calls are the same.                       ***
      if (dispTHR >= 1560) {
        writeLeft(dispTHR);
        writeRight((2000 - dispTHR) + 1000);
      } else if (dispTHR <= 1440) {
        writeLeft(dispTHR);
        writeRight(2000 - (dispTHR - 1000));
      }
    } else if ((dispSTR <= 1440 || dispSTR >=1560) && (dispTHR <= 1440 || dispTHR >= 1560)) {
      // Both STR and THR are out of deadband
      writeLeft(constrain((dispTHR + (dispSTR - 1500)), 1000, 2000));
      writeRight(constrain((dispTHR + -(dispSTR - 1500)), 1000, 2000));
    } else if ((dispTHR >= 1440 || dispTHR <= 1560) && (dispSTR <= 1440 || dispSTR >= 1560)) {
      // THR in deadband?? (STR is not)
      // *** The above if statement looks like it should be:  ***
      //       (dispTHR >= 1440 && dispTHR <= 1560)
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

  // Target RESET
  // The target reset button is "F" on the controller. When the user presses it either 
  // forward or backward, the pwmAUX1 value will go down to something around 1060.
  // Otherwise, it rests at around 1860. 
  // n.b. if the controller itself is not turned on, it is around 1500.
  //
  // Consider anything under 1200 to mean the user is pressing it so we drive
  // the output pin (RESET_PIN) low. Otherwise, we drive it high. This means the
  // reset "pulse" will have a width determined by how long the user holds the button.
  // The relay will catch a pulse of any length so this should not be a problem.
  //
  // n.b. on the robot, the orange and blue wires cokming out of the multi-wire white
  // cable should be connected to RESET_PIN on the Arduino and to the +5V rail on the
  // breadboard. The order doesn't actually matter since current in either direction
  // should activate the relay.
  lcd.setCursor(13, 0);
  if( pwmAUX1 < 1200 ){
    // User is pressing reset button ("F" on controller)
    digitalWrite(RESET_PIN, LOW);
    lcd.print("RST");
  }else{
    digitalWrite(RESET_PIN, HIGH);
    lcd.print("   "); // clear "RST" if button is not being pressed
  }
  
} // end loop()
