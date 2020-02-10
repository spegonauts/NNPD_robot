//----------------------------------------------------------------------------
// NNPD Robot Project
//
// 2020-01-16  Jacob King      - Original version
// 2020-01-17  David Lawrence  - Added comments. Fix some indentation.
// 2020-01-21  David Lawrence  - Added code for RESET button for target
// 2020-02-09  David Lawrence  - Removed unused code and simplified 
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
// but are driven by Victor controllers which use a PWM signal to
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



// Input pins
const byte STR_PIN = 3;
const byte THR_PIN = 2;
const byte AUX_PIN = 12;

// Output pins
const byte LEFT_PINS[]  = { 4,  5,  6,  7};
const byte RIGHT_PINS[] = { 8,  9, 10, 11};
int NLEFT_PINS  = sizeof(LEFT_PINS ); // works since type is byte
int NRIGHT_PINS = sizeof(RIGHT_PINS); // works since type is byte
Servo  left[4]; // (wish we could size these based on size of LEFT_PINS!)
Servo right[4];

const byte RESET_PIN = 13;

// used to record times
volatile unsigned long timerStartSTR, timerStartTHR, timerStartAUX1;
volatile int pulseTimeSTR = 1500;
volatile int pulseTimeTHR = 1500;
volatile int pulseTimeAUX1 = 1500;
unsigned long int pwmAUX1 = 0;

// Variables used to remember the last setting of the left and right PWM
int pwmLEFT;
int pwmRIGHT;

// Initializes the lcd screen, values are for pins
LiquidCrystal lcd(22, 23, 24, 25, 26, 27);

//----------------------------------------
// ISR_STR
//
// ISR function for steering
//----------------------------------------
void ISR_STR() {

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
// ISR_THR
//
// ISR function for throttle
//----------------------------------------
void ISR_THR() {

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
    timerStartAUX1 = now;
  } else {
    if( now > timerStartAUX1 ) {
      pwmAUX1 = now - timerStartAUX1;
    }
    timerStartAUX1 = 0;
  }
}

//----------------------------------------
// setup
//----------------------------------------
void setup() {

  // Input pins
  pinMode(STR_PIN, INPUT_PULLUP);
  pinMode(THR_PIN, INPUT_PULLUP);
  pinMode(AUX_PIN, INPUT_PULLUP);
 
  // ISRs for inputs
  // n.b. the AUX_PIN does not have its own dedicated interrupt so we must use an
  // interrupt group. Hence the use of attachPCINT from the PinChangeInterrupt library.
  attachInterrupt(digitalPinToInterrupt(STR_PIN), ISR_STR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(THR_PIN), ISR_THR, CHANGE);
  attachPCINT(digitalPinToPCINT(AUX_PIN), ISR_AUX1, CHANGE);

  //Setup for lcd and Serial monitor
  lcd.begin(16, 2);
  lcd.print("LEFT:");
  lcd.setCursor(0, 1);
  lcd.print("Right:");
  Serial.begin(9600);

  // Victor setup
  // Victors are common FRC motor controllers, used to recieve and distribute PWM or CAN signals
  // This code uses PWM for simplicity, CAN is far more complicated 

  // The pins are first 'attached' to the motor controllers (tells the servo to look at a specific pin for info)
  // The pin is then set to output mode (the pin will only ever be able to send outputs, never recieve information)
  for(int ipin=0; ipin<NLEFT_PINS; ipin++ ) {
    left[ipin].attach( LEFT_PINS[ipin] );
    pinMode( LEFT_PINS[ipin] , OUTPUT);
  }

  for(int ipin=0; ipin<NRIGHT_PINS; ipin++ ) {
    right[ipin].attach( RIGHT_PINS[ipin] );
    pinMode( RIGHT_PINS[ipin] , OUTPUT);
  }

  pinMode(RESET_PIN, OUTPUT);

} // end setup()


//----------------------------------------
// writeLeft
//
// Set all left motors to same value
//----------------------------------------
void writeLeft(int val) {
  for(int ipin=0; ipin<NLEFT_PINS; ipin++ ) left[ipin].writeMicroseconds(val);
  pwmLEFT = val; // remember for display later
}

//----------------------------------------
// writeRight
//
// Set all right motors to same value
//----------------------------------------
void writeRight(int val) {
  val = 2000 - (val-1000); // reverse right motors
  for(int ipin=0; ipin<NRIGHT_PINS; ipin++ ) right[ipin].writeMicroseconds(val);
  pwmRIGHT = val; // remember for display later
}

//----------------------------------------
// loop
//----------------------------------------
void loop() {

  // map values  -  convert from receiver PWM to motor PWM
  // Values used were found based off of experimentation, may not be %100 accurate
  int dispSTR = map(pulseTimeSTR, 1080, 1876, 1000, 2000);
  int dispTHR = map(pulseTimeTHR, 1100, 1888, 1000, 2000);

  dispSTR = constrain(dispSTR, 1000, 2000);
  dispTHR = constrain(dispTHR, 1000, 2000);

  // Print last PWM values written to drive motors to lcd screen
  lcd.setCursor(7, 0);
  lcd.print(pwmLEFT);
  lcd.setCursor(7, 1);
  lcd.print(pwmRIGHT);

  // drive
  //
  // Here values are written to the drive motors.
  // - A deadband is implmented for both the STR and THR.
  // - Values written are in the standard servo range of 1000-2000 with 1500 representing no motion.
  // - The motors are set using a linear sum of the two controls (when both active).
  // - If only the STR is active, then the robot will spin at a rate proportional to the STR.

  // The following are in PWM units, but in the range -500 to 500. Linear
  // combinations are used to set the motor speeds/polarities
  int  leftSTR  = dispSTR - 1500;
  int  rightSTR = -leftSTR;
  int  leftTHR  = dispTHR - 1500;
  int  rightTHR = dispTHR - 1500;

  bool STR_in_deadband = (dispSTR >= 1440) && (dispSTR <= 1560);
  bool THR_in_deadband = (dispTHR >= 1440) && (dispTHR <= 1560);

  if( STR_in_deadband ) leftSTR = rightSTR = 0; // zero out STR values if in deadband
  if( THR_in_deadband ) leftTHR = rightTHR = 0; // zero out THR values if in deadband

  writeLeft(  constrain( leftSTR +  leftTHR + 1500, 1000, 2000) );
  writeRight( constrain(rightSTR + rightTHR + 1500, 1000, 2000) );

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
