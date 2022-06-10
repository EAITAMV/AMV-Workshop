/******************************************************************************
 * EAIT Mining Workshop Activity 1: Drive forward, turn & reverse
 * 
 * This activity introduce the driving functions to get the RedBot moving 
 * 
 * Hardware setup:
 * The power switch must be on, the motors must be connected, and the board 
 * must be receiving power from the battery. The motor switch must also be 
 * switched to RUN.
 *
 * 04 Feb 2022 R. Danam  
 *****************************************************************************/

// This line "includes" the RedBot library into your sketch. It provides 
// special objects, methods, and functions for the RedBot.
#include <RedBot.h>

#define IDLE 0          // RedBot is IDLE when it is stationary
#define ACTIVE 1        // RedBot is ACTIVE when it is moving
#define BUTTON_PIN 12   // Define BUTTON_PIN as pin 12 on the microcontroller
#define LED 13          // Define LED as pin 13 on the microcontroller

/*****************************************************************************/
/*                            GLOBAL VARIABLES                               */
/*****************************************************************************/

// Instantiate the motor control object. This only needs to be done once
RedBotMotors motors;

// Program variables
unsigned long debounceDuration; // store pushbutton debounce duration
unsigned long lastTimeButtonStateChanged; // record time of button press 
byte buttonState, lastButtonState; // state variables to store button values
int motionState; // state variable to indicate whether RedBot is moving or not


/*****************************************************************************/
/*                          REDBOT CONTROL FUNCTIONS                         */
/*****************************************************************************/
/**
 * Implements the pushbutton as a toggle the RedBot motionState variable 
 * between ACTIVE and IDLE, i.e when the RedBot is moving or stationary, 
 * respectively. Each press of the button will change the motionState variable 
 * from ACTIVE to IDLE or from IDLE to ACTIVE. 
 */
void setMotionState() {
  // Check if difference in current time given by millis() and the last time 
  // the button was pressed is greater than 50ms
  if (millis() - lastTimeButtonStateChanged > debounceDuration) {
    // Read the current button pin value and store this in buttonState
    // buttonState is LOW when button is pressed, HIGH when not pressed
    buttonState = digitalRead(BUTTON_PIN); 
    
    // Check if the current buttonState is not equal to the lastButtonState
    if (buttonState != lastButtonState) {
      // Record the time when the button was pressed
      lastTimeButtonStateChanged = millis(); 
      lastButtonState = buttonState; // Update the lastButtonState value

      if (buttonState == LOW) { // if button is pressed
        // Use a ternary operator to update the motion state
        // syntax: (condition) ? value_if_true : value_if_false
        motionState = (motionState == ACTIVE) ? IDLE: ACTIVE;
      }
    }
  }
}

/*****************************************************************************/
/*                          REDBOT DRIVE FUNCTIONS                           */
/*****************************************************************************/
/**
 * Implements RedBotMotors methods to turn both wheels so that the RedBot 
 * drives forward
 */
void driveForward() {
  motors.drive(125);  // Turn on Left and right motors
  delay(1000);          // delay for 1000 ms
  motors.stop();        // Stops both motors
}

/**
 * Implements RedBotMotors methods to turn both wheels so that the RedBot 
 * moves backward
 */
void reverse() {
  motors.drive(-125);
  delay(1000);    // delay for 1000 ms
  motors.stop();  // brake() motors
  delay(500);     // delay for 500 ms  
}

/**
 * Implements RedBotMotors methods to turn both wheels so that the RedBot 
 * pivots on the spot to the right in a clockwise motion
 */
void pivotRight() {
  motors.rightMotor(-100);    // Turn CCW at motorPower of 100
  motors.leftMotor(-100);     // Turn CCW at motorPower of 100
  delay(500);     // delay for 500 ms    
  motors.brake(); // brake() motors
  delay(1000);    // delay for 1000 ms
}

/**
 * Implements RedBotMotors methods to turn both wheels so that the RedBot 
 * pivots on the spot to the left in a counter-clockwise motion
 */
void pivotLeft() {
  motors.rightMotor(100);    // Turn CW at motorPower of 100
  motors.leftMotor(100);     // Turn CW at motorPower of 100
  delay(500);     // delay for 500 ms
  motors.brake(); // brake() motors
  delay(1000);    // delay for 1000 ms
}

/**
 * Uses the functions above to move the RedBot so it traces out a square or 
 * a rectangle
 */
void moveRedBot() {
  driveForward();

  // Need to tweak the values in rightMotor() & leftMotors() so that it turns
  // 90 degrees accurately
  pivotRight(); 
}

/*****************************************************************************/
/*                                  MAIN                                     */
/*****************************************************************************/
void setup() {
  pinMode(LED, OUTPUT); // The RedBot has an LED connected to pin 13. 

  // Initialise global variables
  motionState = IDLE; // RedBot begins in an IDLE state
  debounceDuration = 50; // miliseconds
  lastTimeButtonStateChanged = 0; 
  lastButtonState = LOW; // button is LOW when pressed, HIGH when not pressed
}

void loop() { 
  setMotionState(); // Checks if pushbutton is pressed and sets motionState
  
  if (motionState == ACTIVE) { // if motionState is ACTIVE
    digitalWrite(LED, HIGH); // Turn LED on when RedBot is moving
    moveRedBot();
  } else {
    digitalWrite(LED, LOW); // Turn LED off when RedBot is stationary
    motors.stop();
  }
}
