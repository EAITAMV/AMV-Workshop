/******************************************************************************
 * EAIT Mining Workshop Activity 3: Line Follower
 * 
 * Code to be implemented on RedBot to simulate an automated mine truck on 
 * a haul and shovel route 
 *
 * 04 Feb 2022 R. Danam
 *****************************************************************************/
#include <RedBot.h>

#define IDLE 0          // RedBot is IDLE when it is stationary
#define ACTIVE 1        // RedBot is ACTIVE when it is moving
#define BUZZER_PIN 9    // Define BUZZER_PIN as pin 9 on the microcontroller
#define BUTTON_PIN 12   // Define BUTTON_PIN as pin 12 on the microcontroller
#define LED 13          // Define LED as pin 13 on the microcontroller

/*****************************************************************************/
/*                            GLOBAL VARIABLES                               */
/*****************************************************************************/
// Initialise RedBot motor variables
RedBotMotors motors;
int driveSpeed; // speed of the RedBot

// Initialise RedBot sensor objects
RedBotBumper lBumper = RedBotBumper(3);  
RedBotBumper rBumper = RedBotBumper(11);
RedBotEncoder encoder = RedBotEncoder(A2, 10);
RedBotSensor IRSensorLeft = RedBotSensor(A3);
RedBotSensor IRSensorMid = RedBotSensor(A6);
RedBotSensor IRSensorRight = RedBotSensor(A7);

// Program variables
unsigned long debounceDuration; // store pushbutton debounce duration
unsigned long lastTimeButtonStateChanged; // record time of button press 
byte buttonState, lastButtonState; // state variables to store button values
int lBumperState, rBumperState;  // state variables to store the bumper values
int motionState; // state variable to indicate whether RedBot is moving or not


/*****************************************************************************/
/*                             BUZZER FUNCTIONS                              */
/*****************************************************************************/
/**
 * Implements the piezo-electric buzzer as a RedBot horn
 */
void beepHorn() {
    tone(BUZZER_PIN, 1000);
    delay(80);
    noTone(BUZZER_PIN);
    delay(100);
    tone(BUZZER_PIN, 1000);
    delay(700);
    noTone(BUZZER_PIN);
}

/**
 * Emits a sequence of tones using the piezo-electric buzzer to indicate the 
 * RedBot calibrating its IR sensor to recognise the IR value of the floor
 */
void toneCalibrateBG() {
    tone(BUZZER_PIN, 587.33); 
    delay(500);
    tone(BUZZER_PIN, 739.989);
    delay(500);
    tone(BUZZER_PIN, 880);
    delay(500);
    noTone(BUZZER_PIN);
    delay(1000);
}

/**
 * Emits a sequence of tones using the piezo-electric buzzer to indicate the 
 * RedBot calibrating its IR sensor to recognise the IR value of the road
 */
void toneCalibrateRoad() {
    tone(BUZZER_PIN, 587.33); 
    delay(180);
    noTone(BUZZER_PIN);
    delay(30);
    tone(BUZZER_PIN, 587.33); 
    delay(60);
    noTone(BUZZER_PIN);
    delay(360);

    tone(BUZZER_PIN, 587.33); 
    delay(180);
    noTone(BUZZER_PIN);
    delay(30);
    tone(BUZZER_PIN, 587.33); 
    delay(60);
    noTone(BUZZER_PIN);
    delay(1440);
}

/**
 * Emits a sequence of tones using the piezo-electric buzzer to indicate the 
 * RedBot has detected an obstacle directly in front of it
 */
void toneFoundObstacle() {
    tone(BUZZER_PIN, 1000);
    delay(80);
    noTone(BUZZER_PIN);
    delay(100);
    tone(BUZZER_PIN, 1000);
    delay(700);
    noTone(BUZZER_PIN);
}

/**
 * Emits a sequence of tones using the piezo-electric buzzer to indicate the 
 * RedBot IR sensors have detected a loading surface
 */
void toneFoundLoading() {
    tone(BUZZER_PIN, 1000);
    delay(100);
    noTone(BUZZER_PIN);
    delay(80);
    tone(BUZZER_PIN, 2000);
    delay(700);
    noTone(BUZZER_PIN);
}

/**
 * Function to play a specific tone that is chosen from the given String type 
 * variable
 */
void playTone(String type) {
    if (type == "calibrate BG") {
        for (int i = 0; i < 2; i++) {
            toneCalibrateBG();
        }
    }
    if (type == "calibrate road") {
        for (int i = 0; i < 2; i++) {
            toneCalibrateRoad();
        }
    }
    if (type == "detected obstacle") {
        toneFoundObstacle();
    }
    if (type == "detected loading") {
        toneFoundLoading();
    }
}


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
                // Update the motion state
                if (motionState == ACTIVE) { 
                    motionState = IDLE;
                } else {
                    motionState = ACTIVE;
                    beepHorn();
                }
            }
        }
    }
}

/**
 * Calibrates the RedBot's three IR sensors to detect the IR values of the  
 * background (colour of the floor surface) and the road surface 
 */
void calibrateIRSensor() {
    playTone("calibrate BG");
    IRSensorLeft.setBGLevel(IRSensorLeft.read());
    IRSensorMid.setBGLevel(IRSensorMid.read());
    IRSensorRight.setBGLevel(IRSensorRight.read());

    playTone("calibrate road");
    IRSensorLeft.setDetectLevel(IRSensorLeft.read());
    IRSensorMid.setDetectLevel(IRSensorMid.read());
    IRSensorRight.setDetectLevel(IRSensorRight.read());

    delay(2000);
}

/**
 * Checks whether the RedBot IR sensors has detected a loading point. Returns 
 * true when the sensors have detected a loading point, else returns false
 */
bool detectedLoadingPt() {
    return IRSensorLeft.check() && IRSensorMid.check() && IRSensorRight.check();
}

/**
 * Checks whether the RedBot has detected an obstacle directly in front of it 
 * by using the bumper whiskers as sensors. Returns true when the either 
 * whisker has bumped an obstacle in front of it, else returns false
 */
bool detectedObstacle() {
    // default bumper state is HIGH, it is LOW when bumped
    lBumperState = lBumper.read();  // left bumper state
    rBumperState = rBumper.read();  // right bumper state

    // if either left or right whiskers are bumped
    if (lBumperState == LOW || rBumperState == LOW) {
        return true; // left or right whiskers have been bumped
    } else {
        return false; // left or right whiskers have NOT been bumped
    }
}


/*****************************************************************************/
/*                           IR SENSOR FUNCTIONS                             */
/*****************************************************************************/
/**
 * Returns true if the RedBot is veering too far to the right of the line by 
 * by comparing IR Sensor readings from the left, mid and right sensors. Else,
 * returns false
 */
bool veeringRight() {
    return IRSensorLeft.read() >= IRSensorMid.read() &&
           IRSensorLeft.read() >= IRSensorRight.read();
}

/**
 * Returns true if the RedBot is veering too far to the left of the line by 
 * by comparing IR Sensor readings from the left, mid and right sensors. Else,
 * returns false
 */
bool veeringLeft() {
    return IRSensorRight.read() >= IRSensorMid.read() &&
           IRSensorRight.read() >= IRSensorLeft.read();
}


/*****************************************************************************/
/*                             DRIVE FUNCTIONS                               */
/*****************************************************************************/
/**
 * Commands the robot to turn right using the given currentDriveSpeed integer
 */
void turnRight(int currentDriveSpeed) {  
    // While the robot is veering too far to the left of the line
    while (veeringLeft()) {  
        // turn the robot to the right
        motors.leftMotor(-currentDriveSpeed);
        motors.rightMotor(-currentDriveSpeed);
    }
    motors.brake();
}

/**
 * Commands the robot to turn right using the given currentDriveSpeed integer
 */
void turnLeft(int currentDriveSpeed) {
  // While the robot is veering too far to the right of the line
    while (veeringRight()) { 
        // turn the robot to the left
        motors.rightMotor(currentDriveSpeed);
        motors.leftMotor(currentDriveSpeed);
    }
    motors.brake();
}

/*
 * Command the robot to drive straight in a forward direction for 
 * a constant distance in milimetres with a constant speed defined 
 * by motorPower between 1-255
 */
void driveStraight(float distance, int motorPower) {
    long lCount = 0;     // left encoder count
    long rCount = 0;     // right encoder count
    long targetCount;    // distance converted to encoder counts
    float numRev;        // number of wheel rotations 

    // variables for tracking the left and right encoder counts
    long prevlCount, prevrCount;

    // difference between current encoder count and previous count
    long lDiff, rDiff;

    // variables for setting left and right motor power
    int leftPower = motorPower;
    int rightPower = motorPower;

    // variable used to offset motor power on right vs left to keep straight.
    int offset = 5;   // offset amount to compensate Right vs. Left drive
    
    numRev = distance / (PI * 65);    // calculate the target # of rotations
    targetCount = numRev * 192;   // calculate the target count

    encoder.clearEnc(BOTH);   // clear the encoder count
    delay(5);   // short delay before starting the motors.
    
    motors.drive(motorPower);   // start motors 

    // while the right encoder is less than the target count
    while (rCount < targetCount) {
        // the encoder values and wait -- this is a holding loop.
        lCount = encoder.getTicks(LEFT);
        rCount = encoder.getTicks(RIGHT);

        motors.leftDrive(leftPower);
        motors.rightDrive(rightPower);

        // calculate the rotation "speed" as a difference in the count from 
        // previous cycle.
        lDiff = (lCount - prevlCount);
        rDiff = (rCount - prevrCount);

        // store the current count as the "previous" count for the next cycle.
        prevlCount = lCount;
        prevrCount = rCount;

        // if left is faster than the right, 
        // slow down the left / speed up right
        if (lDiff > rDiff) {
            leftPower = leftPower - offset;
            rightPower = rightPower + offset;
        }
        // if right is faster than the left, 
        // speed up the left / slow down right
        else if (lDiff < rDiff) {
            leftPower = leftPower + offset;  
            rightPower = rightPower - offset;
        }
        delay(10);  // short delay to give motors a chance to respond.
    }
    motors.brake();  
}


/*****************************************************************************/
/*                         LINE FOLLOWING ALGORITHM                          */
/*****************************************************************************/
/**
 * Make the RedBot operate as an autonomous mining vehicle. Features:
 *  - Follows a line marked out on the ground as a mining route
 *  - Stops upon detecting obstacles and loading ore
 *  - Activates piezo-electric buzzer as a mining vehicle horn
 */
void moveRedBot() {
    // We are now using a more accurate function to drive straight
    driveStraight(2, driveSpeed);

    // turn right if veering too far to the left of the line
    if (0) { // Replace with the correct function!
        turnRight(driveSpeed);
    }
    // turn left if veering too far to the right of the line
    if (0) { // Replace with the correct function!
        turnLeft(driveSpeed);
    }
    
    // RedBot detected an obstacle
    if (0) { // Replace with the correct function!
        motors.brake(); // Stop the motors
        playTone("detected obstacle"); // Beep to indicate detected obstacle
        delay(5000); // delay for five seconds
        beepHorn(); // Beep before moving again
    }
    // RedBot detected a loading point
    if (0) { // Replace with the correct function!
        motors.brake(); // Stop the motors
        playTone("detected loading"); // Beep to indicate RedBot detected a loading point
        delay(5000); // delay for five seconds
        beepHorn(); // Beep before moving again
        driveStraight(30, driveSpeed); // Move past the loading point
    }
}

/*****************************************************************************/
/*                                  MAIN                                     */
/*****************************************************************************/
void setup() {   
    // configures the button as an INPUT where INPUT_PULLUP defaults the pin 
    // to HIGH
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(LED, OUTPUT); // configures LED pin as an output

    // Initialise global variables
    driveSpeed = 100; // driveSpeed value can be between 0-255
    motionState = IDLE; // RedBot begins in an IDLE state
    debounceDuration = 50; // miliseconds
    lastTimeButtonStateChanged = 0; 
    lastButtonState = LOW; // button is LOW when pressed, HIGH when not pressed
    
    calibrateIRSensor();
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
