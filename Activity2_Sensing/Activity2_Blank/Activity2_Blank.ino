/******************************************************************************
 * EAIT Mining Workshop Activity 2: Sensing
 * 
 * This activity introduces RedBot IR sensors, bumper whiskers and the piezo-
 * electric buzzer
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
// RedBot objects
RedBotMotors motors;
RedBotBumper lBumper = RedBotBumper(3);  // initialzes bumper object on pin 3
RedBotBumper rBumper = RedBotBumper(11); // initialzes bumper object on pin 11
RedBotSensor centre = RedBotSensor(A6); // initialize an IR sensor object on A6

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
 * Emits a sequence of tones using the piezo-electric buzzerto indicate the 
 * RedBot IR sensors have detected a loading point
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
 * Calibrates the RedBot's centre IR sensor to detect the IR values of the  
 * background (colour of the floor surface) and the road surface 
 */
void calibrateIRSensor() {
    playTone("calibrate BG");
    centre.setBGLevel(centre.read());

    playTone("calibrate road");
    centre.setDetectLevel(centre.read());

    delay(2000);
}

/**
 * Checks whether the RedBot's centre IR sensor has detected a loading point. 
 * Returns true when the sensor has detected a loading point, else returns 
 * false
 */
bool detectedLoadingPt() {
    return centre.check();
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
/*                          REDBOT DRIVE FUNCTIONS                           */
/*****************************************************************************/
/**
 * Implements RedBotMotors methods to turn both wheels so that the RedBot 
 * drives forward
 */
void driveForward() {
    motors.drive(100);  // Turn on Left and right motors at forward
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
 * Moves the RedBot forward and stops only when it has detected an obstacle or 
 * a road surface 
 */
void moveRedBot() {
    driveForward(); 
    
    // Replace the if statement condition with a function to tell the RedBot
    // it has detected a loading point (class example)
    if (0) {  // RedBot detected a loading point
        motors.brake(); // Stop the motors
        playTone("detected loading"); // Beep to indicate RedBot detected loading point
        digitalWrite(LED, LOW); // Turn LED off when RedBot is stationary
        exit(1);  // exit the program
    }

    // Replace the if statement condition with a function to tell the RedBot
    // it has detected an obstacle in front of it (student exercise)
    if (0) {  // RedBot detected an obstacle
        motors.brake(); // Stop the motors
        playTone("detected obstacle"); // Beep to indicate detected obstacle
        digitalWrite(LED, LOW); // Turn LED off when RedBot is stationary
        exit(1);  // exit the program
    }
}


/*****************************************************************************/
/*                                  MAIN                                     */
/*****************************************************************************/
void setup() {
    pinMode(LED, OUTPUT); // The RedBot has an LED connected to pin 13. 
    pinMode(BUZZER_PIN, OUTPUT);  // Configures the BUZZER_PIN as an OUTPUT

    // Initialise global variables
    motionState = IDLE; // RedBot begins in an IDLE state
    debounceDuration = 50; // miliseconds
    lastTimeButtonStateChanged = 0; 
    lastButtonState = LOW; // button is LOW when pressed, HIGH when not pressed

    //calibrateIRSensor();
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
