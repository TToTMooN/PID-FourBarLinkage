// ME350 Ball Handling Sketch

//////////////////////////////////////////////
// DEFINE CONSTANTS AND GLOBAL VARIABLES:   //
//////////////////////////////////////////////

//** State Machine: **//
// CONSTANTS: 
// Definition of states in the state machine
const int CALIBRATE        = 1;
const int DONE_CALIBRATE   = 2;
const int WAIT             = 3;
const int MOVE_TO_CHUTE    = 4;
const int WAIT_FOR_BALL    = 5;
const int PUT_BALL         = 6;
const int SHOOT_BALL       = 7;

// VARIABLES:
// Global variable that keeps track of the state:
// Start the state machine in calibration state:
int  state = CALIBRATE;

//** Color Sensor: **//
// Include the necessary code headers:
#include "Adafruit_TCS34725.h"
#include <Wire.h>
// CONSTANTS: 
// Definition of ball types:
const int MAIZE = 1; 
const int BLUE  = 2; 
const int RED   = 3; 
const int WHITE = 4; 
const int NONE  = 5; 
// VARIABLES:
// Create a variable that allows us to access the color sensor:
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
// Return values from the sensor
uint16_t red;
uint16_t green;
uint16_t blue;
uint16_t clear;

//** Computation of position and velocity: **//
// CONSTANTS: 
// Settings for velocity computation:
const int MIN_VEL_COMP_COUNT = 2;     // [encoder counts] Minimal change in motor position that must happen between two velocity measurements
const long MIN_VEL_COMP_TIME = 10000; // [microseconds] Minimal time that must pass between two velocity measurements
// VARIABLES:
volatile int motorPosition = 0; // [encoder counts] Current motor position (Declared 'volatile', since it is updated in a function called by interrupts)
volatile int encoderStatus = 0; // [binary] Past and Current A&B values of the encoder  (Declared 'volatile', since it is updated in a function called by interrupts)
// The rightmost two bits of encoderStatus will store the encoder values from the current iteration (A and B).
// The two bits to the left of those will store the encoder values from the previous iteration (A_old and B_old).
float motorAcceleration    = 0; // [encoder counts / seconds^2] Current motor acceleration
float motorVelocity        = 0; // [encoder counts / seconds] Current motor velocity 
float previousMotorVelocity= 0; // [encoder counts / seconds] Previous motor velocity 
int previousMotorPosition  = 0; // [encoder counts] Motor position the last time a velocity was computed 
long previousVelCompTime   = 0; // [microseconds] System clock value the last time a velocity was computed 

//** High-level behavior of the controller:  **//
// CONSTANTS:
// Target positions:
const int CALIBRATION_VOLTAGE = -2;                // [Volt] Motor voltage used during the calibration process
const int WAIT_POSITION       = 360;                // [encoder counts] Motor position corresponding to a wait position near the two chutes
const int CHUTE_1_POSITION    = 0;                // [encoder counts] Motor position corresponding to first chute
const int CHUTE_2_POSITION    = 375;                // [encoder counts] Motor position corresponding to second chute
const int PUT_POSITION        = 930;                // [encoder counts] Motor position corresponding to basket lane
const int LOWER_BOUND         = CHUTE_1_POSITION; // [encoder counts] Position of the left end stop
const int UPPER_BOUND         = PUT_POSITION;     // [encoder counts] Position of the right end stop
const int TARGET_BAND         = 10;                // [encoder counts] "Close enough" range when moving towards a target.
// Timing:
const long  WAIT_TIME         = 300000; // [microseconds] Time waiting for the ball to drop.
// VARIABLES:
int activeChutePosition;     // [encoder counts] position of the currently active chute
unsigned long startWaitTime; // [microseconds] System clock value at the moment the WAIT_FOR_BALL state started
unsigned long startCalibrationTime; // [microseconds] System clock value at the moment the WAIT_FOR_BALL state started
unsigned long startShootTime;// [microseconds] System clock value at the moment the SHOOT_BALL state started

//** PID Controller  **//
// CONSTANTS:
float KP             = 0.025; // [Volt / encoder counts] P-Gain
float KI             = 0.01; // [Volt / (encoder counts * seconds)] I-Gain
float KD             = 0.0015; // [Volt * seconds / encoder counts] D-Gain
const float SUPPLY_VOLTAGE = 12; // [Volt] Supply voltage at the HBridge
const float BASE_CMD       = 4; // [Volt] Voltage needed to overcome friction
// VARIABLES:
int  targetPosition  = 0; // [encoder counts] desired motor position
float positionError  = 0; // [encoder counts] Position error
float integralError  = 0; // [encoder counts * seconds] Integrated position error
float velocityError  = 0; // [encoder counts / seconds] Velocity error
float accelerationError  = 0; // [encoder counts / seconds] Acceleration error
float desiredVoltage = 0; // [Volt] Desired motor voltage
int   motorCommand   = 0; // [0-255] PWM signal sent to the motor
unsigned long executionDuration = 0; // [microseconds] Time between this and the previous loop execution.  Variable used for integrals and derivatives
unsigned long lastExecutionTime = 0; // [microseconds] System clock value at the moment the loop was started the last time

//** Gravity Compensation Lookup Table: **//
// CONSTANTS: 
const float FF_BALANCED_POSITION   = 375; // [encoder counts] Position at which the device is fully balanced. 
const float FF_VOLTAGE_LOWER_BOUND = 2; // [Volt] Voltage to be applied at the left endstop 
const float FF_VOLTAGE_UPPER_BOUND = -2; // [Volt] Voltage to be applied at the right endstop 

//** Pin assignment: **//
// CONSTANTS:
const int PIN_NR_ENCODER_A        = 2;  // Never change these, since the interrupts are attached to pin 2 and 3
const int PIN_NR_ENCODER_B        = 3;  // Never change these, since the interrupts are attached to pin 2 and 3
const int PIN_NR_DROP_REQ         = 4;
const int PIN_NR_ON_OFF_SWITCH    = 5;
const int PIN_NR_CHUTE_1_READY    = 6;
const int PIN_NR_CHUTE_2_READY    = 7;
const int PIN_NRL_LIMIT_SWITCH    = 8;
const int PIN_NR_PWM_OUTPUT       = 9;
const int PIN_NR_PWM_DIRECTION_1  = 10;
const int PIN_NR_PWM_DIRECTION_2  = 11;
// End of CONSTANTS AND GLOBAL VARIABLES


//////////////////////////////////////////////////////////////////////////////////////////
// The setup() function is called when a sketch starts. Use it to initialize variables, //
// pin modes, start using libraries, etc. The setup function will only run once, after  //
// each powerup or reset of the Arduino board:                                          //
//////////////////////////////////////////////////////////////////////////////////////////
void setup() {
    // Declare which digital pins are inputs and which are outputs:
    pinMode(PIN_NR_ENCODER_A,        INPUT);
    pinMode(PIN_NR_ENCODER_B,        INPUT);
    pinMode(PIN_NR_CHUTE_1_READY,    INPUT); 
    pinMode(PIN_NR_CHUTE_2_READY,    INPUT); 
    pinMode(PIN_NR_ON_OFF_SWITCH,    INPUT);
    pinMode(PIN_NRL_LIMIT_SWITCH,    INPUT);
    pinMode(PIN_NR_DROP_REQ,         OUTPUT);
    pinMode(PIN_NR_PWM_OUTPUT,       OUTPUT);
    pinMode(PIN_NR_PWM_DIRECTION_1,  OUTPUT);
    pinMode(PIN_NR_PWM_DIRECTION_2,  OUTPUT);
  
    // Turn on the pullup resistors on the encoder channels
    digitalWrite(PIN_NR_ENCODER_A, HIGH);  
    digitalWrite(PIN_NR_ENCODER_B, HIGH);
  
    // Activate interrupt for encoder pins.
    // If either of the two pins changes, the function 'updateMotorPosition' is called:
    attachInterrupt(0, updateMotorPosition, CHANGE);  // Interrupt 0 is always attached to digital pin 2
    attachInterrupt(1, updateMotorPosition, CHANGE);  // Interrupt 1 is always attached to digital pin 3
  
    // Begin serial communication for monitoring.
    Serial.begin(115200);
    Serial.println("Start Executing Program.");
  
    // Begin the operation of the color sensor and check if it works.
    if (tcs.begin()) {
        Serial.println("Color sensor found");
    } else {
        Serial.println("Color sensor not found.  Please check your connections");
        while (1); // infinite loop to halt the program
    }
    
    // Initialize outputs:
    // Set the dropRequestSignal to low:
    digitalWrite(PIN_NR_DROP_REQ, LOW);
    // Set initial output to the motor to 0
    analogWrite(PIN_NR_PWM_OUTPUT, 0);
}
// End of function setup()


////////////////////////////////////////////////////////////////////////////////////////////////
// After going through the setup() function, which initializes and sets the initial values,   //
// the loop() function does precisely what its name suggests, and loops consecutively,        //
// allowing your program to sense and respond. Use it to actively control the Arduino board.  //
//////////////////////////////////////////////////////////////////////////////////////////////// 
void loop() {
    // Determine the duration it took to execute the last loop. This time is used 
    // for integration and for monitoring the loop time via the serial monitor.
    executionDuration = micros() - lastExecutionTime;
    lastExecutionTime = micros();
  
    // Speed Computation:
    if ((abs(motorPosition - previousMotorPosition) > MIN_VEL_COMP_COUNT) || (micros() - previousVelCompTime) > MIN_VEL_COMP_TIME){
        // If at least a minimum time interval has elapsed or
        // the motor has travelled through at least a minimum angle ... 
        // .. compute a new value for speed:
        // (speed = delta angle [encoder counts] divided by delta time [seconds])
        long now = micros();
        motorVelocity = (double)(motorPosition - previousMotorPosition) * 1000000 / 
                                (now - previousVelCompTime);

        motorAcceleration = (double)(motorVelocity - previousMotorVelocity) * 1000000 / 
                                (now - previousVelCompTime);
        
        // Remember this encoder count and time for the next iteration:
        previousVelCompTime   = now;
        previousMotorPosition = motorPosition;
        previousMotorVelocity = motorVelocity;
    }
    //******************************************************************************//
    // The state machine:
    switch (state) {
        //****************************************************************************//
        // In the CALIBRATE state 1, we move the mechanism to a position outside of the 
        // work space (towards the limit switch).  Once the limit switch is on and 
        // the motor stopped turning, we know that we are against the end stop
        case CALIBRATE:
            // We don't have to do anything here since this state is only used to set
            // a fixed output voltage.  This happens further below.
            
            // Decide what to do next:
            if (digitalRead(PIN_NRL_LIMIT_SWITCH)==HIGH) {
                startCalibrationTime = micros();
                state = DONE_CALIBRATE;
            } 
            // Otherwise we continue calibrating
        break;
        
        case DONE_CALIBRATE:
            
            // Decide what to do next:
            if (micros() - startCalibrationTime > 3000000) { 
                // We reached the endstop.  Update the motor position to the limit:
                // (NOTE: If the limit switch is on the right, this must be UPPER_BOUND)
                motorPosition = LOWER_BOUND;  
                // Reset the error integrator:
                integralError = 0;
                // Calibration is finalized. Transition into WAIT state
                Serial.println("State transition from CALIBRATE to WAIT");
                state = WAIT;
                targetPosition = 0;
            } 
            // Otherwise we continue calibrating
        break;
        //****************************************************************************//
        // In the WAIT state 3, we receive the signal from the arena, to know which chute is activated.
        case WAIT:
            // Set the target position to a neutral position near the chutes:
            //targetPosition = WAIT_POSITION;
            
            // Decide what to do next:
            if (digitalRead(PIN_NR_CHUTE_1_READY) == HIGH) {  // Update this
                // Chute 1 signaled to be ready to drop a ball.  Set the position of chute
                // 1 as the position of the active chute:
                activeChutePosition = CHUTE_1_POSITION;
                // Transit into MOVE_TO_CHUTE state:
                Serial.println("State transition from WAIT to MOVE_TO_CHUTE. Active chute = 1!");
                state = MOVE_TO_CHUTE;
            }
            if (digitalRead(PIN_NR_CHUTE_2_READY) == HIGH) {  // Update this
                // Chute 2 signaled to be ready to drop a ball.  Set the position of chute
                // 2 as the position of the active chute:
                activeChutePosition = CHUTE_2_POSITION;
                // Transit into MOVE_TO_CHUTE state:
                Serial.println("State transition from WAIT to MOVE_TO_CHUTE. Active chute = 2!");
                state = MOVE_TO_CHUTE;
            }
            // Otherwise we continue waiting
        break;
    
        //****************************************************************************//
        // In the MOVE_TO_CHUTE state 4, we move the cup under one of the two chutes 
        // (indicated by the variable active_chute). Once the position was reached 
        // (with some error) and the motor stopped turning, we know that we are under
        // the chute.
        case MOVE_TO_CHUTE:
            // Set the target position to chute 1 or 2:
            targetPosition = activeChutePosition;
            if (activeChutePosition == CHUTE_1_POSITION){
                KP = 0.005;
                KI = 0.005;
                KD = 0.0015;
                if ( abs(motorPosition - targetPosition)< 20 && motorVelocity==0) {  // Update this
                    // We reached the chute.  Ask the playing field to drop a ball by 
                    // setting chuteActivateSignal to HIGH:
                    digitalWrite(PIN_NR_DROP_REQ, HIGH);
                    // Start waiting timer:
                    startWaitTime = micros();
                    // Transition into WAIT_FOR_BALL state
                    Serial.println("State transition from MOVE_TO_CHUTE to WAIT_FOR_BALL");
                    state = WAIT_FOR_BALL;
                } 
            }
            else if (activeChutePosition == CHUTE_2_POSITION){
                KP = 0.025;//Can make it smaller Revise
                KI = 0.005;
                KD = 0.001;
                if ( abs(motorPosition - targetPosition) < 5 && motorVelocity==0) {  // Update this
                    // We reached the chute.  Ask the playing field to drop a ball by 
                    // setting chuteActivateSignal to HIGH:
                    digitalWrite(PIN_NR_DROP_REQ, HIGH);
                    // Start waiting timer:
                    startWaitTime = micros();
                    // Transition into WAIT_FOR_BALL state
                    Serial.println("State transition from MOVE_TO_CHUTE to WAIT_FOR_BALL");
                    state = WAIT_FOR_BALL;
                } 
            }
            // Decide what to do next:
            
            // Otherwise we continue moving towards the chute
        break;
    
        //****************************************************************************//
        // In this state 5, we stay at the chute and wait until the ball was dropped:
        case WAIT_FOR_BALL:
          // The target position remains at either chute 1 or 2:
          targetPosition = activeChutePosition;
          if (targetPosition == CHUTE_1_POSITION){
              motorPosition = CHUTE_1_POSITION;
              integralError = 0;
          }
          // Decide what to do next:
          if (micros()-startWaitTime>WAIT_TIME) {
              // We have waited long enough for the ball to drop. By now we should have
              // recieved it. 
              // Set the chuteActivateSignal back to LOW.
              digitalWrite(PIN_NR_DROP_REQ, LOW);
              // Check for the ball color and output to serial:
              int ballColor = evaluateColorSensor();
              Serial.print("Ball color is: ");
              Serial.print("  R: "); 
              Serial.print(red);
              Serial.print("  G: "); 
              Serial.print(green);
              Serial.print("  B: "); 
              Serial.print(blue);
              Serial.print("  C: "); 
              Serial.print(clear);
              
              switch (ballColor) {
                  case MAIZE: Serial.println("MAIZE."); state = PUT_BALL; break;
                  case BLUE:  Serial.println("BLUE.");  state = PUT_BALL; break;
                  case RED:   Serial.println("RED.");   state = SHOOT_BALL; startShootTime = micros(); break;
                  case WHITE: Serial.println("WHITE."); state = SHOOT_BALL; startShootTime = micros(); break;
                  case NONE:  Serial.println("NONE.");  break;
              }
              // Transition into PUT_BALL state
              Serial.println("State transition from WAIT_FOR_BALL to PUT_BALL");
              if (micros() - startWaitTime > 3000000){//revise
                  state = CALIBRATE;
              }
          } 
          // Otherwise we continue waiting for the ball
          break;
    
        //****************************************************************************//
        // In this state 6, we move the cup to drop it in the basket:
        case PUT_BALL:
            // Set the target position to chute 1 or 2:
            targetPosition = PUT_POSITION;
            if (activeChutePosition == CHUTE_1_POSITION) {
                KP = 0.02;
                KI = 0.00;
                KD = 0.0035;
            }
            else if (activeChutePosition == CHUTE_2_POSITION){
                KP = 0.03;
                KI = 0.001;
                KD = 0.004;
            }
            // Decide what to do next:
            if (abs(motorPosition - targetPosition)< 40 && motorVelocity==0) {  // Update this
                // We reached the basket and dropped the ball.
                // Transition into WAIT state to restart the cycle
                Serial.println("State transition from PUT_BALL to WAIT");
                state = WAIT;
            } 
            // Otherwise we continue moving towards the chute
        break;
        
        //****************************************************************************//
        // In this state 7, we move the cup to drop it in the net:
        case SHOOT_BALL:
            if (activeChutePosition == CHUTE_1_POSITION) {
                targetPosition = PUT_POSITION;
                KP = 0.027;
                KI = 0.01;
                KD = 0.003;
                if (abs(motorPosition - targetPosition)< 40 && motorVelocity==0) {  // Update this
                    // We reached the basket and dropped the ball.
                    // Transition into WAIT state to restart the cycle
                    Serial.println("State transition from PUT_BALL to WAIT");
                    state = WAIT;
                } 
            }
            else if (activeChutePosition == CHUTE_2_POSITION){
                targetPosition = 700;
                KP = 0.4;
                KI = 0.001;
                KD = 0.0015;
                if ( motorVelocity < -1000) {  // Update this
                    // We reached the basket and dropped the ball.
                    // Transition into WAIT state to restart the cycle
                    Serial.println("State transition from PUT_BALL to WAIT");
                    state = WAIT;
                } 
            }
            // Decide what to do next:
            // Otherwise we continue moving towards the chute
        break;
        //****************************************************************************//
        // We should never reach the next bit of code, which would mean that the state
        // we are currently in doesn't exist.  So if it happens, throw an error and 
        // stop the program:
        default: 
            Serial.println("Statemachine reached at state that it cannot handle.  ABORT!!!!");
            Serial.print("Found the following unknown state: ");
            Serial.println(state);
            while (1); // infinite loop to halt the program
        break;
    }
    // End of the state machine.
    //******************************************************************************//
    
   
    //******************************************************************************//
    // Position Controller
    if (digitalRead(PIN_NR_ON_OFF_SWITCH)==HIGH && (!(state == WAIT_FOR_BALL)) && (!(state == DONE_CALIBRATE))) {
        // If the toggle switch is on, run the controller:
        //** PID control: **//  
        // Compute the position error [encoder counts]
        positionError = targetPosition - motorPosition;
        // Compute the integral of the position error  [encoder counts * seconds]
        integralError = integralError + positionError * (float)(executionDuration) / 1000000; 
        // Compute the velocity error (desired velocity is 0) [encoder counts / seconds]
        velocityError = 0 - motorVelocity;
        // This is the actual controller function that uses the error in 
        // position and velocity and the integrated error and computes a
        // desired voltage that should be sent to the motor:
        desiredVoltage = KP * positionError +  
                         KI * integralError +
                         KD * velocityError;
     
        //** Feedforward terms: **//
        // Compensate for friction.  That is, if we now the direction of 
        // desired motion, add a base command that helps with moving in this
        // direction:
        if (positionError <= -5) {
            desiredVoltage = desiredVoltage - BASE_CMD;
        }
        if (positionError >= +5) {
            desiredVoltage = desiredVoltage + BASE_CMD;
        }
        // Gravity compensation lookup.  Here we record which voltage we need
        // to keep the device balanced at the left and at the right, and note 
        // where it is balanced passively.  The feedforward value is determined
        // by linear interpolation between these three points.
        if (motorPosition < FF_BALANCED_POSITION) {
            desiredVoltage = desiredVoltage + (FF_BALANCED_POSITION-motorPosition)/(FF_BALANCED_POSITION-LOWER_BOUND)*FF_VOLTAGE_LOWER_BOUND;
        }
        if (motorPosition > FF_BALANCED_POSITION) {
            desiredVoltage = desiredVoltage + (motorPosition-FF_BALANCED_POSITION)/(UPPER_BOUND-FF_BALANCED_POSITION)*FF_VOLTAGE_UPPER_BOUND;
        }
    
        // Anti-Wind-Up
        if (abs(desiredVoltage) > SUPPLY_VOLTAGE) {
            // If we are already saturating our output voltage, it does not make
            // sense to keep integrating the error (and thus ask for even higher
            // and higher output voltages).  Instead, stop the inegrator if the 
            // output saturates. We do this by reversing the summation at the 
            // beginning of this function block:
            integralError = integralError - positionError * (float)(executionDuration) / 1000000; 
        }
        // End of 'if(onOffSwitch==HIGH)'
        
        // Override the computed voltage during calibration.  In this state, we simply apply a 
        // fixed voltage to move against one of the end-stops.
        if (state == CALIBRATE) {
            // If the toggle switch is on, run the controller:
            //** PID control: **//  
            desiredVoltage = 0.01 * (-1300 - motorVelocity) +  
                             0.01 * motorPosition +
                             0.0001 * (-motorAcceleration);
            //** Feedforward terms: **//
            // Compensate for friction.  That is, if we now the direction of 
            // desired motion, add a base command that helps with moving in this
                desiredVoltage = desiredVoltage - BASE_CMD;
            // Gravity compensation lookup.  
            if (motorPosition < FF_BALANCED_POSITION) {
                desiredVoltage = desiredVoltage + (FF_BALANCED_POSITION-motorPosition)/(FF_BALANCED_POSITION-LOWER_BOUND)*FF_VOLTAGE_LOWER_BOUND;
            }
            if (motorPosition > FF_BALANCED_POSITION) {
                desiredVoltage = desiredVoltage + (motorPosition-FF_BALANCED_POSITION)/(UPPER_BOUND-FF_BALANCED_POSITION)*FF_VOLTAGE_UPPER_BOUND;
            }
        }
    } 
    else { 
        // Otherwise, the toggle switch is off, so do not run the controller, 
        // stop the motor...
        desiredVoltage = 0; 
        // .. and reset the integrator of the error:
        integralError = 0;
        // Produce some debugging output:
        Serial.println("The toggle switch is off.  Motor Stopped.");
    } 
    // End of  else onOffSwitch==HIGH
    
    //** Send signal to motor **//
    // Convert from voltage to PWM cycle:
    motorCommand = int(abs(desiredVoltage * 255 / SUPPLY_VOLTAGE));
    // Clip values larger than 255
    if (motorCommand > 255) {
        motorCommand = 255;
    }
    // Send motor signals out
    analogWrite(PIN_NR_PWM_OUTPUT, motorCommand);
    // Determine rotation direction
    if (desiredVoltage >= 0) {
        // If voltage is positive ...
        // ... turn forward
        digitalWrite(PIN_NR_PWM_DIRECTION_1,LOW);  // rotate forward
        digitalWrite(PIN_NR_PWM_DIRECTION_2,HIGH); // rotate forward
    } else {
        // ... otherwise turn backward:
        digitalWrite(PIN_NR_PWM_DIRECTION_1,HIGH); // rotate backward
        digitalWrite(PIN_NR_PWM_DIRECTION_2,LOW);  // rotate backward
    }
    // End of Position Controller
    //*********************************************************************//
    
    // Print out current controller state to Serial Monitor.
    printStateToSerial();
}
// End of main loop
//***********************************************************************//


//////////////////////////////////////////////////////////////////////
// This is a function that returns the type of ball found in the    //
// cup.  It is called from the loop()-routine.  It returns one of   //
// the following values:                                            //
// 'MAIZE', 'BLUE', 'RED', 'WHITE', 'NONE'.                         //
// This function is not completed.  It currently always returns     //
// 'NONE'                                                           //
//////////////////////////////////////////////////////////////////////
int evaluateColorSensor(){
    // initialize ball type with 'NONE'.  Override later if a ball color was detected
    int ballType = NONE;
    
    // Read color values from sensor:
    tcs.setInterrupt(false);      // turn on LED
    delay(100); // Takes 0.1s to turn on the LED and stablize it
    tcs.getRawData(&red, &green, &blue, &clear);
    tcs.setInterrupt(true);
    
    // Check if the ball is MAIZE
    // NOTE THAT THIS MUST BE UPDATED WITH REALISTIC VALUES!
    if ((red>8000) && (red <11000) && (green>9000) && (green <12000)&& (blue>4500) && (blue <7000) && (clear>15000)){
        ballType = MAIZE;
    }
    // Check if the ball is BLUE
    // NOTE THAT THIS MUST BE UPDATED WITH REALISTIC VALUES!
    if ((red>1000) && (red <3000) && (green>3000) && (green < 6000)&& (blue>4500) && (blue <8000) && (clear>10000)){
        ballType = BLUE;
    }
    // Check if the ball is RED
    // NOTE THAT THIS MUST BE UPDATED WITH REALISTIC VALUES!
    if ((red>2000) && (red <5000) && (green>2000) && (green <4000)&& (blue>1000) && (blue <4000) && (clear<15000)){
        ballType = RED;
    }
    // Check if the ball is WHITE
    // NOTE THAT THIS MUST BE UPDATED WITH REALISTIC VALUES!
    if ((red>10000) && (red <15000) && (green>15000) && (green <20000)&& (blue>10000) && (blue <20000) && (clear>12000) ){
        ballType = WHITE;
    }
    // Note: If none of these if-statements is true, ballType remains 'NONE'
    return ballType;
} 
// End of function evaluateColorSensor()


//////////////////////////////////////////////////////////////////////
// This is a function to update the encoder count in the Arduino.   //
// It is called via an interrupt whenever the value on encoder      //
// channel A or B changes.                                          //
//////////////////////////////////////////////////////////////////////
void updateMotorPosition() {
    // Bitwise shift left by one bit, to make room for a bit of new data:
    encoderStatus <<= 1;   
    // Use a compound bitwise OR operator (|=) to read the A channel of the encoder (pin 2)
    // and put that value into the rightmost bit of encoderStatus:
    encoderStatus |= digitalRead(2);   
    // Bitwise shift left by one bit, to make room for a bit of new data:
    encoderStatus <<= 1;
    // Use a compound bitwise OR operator  (|=) to read the B channel of the encoder (pin 3)
    // and put that value into the rightmost bit of encoderStatus:
    encoderStatus |= digitalRead(3);
    // encoderStatus is truncated to only contain the rightmost 4 bits by  using a 
    // bitwise AND operator on mstatus and 15(=1111):
    encoderStatus &= 15;
    if (encoderStatus==2 || encoderStatus==4 || encoderStatus==11 || encoderStatus==13) {
        // the encoder status matches a bit pattern that requires counting up by one
        motorPosition++;         // increase the encoder count by one
    } else {
        // the encoder status does not match a bit pattern that requires counting up by one.  
        // Since this function is only called if something has changed, we have to count downwards
        motorPosition--;         // decrease the encoder count by one
    }
}
// End of function updateMotorPosition()


//////////////////////////////////////////////////////////////////////
// This function sends a status of the controller to the serial     //
// monitor.  Each character will take 85 microseconds to send, so   //
// be selective in what you write out:                              //
//////////////////////////////////////////////////////////////////////
void printStateToSerial() {
    //*********************************************************************//
    // Send a status of the controller to the serial monitor.  
    // Each character will take 85 microseconds to send, so be selective
    // in what you write out:
  
    //Serial.print("State Number:  [CALIBRATE = 1; WAIT = 2; MOVE_TO_CHUTE = 3; WAIT_FOR_BALL = 4; PUT_BALL = 5]: ");
    Serial.print("State#: "); 
    Serial.print(state);
  
    //Serial.print("Power switch [on/off]: ");
    //Serial.print("  PWR: "); 
    //Serial.print(digitalRead(PIN_NR_ON_OFF_SWITCH));
  
    //Serial.print("      Motor Position [encoder counts]: ");
    Serial.print("  MP: "); 
    Serial.print(motorPosition);
  
    //Serial.print("      Motor Velocity [encoder counts / seconds]: ");
    Serial.print("  MV: "); 
    Serial.print(motorVelocity);
  
    //Serial.print("      Encoder Status [4 bit value]: ");
    //Serial.print("  ES: "); 
    //Serial.print(encoderStatus);
  
    //Serial.print("      Target Position [encoder counts]: ");
    Serial.print("  TP: "); 
    Serial.print(targetPosition);
  
    //Serial.print("      Position Error [encoder counts]: ");
    Serial.print("  PE: "); 
    Serial.print(positionError);
  
    //Serial.print("      Integrated Error [encoder counts * seconds]: ");
    Serial.print("  IE: "); 
    Serial.print(integralError);
  
    //Serial.print("      Velocity Error [encoder counts / seconds]: ");
    Serial.print("  VE: "); 
    Serial.print(velocityError);
  
    //Serial.print("      Desired Output Voltage [Volt]: ");
    Serial.print("  DV: "); 
    Serial.print(desiredVoltage);
    
    //Serial.print("      Motor Command [0-255]: ");
    Serial.print("  MC: "); 
    Serial.print(motorCommand);
  
    //Serial.print("      Execution Duration [microseconds]: ");
    Serial.print("  ED: "); 
    Serial.print(executionDuration);
  
    //Serial.print("      Raw signals from the color sensor: ");
    //Serial.print("  R: "); 
    //Serial.print(red);
    //Serial.print("  G: "); 
    //Serial.print(green);
    //Serial.print("  B: "); 
    //Serial.print(blue);
    //Serial.print("  C: "); 
    //Serial.print(clear);
    //Serial.print();
  
    // ALWAYS END WITH A NEWLINE.  SERIAL MONITOR WILL CRASH IF NOT
    Serial.println(); // new line
}
// End of Serial Out
