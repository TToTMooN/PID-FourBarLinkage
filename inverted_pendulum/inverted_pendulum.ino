#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>

#include <Stepper.h>

/* Assign a unique ID to the sensors */
Adafruit_9DOF                dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);


const int stepsPerRevolution = 200;  // change this to fit the number of steps per revolution
// for your motor




// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 4, 7, 8, 12);

int stepCount = 0;  // number of steps the motor has taken
int nextCount = 0;
int motorSpeed = 150;
long executionDuration = 0;
long lastExecutionTime = 0;
const int motorRange = 1000;

float initialAngle = 0;
float angleDifference = 0;

void initSensors()
{
  if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
}

void setup() {
    Serial.begin(115200);
    initSensors();
    myStepper.setSpeed(motorSpeed);
    myStepper.step(motorRange/2);
    stepCount = motorRange/2;
    delay(1000);
    sensors_event_t mag_event;
    sensors_vec_t   orientation;
    mag.getEvent(&mag_event);
    dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation);
    
    initialAngle = orientation.heading;
}

void loop() {
    executionDuration = micros() - lastExecutionTime;
    lastExecutionTime = micros();
    
    //calculating dynamic parameters
    //sensors_event_t accel_event;
    sensors_event_t mag_event;
    sensors_vec_t   orientation;
    mag.getEvent(&mag_event);
    dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation);
    
    angleDifference = orientation.heading - initialAngle;
    Serial.print("heading:");
    Serial.print(orientation.heading);
    Serial.print("  AD:");
    Serial.println(angleDifference);
    
    
    //calculating the motor speed
    //motorSpeed = 
    
    
    
    
    //drive the motor
    if (!(motorSpeed == 0)){
        nextCount = stepCount + motorSpeed / abs(motorSpeed);
    }
    if (!(motorSpeed == 0) && nextCount >= 0 && nextCount <= motorRange) {
        myStepper.setSpeed(motorSpeed);
        myStepper.step(1);
        stepCount = stepCount + motorSpeed / abs(motorSpeed);
    }
}
