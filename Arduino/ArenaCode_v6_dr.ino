/* Version 6 by David Remy 11/25/16

This is the code used for the playing field.  It uses either a fixed sequence of 'N_DROPS' balls (determined
in the variable 'dropSequence') or uses a random drop sequence (if 'RANDOM_DROP' is true).  The program also 
incorporates a timer that counts down from 'RUN_TIME' to 0.

Once the timer has been started (indicated by the Arduino LED), the program cycles through three states:
DETERMINE_CHUTE: In this state, one chute is selected and activated.  READY_TO_DROP is next.               
READY_TO_DROP:   Waiting for the drop request signal to go HIGH.  Then drop the ball, reload the chute and WAIT_FOR_RESET
WAIT_FOR_RESET:  Waiting for the drop request signal to go LOW.  Then DETERMINE_CHUTE

If all balls have been released, it goes in the waiting state OUT_OF_BALLS, and if the timer has run
out, it goes into the terminal state OUT_OF_TIME (which also turns off the LED).
*/

// Give the chutes a name:
const int L = 0;
const int R = 1;

/**********************
 * MAKE CHANGES HERE: *
 **********************/
// Define the playing parameters:
const int N_DROPS = 30;       // [.] Total number of drops
const long RUN_TIME = 120;    // [s] Time given to play the game
const int RANDOM_DROP = true; // [bool] If this variable is true, the manual drop sequence will be overwritten by a randomized one. 
// This is the fixed drop sequence (unless overwritten by randomization)
int dropSequence[N_DROPS] = {L,L,R,R,L,L,R,R,L,L,R,R,L,L,R,R,L,L,R,R,L,L,R,R,L,L,R,R,L,L};  


// State machine:
int state;  // Variable containing the state
// States of this program:
const int DETERMINE_CHUTE = 1; // In this state, one chute is selected and activated.  READY_TO_DROP is next.               
const int READY_TO_DROP = 2;   // Waiting for the drop request signal to go HIGH.  Then drop the ball, reload the chute and WAIT_FOR_RESET
const int WAIT_FOR_RESET = 3;  // Waiting for the drop request signal to go LOW.  Then DETERMINE_CHUTE
const int OUT_OF_TIME = 4;     // Clock ran down.  This state will never exit
const int OUT_OF_BALLS = 5;    // All balls have been dropped.  This state will only exit if time is running out.

// Variable controlling the sequence:
int ballCounter; // [0..nDROP-1]  Number of current ball
int nextDrop;    // [L/R] The currently active chute

// timing
long startTime;       // [ms] When the timer was started
long nextDisplayTime; // [ms] Determines the next time we print out the remaining time to the serial port

// Definition of hardware interface:
// Timing of ball dropping:
const int FALL_TIME  = 50;         // [ms] time for a ball to fall (open close of solenoid)
const int CLOSE_TIME = 200;        // [ms] time to close the chute (before opening the top solenoid)
// Pin assignment:
const int LEFT_CHUTE_ACTIVE = 8;   // HIGH if left chute is ready to drop a ball
const int RIGHT_CHUTE_ACTIVE = 2;  // HIGH if right chute is ready to drop a ball
const int DROP_REQUEST = 3;        // HIGH if the mechanism sends a drop request
const int RIGHT_DROP_SOLENOID = 4; // HIGH to trigger the bottom solenoid of the right chute (drop ball)
const int RIGHT_LOAD_SOLENOID = 5; // HIGH to trigger the top solenoid of the right chute (load ball)
const int LEFT_DROP_SOLENOID = 6;  // HIGH to trigger the bottom solenoid of the left chute (drop ball)
const int LEFT_LOAD_SOLENOID = 7;  // HIGH to trigger the top solenoid of the left chute (load ball)


void setup() {
  // Activate pins for input and output:
  pinMode(LEFT_CHUTE_ACTIVE, OUTPUT);
  pinMode(RIGHT_CHUTE_ACTIVE, OUTPUT);
  pinMode(DROP_REQUEST, INPUT);
  pinMode(LEFT_DROP_SOLENOID, OUTPUT);
  pinMode(LEFT_LOAD_SOLENOID, OUTPUT);
  pinMode(RIGHT_DROP_SOLENOID, OUTPUT);
  pinMode(RIGHT_LOAD_SOLENOID, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // Turn off the chute ready signals:
  digitalWrite(LEFT_CHUTE_ACTIVE, LOW);
  digitalWrite(RIGHT_CHUTE_ACTIVE, LOW);
  // Turn off the solenoids:
  digitalWrite(LEFT_DROP_SOLENOID, LOW);
  digitalWrite(LEFT_LOAD_SOLENOID, LOW);
  digitalWrite(RIGHT_DROP_SOLENOID, LOW);
  digitalWrite(RIGHT_LOAD_SOLENOID, LOW);

  // Begin serial communication:
  Serial.begin(115200);
  Serial.println();
  Serial.println();
  Serial.println("Welcome: Start executing program.");
  Serial.println();

  // Randomize drop sequence if requested
  if (RANDOM_DROP) {
    Serial.println("Randomizing drop sequence.");
    Serial.println();
    randomizeDropSequenceFcn();
  }

  // Print out the drop sequence:
  printDropSequenceFcn();

  // Making sure the chutes are loaded:
  Serial.println("Loading both chutes.");
  Serial.println();
  digitalWrite(LEFT_LOAD_SOLENOID, HIGH);  // open the top solenoid to load next ball
  digitalWrite(RIGHT_LOAD_SOLENOID, HIGH); // open the top solenoid to load next ball
  delay(FALL_TIME);                        // delay for the balls to fall
  digitalWrite(LEFT_LOAD_SOLENOID, LOW);   // close the top solenoid to finish drop sequence
  digitalWrite(RIGHT_LOAD_SOLENOID, LOW);  // close the top solenoid to finish drop sequence

  // Perform a 5 second count down:
  performCountdownFct(5);
 
  // Start timer:
  Serial.println("********************************");
  Serial.println("Starting timer.");
  Serial.println("********************************");
  Serial.println();
  digitalWrite(LED_BUILTIN , HIGH); // Turn on the LED on the Arduino
  
  startTime = millis();
  delay(1);
  nextDisplayTime = nextDisplayTimeFct();

  // get ready for dropping balls
  Serial.println("********************************");
  Serial.println("Start dropping balls.");
  Serial.println("********************************");
  Serial.println();
  ballCounter = 0;
  state =  DETERMINE_CHUTE;
}


void loop() {
  //******************************************************************************//
  // The state machine:
  switch (state) {
    case DETERMINE_CHUTE:
      if (ballCounter==N_DROPS) {
        state = OUT_OF_BALLS;  
        Serial.println();
        Serial.println("********************************");
        Serial.println("CONGRATULATIONS!! YOU ARE DONE!!");
        Serial.println("********************************");
        break;
      }
      Serial.print("Getting ball # ");
      Serial.print(ballCounter + 1);
      nextDrop = dropSequence[ballCounter];
      ballCounter++;   
      if (nextDrop==L) {
        Serial.println(" ready.");
         Serial.println(" - Left chute activated.  Waiting for drop signal to go HIGH.");
        digitalWrite(LEFT_CHUTE_ACTIVE, HIGH);
      } else {
        Serial.println(" ready.");  
         Serial.println(" - Right chute activated.  Waiting for drop signal to go HIGH.");
        digitalWrite(RIGHT_CHUTE_ACTIVE, HIGH);
      }
      // immediately transit
      state = READY_TO_DROP;
      break;
    case READY_TO_DROP:
      // Just wait, do nothing
      if (digitalRead(DROP_REQUEST)==HIGH) {
        Serial.println(" - Drop signal recieved");
        dropBallFct(nextDrop);
        // Reset active signal
        digitalWrite(LEFT_CHUTE_ACTIVE, LOW);
        digitalWrite(RIGHT_CHUTE_ACTIVE, LOW);
        Serial.println(" - Ball has been dropped.  Waiting for drop signal to go LOW.");
        state = WAIT_FOR_RESET;
      }
    break;
    case WAIT_FOR_RESET:
      // Just wait, do nothing
      if (digitalRead(DROP_REQUEST)==LOW) {
        Serial.println(" - Drop signal was reset.  Activate next ball.");
        state = DETERMINE_CHUTE;
      }
    break;
    case OUT_OF_BALLS:
      // Do nothing.  We'll remain here until time runs out.
    break;
    case OUT_OF_TIME:
      // Do nothing.  This is nirvana. Nothing leads out of this state.
    break;
  }
  //******************************************************************************//
  
  // Timing
  if (state!= OUT_OF_TIME && timeLeftToPlayFct()<0) {
    state = OUT_OF_TIME;
    digitalWrite(LED_BUILTIN , LOW); // Turn off the LED on the Arduino
    Serial.println();
    Serial.println("********************************");
    Serial.println("SORRY. YOU ARE OUT OF TIME!!!");
    Serial.println("********************************");
  }
  // Display
  if (state!= OUT_OF_TIME && timeLeftToPlayFct()<nextDisplayTime) {
    Serial.print("TIMER: ");
    Serial.print(nextDisplayTime/1000);
    Serial.println(" seconds to go.");
    nextDisplayTime = nextDisplayTimeFct();
  }
}

// Drops a ball from either the left (chute==L) or right (chute==R) chute
void dropBallFct(int chute) {
  if(chute==L) {
      digitalWrite(LEFT_DROP_SOLENOID,HIGH); // the bottom solenoid is opened
      delay(FALL_TIME);                      // delay for the first ball to fall
      digitalWrite(LEFT_DROP_SOLENOID,LOW);  // close the bottom solenoid
      delay(CLOSE_TIME);                     // delay for the chute to close again
      digitalWrite(LEFT_LOAD_SOLENOID,HIGH); // open the top solenoid to load next ball
      delay(FALL_TIME);                      // delay for the balls to fall
      digitalWrite(LEFT_LOAD_SOLENOID,LOW);  // close the top solenoid to finish drop sequence
  } else {
      digitalWrite(RIGHT_DROP_SOLENOID,HIGH); // the bottom solenoid is opened
      delay(FALL_TIME);                       // delay for the first ball to fall
      digitalWrite(RIGHT_DROP_SOLENOID,LOW);  // close the bottom solenoid
      delay(CLOSE_TIME);                      // delay for the chute to close again
      digitalWrite(RIGHT_LOAD_SOLENOID,HIGH); // open the top solenoid to load next ball
      delay(FALL_TIME);                       // delay for the balls to fall
      digitalWrite(RIGHT_LOAD_SOLENOID,LOW);  // close the top solenoid to finish drop sequence
  }
}


// Thus function reates a random drop sequence.
// This could be improved to ensure that there are equally many drops left and right
void randomizeDropSequenceFcn() {
  randomSeed(analogRead(0));
  // Set everything to 'L'
  for (int i=0; i<N_DROPS; i++) {
    dropSequence[i] = L;
  }
  // Now set N_DROPS/2 to 'R'
  for (int i=0; i<N_DROPS/2; i++) {
    int change = random(N_DROPS);
    while (dropSequence[change] == R) {
      // Repeat random process if this drop has already been set to R
      change = random(N_DROPS);
    }
    dropSequence[change] = R;
  }
}


// Count down n seconds:
void performCountdownFct(int n) {
  Serial.print("Countdown: ");
  for (int i=n; i>0; i--) {
    Serial.print(i);
    delay(500);
    Serial.print(".");
    delay(500);
  }
  Serial.println("0");
}


// Print out the drop sequence:
void printDropSequenceFcn() {
  Serial.println("Here is today's drop sequence:");
  for (int i=0; i<N_DROPS; i++) {
    Serial.print("Drop #: ");
    if (i+1<10) {
      Serial.print(" ");
    }  
    Serial.print(i+1);
    if (dropSequence[i]==L) {
      Serial.print(" | L |   |");
    } else{ 
      Serial.print(" |   | R |");
    }
    Serial.println();
  }
  Serial.println();
}


// Returns the time left in this game (in milliseconds)
long timeLeftToPlayFct() {
  long timeLeft = RUN_TIME*1000 + startTime - millis();
  return timeLeft;
}


// Returns the next instance the timer should do something (in milliseconds)
// Count down increments of 10
// Count down the last 10 seconds individually
long nextDisplayTimeFct() {
  // Obtain time left on the clock
  long NextTime = timeLeftToPlayFct();
  // Then round it to the next suitable integer  
  NextTime = NextTime/1000;  // Convert to seconds
  if (NextTime >10) {
    // round to 10 second intervals:
    NextTime = NextTime/10;
    NextTime = NextTime*10;
  }
  NextTime = NextTime*1000;
  return NextTime;
}
