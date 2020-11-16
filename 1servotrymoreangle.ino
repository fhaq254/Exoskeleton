#include <VarSpeedServo.h>

// Servo
VarSpeedServo servo_1;
VarSpeedServo servo_2;
int servo_2_pin = 11;  // digital pin 10 (PWM)
// The direction of the rotation is actually backward
int MAX2 = 220;
int MIN2 = 80;
int turnSpeed = 10; // the turn speed, see "VarSpeedServo Details" for information
int currAng2; // holds second servo's measurement
int minPW = 500;   // 20 kg-cm Digital Servo: 500 - 2500
int maxPW = 2500;  // Analog Hobby Servo: 1500 - 1900 

// Force Sensor
int fsrPin = 0; // the FSR and ~10K pulldown are connected to a0
int fsrReading; // the analog reading from the FSR resistor divider
int graspThresh = 350; // the analog reading after which the pressure is strong enough to grasp something: the higher, the stronger

// Button
int button_pin = 2;
int current;         // Current state of the button
                     // (LOW is pressed b/c i'm using the pullup resistors)
long millis_held;    // How long the button was held (milliseconds)
long secs_held;      // How long the button was held (seconds)
long prev_secs_held; // How long the button was held in the previous check
byte previous = HIGH;
unsigned long firstTime; // how long since the button was first pressed 


// VarSpeedServo Details
  // slowmove(newpos, speed)
  // 0 = fastest speed
  // 1 = slowest speed
  // 2 - 255 = slow to fast, somewhere in between, there is usually a cap

void setup() {
  servo_2.attach(servo_2_pin, minPW, maxPW);
  pinMode(button_pin,INPUT);
  Serial.begin(9600);
  
  // Start at angle 0
  servo_2.slowmove(MIN2,turnSpeed); 
  currAng2 = servo_2.read();
  while(currAng2 < MIN2){
    Serial.println("Setting Up");
    currAng2 = servo_2.read();
  }
  Serial.println(currAng2);
}

void loop() {
  // CHECK for GRAB COMMAND
  while(!isHold()){
    // Do Nothing
    Serial.println("Resting");
  }

//  // CHECK FOR SLIGHT TOUCH
//  while(!isTouch()){
//    // Do Nothing
//    Serial.println("Waiting for Touch");
//  }
  
  // STARTING GRASP
  servo_2.slowmove(MAX2, turnSpeed);
  currAng2 = servo_2.read();
  Serial.println(currAng2);
  
  // Continue turning UNLESS we are grasping
  while(currAng2 < MAX2){
    if(isGrasp()){                             // If met with resistance, interrupt old angle with most current angle and
                                               // break out of loop
      servo_2.slowmove(currAng2,turnSpeed); 
      break;
    }
    Serial.println("Grasping");
    Serial.println(currAng2);
    currAng2 = servo_2.read();
  } // If nothing held, stop at angle defined by MAX

  // CHECK for RELEASE COMMAND
  while(!isHold()){  // Hold here until button is pressed again
    // Do Nothing
    Serial.println("Holding");
  }
  
  // RELEASING
  servo_2.slowmove(MIN2, turnSpeed);
  currAng2 = servo_2.read();
  // Continue opening
  while(currAng2 > MIN2){
    Serial.println("Opening");
    Serial.println(currAng2);
    currAng2 = servo_2.read();
  }  // Stop at angle defined by MIN
  
}


// Checks if we have grasped an item at a specified force threshold
bool isGrasp(){
  fsrReading = analogRead(fsrPin);
//  Serial.println("Force Reading:");
//  Serial.print(fsrReading);
  if(fsrReading > graspThresh){ return true;}
  return false;
}

// Checks if the force sensor is barely touching
bool isTouch(){
  fsrReading = analogRead(fsrPin);
  if(fsrReading > 100){ return true;}
  return false;
}

// Checks if button to grab is clicked
// Positive logic
bool grabRelease(){
  if(digitalRead(button_pin) == HIGH){ return true;}
  return false;
}

// Implements a hold for the button
// Positive logic
bool isHold(){
  current = digitalRead(button_pin);  // get current reading
  
  // if the button state changes to pressed, remember the start time 
  if (current == LOW && previous == HIGH && (millis() - firstTime) > 200) {
    firstTime = millis();
  }

  // get elapsed time
  millis_held = (millis() - firstTime);
  secs_held = millis_held / 1000;

  // if elapsed time is greater than 50 milliseconds, there is no mistake
  if (millis_held > 50){
    // if elapsed time is greater than 1 second, the button has been held
    if(secs_held > 1){
      Serial.println("HELD");
      return true;
    }
  }
  // Otherwise, the button has not been held for long enough
  return false;
}





