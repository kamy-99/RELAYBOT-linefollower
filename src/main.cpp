#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// Sonar pin defines
#define trig 4
#define echo 7

// Motor pin defines
#define MotorA1 8 // MotorA1 at port 8 digital
#define MotorA2 10 // MotorA2 at port 10 analog 
#define MotorB1 11 // MotorB1 at port 11 analog
#define MotorB2 12 // MotorB2 at port 12 digital  

#define Gripper 9 // gripper at 9 analog
#define RotationR1 2 // RotationR1 at port 2 digital
#define RotationR2 3 // RotationR2 at port 3 digital

#define LEDPIN 5
#define DVALUE 10 // debounce value in ms
#define SERVO_INTERVAL 20

// Communication
#define SLAVE_ID 1  // slave ID 1 is linefollower, 2 is linemaze, 3 is physicalmaze
int speed = 25; // insert real data instead of the premade dummy data!
String action = ""; // insert real data! For this one put a simple string into the movement ones where it just updates them into "forward", "left", "right", "object"
bool Finished = false;

// A1 left backward
// A2 left forward
// B1 right forward
// B2 right backward

// Sensor pin definitions
int linePins[] = {A0, A1, A2, A3, A4, A5, A6, A7};

// Arrays to store sensor values
int minlineValue[8] = {9999};
int maxlineValue[8] = {-9999};
int avglineValue[8];
int High_Threshold[8];
int Low_Threshold[8];


// Sensor states
String sensorStates[8]; // Holds the current state for each sensor
int sensorValues[8];

// Add an array to store the previous state of each sensor
String previousSensorStates[8];

// Define the weights for each sensor (based on their distance from the center)
int sensorWeights[] = {-7, -5, -3, 1, 1, 3, 5, 7};  // A0 has weight -7, A7 has weight +7


#define AVERAGE_WINDOW 1  // Number of readings to average
int recentReadings[8][AVERAGE_WINDOW];
int readingIndex = 0;

// NeoPixels
int NUMPIXELS = 4;
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, LEDPIN, NEO_RGB + NEO_KHZ800);


//drop cone
bool coneGrabbed = false;
bool endPart = false;
int finish_counter = 0;
int black_counter = 0;

// gripper
int pulse;

// PWM
int pwm;
int amount = 0;

// Rotation sensor
int RightRotation = 0;
int LeftRotation = 0;

// Sonar
float duration;
int distance;
bool flagUp = false;
bool hasExecuted = false;
bool objectDistance = false;

bool startSonar = true;

// PID variables
float Kp = 7.25;    // Increase proportional constant for quicker response 7.25 || 9 || 8.5
float Ki = 1.25;    // Small integral constant to reduce drift over time 1.25 || 9 || 7.0
float Kd = 0.75;   // Moderate derivative constant to dampen oscillations 0.5 || 0.75 || 2.0

int lastKnownSteeringError = 0;
float prevError = 0;    // Previous error value (for derivative term)
float integral = 0;     // Accumulated error (for integral term)
int steeringError = 0;


// To be able to use the functions via Platform IO
// movement functions
void stop();

// movement functions with rotation
void spin_LeftR();
void turn_RightR();
void backwardRotation();

// sonar
void flagCheck();
int sonar();
bool objectCount();
void objectAvoidance();
bool checkForLine();

// line
void Initializing();
void calibrateSensors();
void lineCalibration();
void determineStates();

// rotation
void updaterotation_R1();
void updaterotation_R2();
void reset_Rotations();

//pickup cone
void grabCone();
void forwardRotation1();
void forwardRotation2();
void spin_Left1();
void gripper(int pulse); 

// drop cone
bool Blackdetecting();
void dropCone();
void checkBlack();

// PID
int steerError();
int PID(int steeringError);
void adjustSteering(int steeringError);
void updateLineCalibration();
void returnToLine(int steeringError);

// NeoPixels
void normal_Pixel();
void braking_Pixel();
void right_Pixel();
void left_Pixel();
void back_Pixel();

// Communication
void communication(int speed, int lrotation, int rrotation, String action, int sonar, bool Finished);

void setup() 
{
 Serial.begin(9600);
 Initializing();
 pinMode(MotorA1, OUTPUT); // MotorA1 is output
 pinMode(MotorA2, OUTPUT); // MotorA2 is output
 pinMode(MotorB1, OUTPUT); // MotorB1 is output
 pinMode(MotorB2, OUTPUT); // MotorB2 is output

 pinMode(trig, OUTPUT);
 pinMode(echo, INPUT);

 pinMode(Gripper, OUTPUT);
 pinMode(RotationR1, INPUT_PULLUP);
 pinMode(RotationR2, INPUT_PULLUP);

 attachInterrupt(digitalPinToInterrupt(RotationR1), updaterotation_R1, CHANGE);
 attachInterrupt(digitalPinToInterrupt(RotationR2), updaterotation_R2, CHANGE);

  for (int i = 0; i < 10; i++) {    // gripper at 1800 open
     gripper(1800);
     delay(20);
  }
  pixels.begin();
}

void loop()
{
  if (Finished == false)
  {
  gripper(0);
  // always loop begin
  updateLineCalibration();
  calibrateSensors();
  determineStates();
  // always loop end

  int steeringError = 0; // Variable to store the calculated error for steering

  // Read all sensor values and calculate the weighted sum (steering error)
   for (int i =0; i < 8; i++)
  {
  if (sensorStates[i] == "black")
   {
    steeringError += sensorWeights[i];  // Add weighted sensor value to the total error
   }
  }

  flagCheck();
  grabCone();
  objectAvoidance();
 
  PID(steeringError);
  adjustSteering(steeringError);

  steerError();
  checkBlack();

  if (endPart == true)
  {
    stop();
  }
  }
  communication (speed, LeftRotation, RightRotation, action, distance, Finished);
}

void communication(int speed, int lrotation, int rrotation, String action, int sonar, bool Finished) {
  static unsigned long timer;
  // static unsigned long microTimer = 0;
  if (millis() > timer)
  {
  if (Serial.available()) {
    String message = Serial.readStringUntil('\n');
    if (message.length() >= 2 && message[0] == SLAVE_ID + '0' && message[1] == '?' && Finished == true) {
      // Finished should be just a true or false after putting the obj down
      Serial.print("DONE1");
    }
    timer = millis() + 2000;
    // else if (message.length() >= 2 && message[0] == SLAVE_ID + '0' && message[1] == '?') {
    //   String data = "sn:" + String(SLAVE_ID) + 
    //                 ",s:" + String(speed) + 
    //                 ",lr:" + String(lrotation) + 
    //                 ",rr:" + String(rrotation) + 
    //                 ",a:" + action +
    //                 ",so:" + String(sonar);

    //   Serial.print(data);
    // } 
  }
  }
}

int sonar()
{
  static unsigned long timer;
  // static unsigned long microTimer = 0;
  if (millis() < timer) 
  {
    return 0;
  }
  else if (millis() > timer)
  {
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    duration = pulseIn(echo, HIGH);
    distance = (duration * 0.017); // Speed of sound is approximately 343 m/s or 0.034 cm/Âµs
    timer = millis() + 250; // Reset timer for next measurement
  }
  return distance; // Return last known distance if measurement is not complete
}

bool Blackdetecting()
{
  int black_counter = 0;
   // Read all sensor values
  if (sensorStates[1] && sensorStates[2] && sensorStates[3] && sensorStates[4]&& sensorStates[5]&& sensorStates[6]&& sensorStates[7]&& sensorStates[8] == "black")
    {
      black_counter += 8;
    }
  if (black_counter == 8) 
   {
    return true;
    stop();
   }
  return false;
}

bool objectCount()
{
  static int detectionCount = 0;
  distance = sonar();
  if (distance <= 0)
  {
    return false;
  }
  if (distance > 0)
  {
    if(flagUp && hasExecuted && !objectDistance) 
    {
      if(distance > 0 && distance <= 20) // Check if object is within 20 cm
      {
        detectionCount++;
      }
      if (detectionCount > 2)
      {
        detectionCount = 0;
        return true;
      }
    }
  }
  return false;
}

void objectAvoidance()
{
  if(objectCount()) // If objectCount is true (so it is 3)
  {
    spin_LeftR();
    turn_RightR();
    turn_RightR();
    turn_RightR();
  }
}

void flagCheck()
{
  while (!flagUp) // Run continuously
  {
    distance = sonar(); // This will now update only if enough time has passed
    if (distance > 0)
    {
      
      if (!flagUp && distance > 20)
      {
        flagUp = true;
        return;
      }
    }
  }
}

void grabCone()
{
  normal_Pixel();
    if (!hasExecuted && flagUp) // Check if it hasn't executed and flagUp is true
    {
        gripper(1800);  // gripper open
        forwardRotation1(); // move forward till cone
          for (int i = 0; i < 10; i++) {  // close gripper 10 pulses per 200 mx
            gripper(950);
            delay(20);
          }
        forwardRotation2(); // move forward until over square
        spin_Left1();   // spin onto track
        stop();
        hasExecuted = true; // Mark as executed
        coneGrabbed = true; 
    }
}

void backwardRotation()
{
  reset_Rotations();

  // Start moving forward
  analogWrite(MotorA2, 0);
  digitalWrite(MotorA1, HIGH); 

  analogWrite(MotorB1, 0);
  digitalWrite(MotorB2, HIGH);
  action = "backward";  
  speed = 255;

  // Wait until both motors have completed 20 rotations
  while (RightRotation < 40 && LeftRotation < 40) 
  {
    back_Pixel();
    // Do nothing, just wait
    delay(10);  // Small delay
  }

  // Stop motors after reaching 20 rotations
  stop();
  braking_Pixel();
}

void dropCone()
{
  static unsigned long timer;
  if (millis() > timer) 
  {
  if (flagUp && hasExecuted && coneGrabbed)
  {
  for (int i = 0; i < 10; i++)  // open gripper 10 pulses per 200 ms
      { 
        gripper(1800);
        timer = millis() + 20; // update every 0.02s can change
      }
    Finished = true;
    backwardRotation();
  }
  }
}

void checkBlack()
{
  if (coneGrabbed && finish_counter >= 5)
  {
    dropCone(); // start the finish which just puts down the obj, goes back, send signal that its done
    endPart = true;
  }
}

//-----------------------------------------------------------------------------
// movement for object avoidance

// spin left 45 degrees
void spin_LeftR()
{
    reset_Rotations();

  // Start moving forward
  analogWrite(MotorA2, 0);
  digitalWrite(MotorA1, HIGH); 

  analogWrite(MotorB1, 255);
  digitalWrite(MotorB2, LOW);  
  action = "spin left";
  speed = 255;

  // Wait until both motors have completed 20 rotations
  while (RightRotation < 10 && LeftRotation < 10) 
  {
    // Do nothing, just wait
    delay(10);  // Small delay 
  }

  // Stop motors after reaching 20 rotations
  stop();
  braking_Pixel();
}

// turn right with rotations
void turn_RightR()
{
  stop();
  braking_Pixel();

 reset_Rotations();

  while (LeftRotation < 40 && RightRotation < 20) { // A is left B is right
        if (LeftRotation < 40) {
            digitalWrite(MotorA1, LOW);
            analogWrite(MotorA2, 255);
        } else {
            digitalWrite(MotorA1, LOW);
            analogWrite(MotorA2, 0);
        }
      
        if (RightRotation < 20) {
            digitalWrite(MotorB2, LOW);
            analogWrite(MotorB1, 150);
        } else {
            digitalWrite(MotorB2, LOW);
            analogWrite(MotorB1, 0);
        }
        action = "turn right";
        speed = 202;
  }
  stop();
  braking_Pixel();
}

void gripper (int pulse) // 1000 closed   2000 open angle
{
  // int pulseWidth = map(angle, 0, 180, 1000, 2000); // -10 closed 130 open
  static unsigned long timer;
  static int lastPulse;
  if (pulse == 0)
  {
    pulse = lastPulse;
  }
  else
  {
    lastPulse = pulse;
  }
    if (millis() > timer)
    {
      digitalWrite (Gripper, HIGH);
      delayMicroseconds(pulse);
      digitalWrite(Gripper, LOW);
      timer = millis() + SERVO_INTERVAL;
    }
}

//---------------------------------------------------------------------------

bool checkForLine()
{
  // Check if any sensor detects the line
  for (int i = 0; i < 8; i++)
  {
    if (sensorStates[i] == "black")
    {
      return true;
    }
  }
  return false;
}


int steerError()
{
  int steeringError = 0; // Variable to store the calculated error for steering
  int black_counter = 0;
   // Read all sensor values and calculate the weighted sum (steering error)
   for (int i = 0; i < 8; i++)
   {
    if (sensorStates[i] == "black")
    {
      steeringError += sensorWeights[i];  // Add weighted sensor value to the total error
      black_counter += 1;
    }
   }
   if (black_counter == 8) 
   {
    finish_counter += 1;
   }
   else 
   {
    finish_counter = 0;
   }
}

int PID(int steeringError)
{
  static unsigned long timer = 0;
  if (millis() > timer) // Check if 50ms have passed
  {
    // Proportional term
    float proportional = Kp * steeringError;

    // Integral term
    integral += steeringError;  // Accumulate error over time
    float integralTerm = Ki * integral;

    // Derivative term
    float derivative = steeringError - prevError;  // Rate of change of error
    float derivativeTerm = Kd * derivative;

    // Update previous error for the next loop
    prevError = steeringError;
    
    // Update the timer
    timer = millis() + 12;
    
    // Return the combined PID output
    return proportional + integralTerm + derivativeTerm;
  }
}


void adjustSteering(int steeringError) 
{
  // Calculate PID output based on the steering error
  float PID_output = PID(steeringError);

  // Base speed for motors when moving forward
  int baseSpeed = 255; // Adjust this as necessary for your robot 235

    // Update last known steering error if line is detected
  if (steeringError != 0) {
    lastKnownSteeringError = steeringError;
  }

  // Adjust movement based on the steering error
  if (steeringError <= 4 && steeringError >= -4 && steeringError != 0) // 7, -7 
  {
    // Robot is aligned, move straight
    analogWrite(MotorA2, 255); // Left motor forward
    digitalWrite(MotorA1, LOW);      // Ensure left motor doesn't go backward
    analogWrite(MotorB1, 255); // Right motor forward
    digitalWrite(MotorB2, LOW);      // Ensure right motor doesn't go backward
    action = "straight";
    speed = 255;
  }
  else if (steeringError <= -6) // -11
  {
    // Robot needs to turn left
    int leftSpeed = baseSpeed - PID_output;  // Reduce speed for left motor
    int rightSpeed = baseSpeed + PID_output; // Increase speed for right motor

    // Constrain speeds to valid PWM range
    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);

    // Apply motor speeds
    analogWrite(MotorA2, leftSpeed); // Left motor forward
    digitalWrite(MotorA1, LOW);      // Ensure left motor doesn't go backward
    analogWrite(MotorB1, rightSpeed); // Right motor forward
    digitalWrite(MotorB2, LOW);       // Ensure right motor doesn't go backward
    action = "left";
    speed = (leftSpeed + rightSpeed) / 2; 
  }
  else if (steeringError >= 6) // 11
  {
    // Robot needs to turn right
    int leftSpeed = baseSpeed + PID_output;  // Increase speed for left motor
    int rightSpeed = baseSpeed - PID_output; // Reduce speed for right motor

    // Constrain speeds to valid PWM range
    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);

    // Apply motor speeds
    analogWrite(MotorA2, leftSpeed); // Left motor forward
    digitalWrite(MotorA1, LOW);      // Ensure left motor doesn't go backward
    analogWrite(MotorB1, rightSpeed); // Right motor forward
    digitalWrite(MotorB2, LOW);       // Ensure right motor doesn't go backward
    action = "right";
    speed = (leftSpeed + rightSpeed) / 2;
  }
  // Optional: Debugging output
  Serial.print("Steering Error: ");
  Serial.print(steeringError);
  Serial.print(" | PID Output: ");
  Serial.print(PID_output);
}

void returnToLine(int steeringError) {
  int baseSpeed = 255; // Lower speed for more precise movement
  unsigned long timer = 0;
  bool lineFound = false;

  // Turn in the direction of the last known error for 100 ms
  if (millis() > timer && !lineFound) {
    if (lastKnownSteeringError < 0) {
      // Turn left
      analogWrite(MotorA2, baseSpeed - 100 );
      digitalWrite(MotorA1, HIGH);
      analogWrite(MotorB1, baseSpeed);
      digitalWrite(MotorB2, LOW);
      action = "left";
      speed = ((255 - (baseSpeed - 100)) + baseSpeed) / 2;
    } else {
      // Turn right (including when lastKnownSteeringError == 0)
      analogWrite(MotorA2, baseSpeed);
      digitalWrite(MotorA1, LOW);
      analogWrite(MotorB1, baseSpeed - 100);
      digitalWrite(MotorB2, HIGH);
      action = "right";
      speed = ((255 - (baseSpeed - 100)) + baseSpeed) / 2;
    }
    timer = millis() + 100;

    // Check if line is detected
    int currentError = steeringError; // Implement this function to calculate current error
    if (currentError != 0) {
      lineFound = true;
      return;
    }
  }
}

// ----------------------------------------------------------------

// move forward till square
void forwardRotation1()
{
  reset_Rotations();

  // Start moving forward
  analogWrite(MotorA2, 255);
  digitalWrite(MotorA1, LOW); 

  analogWrite(MotorB1, 255);
  digitalWrite(MotorB2, LOW);
  action = "forward";  
  speed = 255;

  // Wait until both motors have completed 20 rotations
  while (RightRotation < 35 && LeftRotation < 35) 
  {
    normal_Pixel();
    // Do nothing, just wait
    delay(10);  // Small delay 
  }

  // Stop motors after reaching 20 rotations
  stop();
  braking_Pixel();
}

// move forward over square
void forwardRotation2()
{
  reset_Rotations();
  
  // Start moving forward
  analogWrite(MotorA2, 255);
  digitalWrite(MotorA1, LOW); 

  analogWrite(MotorB1, 255);
  digitalWrite(MotorB2, LOW); 
  action = "forward";  
  speed = 255; 

  // Wait until both motors have completed 20 rotations
  while (RightRotation < 10 && LeftRotation < 10) 
  {
    normal_Pixel();
    // Do nothing, just wait
    delay(10);  // Small delay to prevent CPU hogging
  }

  // Stop motors after reaching 20 rotations
  stop();
  braking_Pixel();
}

void spin_Left1() // spin left on its axle in a 90 degree angle
{
    reset_Rotations();

  // Start moving forward
  analogWrite(MotorA2, 0);
  digitalWrite(MotorA1, HIGH); 

  analogWrite(MotorB1, 255);
  digitalWrite(MotorB2, LOW);  

  // Wait until both motors have completed 20 rotations
  while (RightRotation < 20 && LeftRotation < 20) 
  {
    left_Pixel();
    // Do nothing, just wait
    delay(10);  // Small delay
  }

  // Stop motors after reaching 20 rotations
  stop();
  braking_Pixel();
}

//------------------------------------------------------------------

void Initializing()
{
  for (int i = 0; i < 8; i++) {
    pinMode(linePins[i], INPUT);
    sensorStates[i] = "white"; // Initial state for all sensors

    // Initialize recent readings
    for (int j = 0; j < AVERAGE_WINDOW; j++) {
      recentReadings[i][j] = 0;
    }
  }
}

void updateLineCalibration() {
  static unsigned long timer = 0;
  if (millis() - timer > 1000) {
    lineCalibration();
  }
  timer = millis(); // update every 0.25s can change
}


void calibrateSensors() {

  for (int i = 0; i < 10; i++) {  // Adjust the number of calibration iterations as needed
    for (int j = 0; j < 8; j++) {
      int sensorValue = analogRead(linePins[j]);

      // Update minimum and maximum values
      if (sensorValue < minlineValue[j]) {
        minlineValue[j] = sensorValue;
      }
      if (sensorValue > maxlineValue[j]) {
        maxlineValue[j] = sensorValue;
      }
      sensorValues[j] = sensorValue;
    }
  }
   // Calculate static thresholds for each sensor
  for (int i = 0; i < 8; i++) {
  High_Threshold[i] = 800; 
  Low_Threshold[i] =  High_Threshold[i] - 50; // Adjusted white threshold (higher range)
  }

}

void lineCalibration() {
  for (int i = 0; i < 8; i++) {
    int sensorValue = analogRead(linePins[i]);

    // Update recent readings
    recentReadings[i][readingIndex] = sensorValue;

    // Calculate moving average
    long sum = 0;
    for (int j = 0; j < AVERAGE_WINDOW; j++) {
      sum += recentReadings[i][j];
    }
    avglineValue[i] = sum;
  }

  // Update reading index
  readingIndex = (readingIndex + 1) % AVERAGE_WINDOW;

  //delay(200); // Delay for stability
}

void determineStates() {
  for (int i = 0; i < 8; i++) {
    if (sensorValues[i] >= High_Threshold[i]) {
      sensorStates[i] = "black";
    } else if (sensorValues[i] <= Low_Threshold[i]) {
      sensorStates[i] = "white";
    } else {
      // In the gap between thresholds, maintain the previous state
      sensorStates[i] = previousSensorStates[i];
    }

    // Update the previous state after determining the current state
    previousSensorStates[i] = sensorStates[i];
  }
}

void updaterotation_R1() // right rotation
{
  static unsigned long timer;
  static bool lastState;
  noInterrupts();
  if (millis() > timer) {
    bool state = digitalRead(RotationR1);
    if (lastState != state) {
      RightRotation++;
      lastState = state;
    }
  timer = millis() + DVALUE;
  }
  interrupts();
}

void updaterotation_R2() // left rotation
{
  static unsigned long timer;
  static bool lastState;
  noInterrupts();
  if (millis() > timer) {
    bool state = digitalRead(RotationR2);
    if (lastState != state) {
      LeftRotation++;
      lastState = state;
    }
  timer = millis() + DVALUE;
  }
  interrupts();
}

void reset_Rotations() 
{
  noInterrupts();
  RightRotation = 0;
  LeftRotation = 0;
  interrupts();
}

//-----------------------------------------------------------
// LED neopixels

void normal_Pixel()
{
  //pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(75,0,0)); // red at 60
  pixels.setPixelColor(1, pixels.Color(75,0,0)); // red at 60
  pixels.setPixelColor(2, pixels.Color(255, 255, 75)); // white ish headlights 100
  pixels.setPixelColor(3, pixels.Color(255, 255, 75)); // white ish headlights 100
  pixels.show();
}

void braking_Pixel()
{
  //pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(255,0,0)); // red at 100
  pixels.setPixelColor(1, pixels.Color(255,0,0)); // red at 100
  pixels.setPixelColor(2, pixels.Color(255, 255, 75)); // white ish headlights 100
  pixels.setPixelColor(3, pixels.Color(255, 255, 75)); // white ish headlights 100
  pixels.show();
}

void right_Pixel()
{
    static bool lightOn = false;
    static unsigned long timer = millis();

    if ((millis() - timer) > 500)
    {
      if(lightOn == true)
      {
        pixels.clear();
        pixels.setPixelColor(0, pixels.Color(75, 0, 0));      // Red at 60 left back
        pixels.setPixelColor(1, pixels.Color(255, 100, 0));   // Orange right
        pixels.setPixelColor(2, pixels.Color(255, 100, 0));   // Orange right
        pixels.setPixelColor(3, pixels.Color(255, 255, 75));  // White-ish headlights left front
        pixels.show();
      }
      else
      {
        pixels.clear();
        pixels.setPixelColor(0, pixels.Color(75, 0, 0));      // Red at 60 left back
        pixels.setPixelColor(1, pixels.Color(0,0,0));         // off right
        pixels.setPixelColor(2, pixels.Color(0,0,0));         // off right
        pixels.setPixelColor(3, pixels.Color(255, 255, 75));  // White-ish headlights left front
        pixels.show(); // Turn off the orange pixels 
      }
      lightOn = !lightOn;
      timer = millis();
    }
}


void left_Pixel()
{
    static bool lightOn = false;
    static unsigned long timer = millis();

    if ((millis() - timer) > 500)
    {
      if(lightOn == true)
      {
        pixels.clear();
        pixels.setPixelColor(0, pixels.Color(255, 100, 0));   // Orange left back
        pixels.setPixelColor(1, pixels.Color(75, 0, 0));      // Red at 60 right back
        pixels.setPixelColor(2, pixels.Color(255, 255, 75));  // White-ish headlights right front
        pixels.setPixelColor(3, pixels.Color(255, 100, 0));   // Orange left front
        pixels.show();
      }
      else
      {
        pixels.clear();
        pixels.setPixelColor(1, pixels.Color(75, 0, 0));      // Red at 60 right back
        pixels.setPixelColor(0, pixels.Color(0,0,0));         // off left back
        pixels.setPixelColor(3, pixels.Color(0,0,0));         // off left front
        pixels.setPixelColor(2, pixels.Color(255, 255, 75));  // White-ish headlights right front
        pixels.show(); // Turn off the orange pixels
      }
      lightOn = !lightOn;
      timer = millis();
    }
  }

void back_Pixel()
{
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(255,0,0)); // red at 100
  pixels.setPixelColor(1, pixels.Color(255,255,255)); // red at 100
  pixels.setPixelColor(2, pixels.Color(255, 255, 75)); // white ish headlights 100
  pixels.setPixelColor(3, pixels.Color(255, 255, 75)); // white ish headlights 100
  pixels.show();
}


//------------------------------------------------------
// basic movement

void stop() // stop all motors
{
  analogWrite(MotorB1, 0);  // right forward
  digitalWrite(MotorA1, LOW);// left bacward
  analogWrite(MotorA2, 0);   // left forward
  digitalWrite(MotorB2, LOW); // right backward
  braking_Pixel();
  action = "stopped";  
  speed = 0;
}


 


