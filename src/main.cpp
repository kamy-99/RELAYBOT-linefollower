#include <Arduino.h>

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
#define DVALUE 10 // debounce value in ms
#define PIVALUE 3.141592653589793238462643383279502884197 // 39 digits or so

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
float sensorWeights[] = {-5, -4, -3, 1, 1, 3, 4, 5};  // A0 has weight -3, A7 has weight +3


#define AVERAGE_WINDOW 1  // Number of readings to average
int recentReadings[8][AVERAGE_WINDOW];
int readingIndex = 0;

// PWM
int pwm;
int amount = 0;

// Rotation sensor
int RightRotation = 0;
int LeftRotation = 0;
int angle = 50;

// Sonar
long duration = pulseIn(echo, HIGH);
int distance = (duration * 0.017); // Distance in cm
bool flagUp = false;

// PID variables
float Kp = 6.0;   // Increase proportional constant for quicker response
float Ki = 0.7;   // Small integral constant to reduce drift over time
float Kd = 1.1;   // Moderate derivative constant to dampen oscillations

float prevError = 0;    // Previous error value (for derivative term)
float integral = 0;     // Accumulated error (for integral term)


// To be able to use the functions via Platform IO
void forward();
void backward();
void stop();
void spin_Right();
void spin_Left();
void turn_Right();
void turn_Left();
void sonar();
void Initializing();
void calibrateSensors();
void lineCalibration();
void determineStates();
void printSensorStates();
void updaterotation_R1();
void updaterotation_R2();
void reset_Rotations();
void steerError();
int PID(int steeringError);
void adjustSteering(int steeringError);
void updateLineCalibration();

void setup() {
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
}
 
void loop()
{
  updaterotation_R2();
  updaterotation_R1();
  updateLineCalibration();
  calibrateSensors();
  determineStates();
  int steeringError = 0; // Variable to store the calculated error for steering

   // Read all sensor values and calculate the weighted sum (steering error)
   for (int i =0; i < 8; i++)
   {
    if (sensorStates[i] == "black")
    {
      steeringError += sensorWeights[i];  // Add weighted sensor value to the total error
    }
   }
  
  PID(steeringError);

  adjustSteering(steeringError); 
  printSensorStates();
  // Serial.println("after loop");
  // if (flagUp == true)
  // {
  //   turn_Left();
  //   spin_Right();
  //   turn_Right();
  //   spin_Left();
  //   stop();
  // }
  //sonar();
}

void flagCheck()
{
  if(distance > 20)
  {
    flagUp = true;
  }
}

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
  // Serial.println("Calibrating sensors...");

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
  High_Threshold[i] = 800; //(maxlineValue[i] + minlineValue[i]) / 2 + 100; // Black threshold
  Low_Threshold[i] =  High_Threshold[i] - 50;//( maxlineValue[i] + minlineValue[i]) / 2 - 100; // Adjusted white threshold (higher range)
  }
  // Serial.println("Calibration complete!");
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


void steerError()
{
  int steeringError = 0; // Variable to store the calculated error for steering

   // Read all sensor values and calculate the weighted sum (steering error)
   for (int i =0; i < 8; i++)
   {
    if (sensorStates[i] == "black")
    {
      steeringError += sensorWeights[i];  // Add weighted sensor value to the total error
    }
   }
}

int PID(int steeringError)
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

  // Return the combined PID output
  return proportional + integralTerm + derivativeTerm;
}


void adjustSteering(int steeringError) 
{
  // Calculate PID output based on the steering error
  float PID_output = PID(steeringError);

  // Base speed for motors when moving forward
  int baseSpeed = 200; // Adjust this as necessary for your robot

  // Adjust movement based on the steering error
  if (steeringError <= 4 && steeringError >= -4 && steeringError != 0)
  {
    // Robot is aligned, move straight
    analogWrite(MotorA2, 255); // Left motor forward
    digitalWrite(MotorA1, LOW);      // Ensure left motor doesn't go backward
    analogWrite(MotorB1, 243); // Right motor forward
    digitalWrite(MotorB2, LOW);      // Ensure right motor doesn't go backward
  }
  else if (steeringError > 6)
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
  }
  else if (steeringError == 0)
  {
    // Apply motor speeds
    analogWrite(MotorA2, 0); 
    digitalWrite(MotorA1, LOW);      
    analogWrite(MotorB1, 0); 
    digitalWrite(MotorB2, LOW);       
  }
  else if (steeringError < -6)
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
  }

  // Optional: Debugging output
  Serial.print("Steering Error: ");
  Serial.print(steeringError);
  Serial.print(" | PID Output: ");
  Serial.print(PID_output);
  // Serial.print(" | Left Speed: ");
  // Serial.print((steeringError >= 0) ? baseSpeed + PID_output : baseSpeed - PID_output);
  // Serial.print(" | Right Speed: ");
  // Serial.println((steeringError >= 0) ? baseSpeed - PID_output : baseSpeed + PID_output);
}

 
void sonar() // printing distance
{
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
 
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
 
  delay(250);
 
  if (distance <= 40)
  {
    forward();
    if (distance <= 35)
    {
      forward();
      if (distance <= 30)
      {
        spin_Left();
        turn_Right();
        turn_Right();
        spin_Left();
        stop();
      }
    }
  }
  else
  {
    forward();
  }
  return;
}
 
void forward()
{
  reset_Rotations();
  analogWrite(MotorA2, 250); // Left motor runs at full speed
  digitalWrite(MotorA1, LOW);
  digitalWrite(MotorB2, LOW);
 
  for(int pwm = 0; pwm < 256; pwm++)
  {
    analogWrite(MotorB1, pwm + 12); // Right motor with PWM control, slightly reduced
    delay(200);
  }
  delay(5000);
}
 
void backward()
{
  reset_Rotations();
  // analogWrite(MotorB1,)
  analogWrite(MotorA2, 0);
  digitalWrite(MotorB2, HIGH);
  digitalWrite(MotorA1, HIGH); // Left motor runs at full speed
 
  for(int pwm = 255; pwm > 0; pwm++)
  {
    analogWrite(MotorB1, pwm - 17); // Right motor with PWM control, slightly reduced
    delay(200);
  }
  delay(5000);
}
 
void stop() // stop all motors
{
  analogWrite(MotorB1, 0);  // right forward
  digitalWrite(MotorA1, LOW);// left bacward
  analogWrite(MotorA2, 0);   // left forward
  digitalWrite(MotorB2, LOW); // right backward
}

void spin_Right() // spin right on its axle in a 90 degree angle
{
  reset_Rotations();
  analogWrite(MotorB1, 62);  // right forward
  digitalWrite(MotorA1, LOW);// left bacward
  analogWrite(MotorA2, 255);   // left forward
  digitalWrite(MotorB2, HIGH); // right backward
  delay(510); // 455
}
 
void spin_Left() // spin left on its axle in a 90 degree angle
{
  reset_Rotations();  
   analogWrite(MotorB1, 255);  // right forward
  digitalWrite(MotorA1, HIGH);// left bacward
  analogWrite(MotorA2, 22);   // left forward
  digitalWrite(MotorB2, LOW); // right backward
  delay(550); //495
}
 
void turn_Left() // turn left with a 90 degree angle
{
  reset_Rotations();
  analogWrite(MotorA2, 255);
  analogWrite(MotorB1, 255);
  delay(100);
  digitalWrite(MotorA1, LOW);// left bacward
  analogWrite(MotorA2, 140); // left forward
  analogWrite(MotorB1, 255); // right forward
  digitalWrite(MotorB2, LOW); // right backward
  delay(1800);
}
 
void turn_Right() // turn right with a 90 degree angle
{
  reset_Rotations();
  analogWrite(MotorA2, 255);
  analogWrite(MotorB1, 255);
  delay(100);
  digitalWrite(MotorA1, LOW);// left bacward
  analogWrite(MotorA2, 255); // left forward
  analogWrite(MotorB1, 140); // right forward
  digitalWrite(MotorB2, LOW); // right backward
  delay(2625);
}



// void spin_Left() 
// { // should be done
//   reset_Rotations();  
//   analogWrite(MotorB1, 255);  // right forward
//   digitalWrite(MotorA1, HIGH);// left bacward
//   analogWrite(MotorA2, 22);   // left forward
//   digitalWrite(MotorB2, LOW); // right backward
// }

// void spin_Right() // spin right on its axle in a 90 degree angle
// {
//   reset_Rotations();
//   analogWrite(MotorB1, 62);  // right forward
//   digitalWrite(MotorA1, LOW);// left bacward
//   analogWrite(MotorA2, 255);   // left forward
//   digitalWrite(MotorB2, HIGH); // right backward
// }

// void forward()
// {
//   reset_Rotations();
//   analogWrite(MotorB1, 243);  // right forward
//   digitalWrite(MotorA1, LOW);// left bacward
//   analogWrite(MotorA2, 255);   // left forward
//   digitalWrite(MotorB2, LOW); // right backward
//   //old right 243
//   // old left 255
// }

// void backward()
// {
//   reset_Rotations();
//   analogWrite(MotorB1, 0);  // right forward
//   digitalWrite(MotorA1, HIGH);// left bacward
//   analogWrite(MotorA2, 0);   // left forward
//   digitalWrite(MotorB2, HIGH); // right backward
//   // old right 238
//   // old left 250
// }

// void turn_Left() // turn left with a 90 degree angle
// {
//   reset_Rotations();
//   analogWrite(MotorB1, 255);  // right forward
//   digitalWrite(MotorA1, LOW);// left bacward
//   analogWrite(MotorA2, 140);   // left forward
//   digitalWrite(MotorB2, LOW); // right backward
//   // old left 140
//   //old right 255
// }

// void turn_Right()  // turn right with a 90 degree angle
// { 
//   reset_Rotations();
//   analogWrite(MotorB1, 140);  // right forward
//   digitalWrite(MotorA1, LOW);// left bacward
//   analogWrite(MotorA2, 255);   // left forward
//   digitalWrite(MotorB2, LOW); // right backward
//   // old right 140
//   // old left 255
// }


// void gripper(int angle) 
// {
//   // Map the angle to the pulse width (1000 to 2000 microseconds)
//   int pulseWidth = map(angle, 0, 180, 1000, 2000);
//   // Send the PWM signal
//   for (int i = 0; i < 50; i++) { // Repeat to maintain the pulse for ~1 second
//     analogWrite(Gripper, 255); // Set the pin high
//     delayMicroseconds(pulseWidth); // Wait for the pulse width duration
//     analogWrite(Gripper, 0); // Set the pin low
//     delayMicroseconds(20000 - pulseWidth); // Wait for the rest of the 20ms period
//   }
// }

void printSensorStates() {
  // for (int i = 0; i < 8; i++) {
  //   Serial.print("Sensor ");
  //   Serial.print(i + 1);
  //   Serial.print(": ");
  //   Serial.print("S.value = ");
  //   Serial.print(sensorValues[i]);
  //   Serial.print(" -> State: ");
  //   Serial.println(sensorStates[i]);
  //   Serial.println(Low_Threshold[i]);
  //   Serial.println(High_Threshold[i]);
  // }
  // Serial.println(" ");
  // delay(500); // Delay for readability in the Serial Monitor
}