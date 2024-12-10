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
#define SERVO_INTERVAL 20
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

//drop cone
bool allBlack = false;
int allBlackRotationCount = 0;
int previousRightRotation = 0;
int previousLeftRotation = 0;

// gripper
int angle;
int pulse;

// PWM
int pwm;
int amount = 0;

// Rotation sensor
int RightRotation = 0;
int LeftRotation = 0;

// Sonar
float duration, ver_dis;
int distance;
bool flagUp = false;
bool hasExecuted = false;

// PID variables
float Kp = 6.0;   // Increase proportional constant for quicker response
float Ki = 0.7;   // Small integral constant to reduce drift over time
float Kd = 1.1;   // Moderate derivative constant to dampen oscillations

float prevError = 0;    // Previous error value (for derivative term)
float integral = 0;     // Accumulated error (for integral term)


// To be able to use the functions via Platform IO
// movement functions
void forward();
void backward();
void stop();
void spin_Right();
void spin_Left();
void turn_Right();
void turn_Left();

// movement functions with rotation
void spin_LeftR();
void spin_RightR();
void turn_RightR();

// sonar
void flagCheck();
void sonar();
void objectAvoidance();
void updateSonar(); 


// line
void Initializing();
void calibrateSensors();
void lineCalibration();
void determineStates();
void printSensorStates();

// rotation
void updaterotation_R1();
void updaterotation_R2();
void reset_Rotations();

//pickup cone
void grabCone();
void forwardRotation1();
void forwardRotation2();
void gripper(int pulse); 

// drop cone
void dropCone();

// PID
void steerError();
int PID(int steeringError);
void adjustSteering(int steeringError);
void updateLineCalibration();

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

gripper(2000);
}
 
void loop()
{
    // always loop begin
  updaterotation_R2();
  updaterotation_R1();
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
  
  PID(steeringError);

  adjustSteering(steeringError); 
  printSensorStates();
}

void flagCheck()
{
  if(flagUp == false)
  {
    Serial.println("Stuck before");
    // if (distance[0] > 0 && distance[1] > 0 && abs(distance[0] - distance[1]) < 20)
    if (distance > 20)
      {
        Serial.println("Does it run");
        flagUp = true;
        Serial.println("flagUp = true");
      }
  }
  delay (500);
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
      Serial.println(timer);
    }
}

void grabCone()
{
    if (!hasExecuted && flagUp) // Check if it hasn't executed and flagUp is true
    {
        gripper(2000);
        forwardRotation1();
        gripper(900);
        forwardRotation2();
        spin_Left();
        stop();

        hasExecuted = true; // Mark as executed
    }
}

void sonar()
{
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    duration = pulseIn(echo, HIGH);
    distance = (duration * 0.017); // Speed of sound is approximately 343 m/s or 0.034 cm/µs

    Serial.print("Distance: ");
    Serial.println(distance);
    updateSonar();
} 

void updateSonar() 
{
  static unsigned long timer;
  if (millis() > timer) 
  {
      timer = millis() + 250; // update every 0.25s can change
  }
}

// void dropCone()
// {
//   //check if all sensors are black
//   for (int i = 0; i < 8; i++)
//   {
//     if (sensorStates[i] == "black")
//     {
//       allBlack = true
//       break;
//     }
//   }

//   // tel 5 rotaties vanaf moment alle lijnen zwart 
//   if (allBlack == true)
//   {  
//     while (RightRotation < 5 && LeftRotation < 5) 
//   {
//     delay(10);
//     stop 
//   }
//   }
// }

/*
check voor alle 8 sensoren
status black 
als afgelopen 5 rotaties alle lijnen zwart dan drop cone
*/


void dropCone() 
{
  // Check if all sensors are black
  
  for (int i = 0; i < 8; i++) {
    if (sensorStates[i] = "black") {
      allBlack = true;
      break;
      Serial.print("sensor all black");
    }
  }

  // If all sensors are black, check rotation
  if (allBlack == true) 
  {
    Serial.print("allblack true");
    // Check if either wheel has completed a new rotation
    if (RightRotation > previousRightRotation || LeftRotation > previousLeftRotation) {
      allBlackRotationCount++;
      previousRightRotation = RightRotation;
      previousLeftRotation = LeftRotation;
      Serial.print("rightrotaion leftrotation");
    }

    // If 5 rotations have been completed while all sensors are black, stop
    if (allBlackRotationCount >= 5) 
    {
      spin_Left();
      allBlackRotationCount = 0;  // Reset for future use
      Serial.print("blackrotations");
    }
  } 
  else 
  {
    // Reset the count if not all sensors are black
    allBlackRotationCount = 0;
    previousRightRotation = RightRotation;
    previousLeftRotation = LeftRotation;
    forward();

    Serial.print("else forward");
  }
}

void objectAvoidance()
{
    int detectionCount = 0;
    for(int attempt = 0; attempt < 3; attempt++)
    {
        sonar(); // Get a fresh distance measurement
        if(ver_dis > 0 && ver_dis <= 30) // Check if object is within 30 cm6
        {
            detectionCount++;
        }
        delay(100); // Short delay between measurements
    }

    if(detectionCount >= 3) // If object detected 3 times
    {
        // Perform avoidance maneuver
        spin_LeftR();
        turn_RightR();
        spin_LeftR();
        stop();
    }
}

void spin_LeftR()
{
    reset_Rotations();

  // Start moving forward
  analogWrite(MotorA2, 0);
  digitalWrite(MotorA1, HIGH); 

  analogWrite(MotorB1, 255);
  digitalWrite(MotorB2, LOW);  

  // Wait until both motors have completed 20 rotations
  while (RightRotation < 7 && LeftRotation < 7) 
  {
    // Do nothing, just wait
    delay(10);  // Small delay to prevent CPU hogging
  }

  // Stop motors after reaching 20 rotations
  stop();

}

void spin_RightR()
{
  reset_Rotations();

  // Start moving forward
  analogWrite(MotorA2, 255);
  digitalWrite(MotorA1, LOW); 

  analogWrite(MotorB1, 0);
  digitalWrite(MotorB2, HIGH);  

  // Wait until both motors have completed 20 rotations
  while (RightRotation < 7 && LeftRotation < 7) 
  {
    // Do nothing, just wait
    delay(10);  // Small delay to prevent CPU hogging
  }

  // Stop motors after reaching 20 rotations
  stop();
}

void turn_RightR()
{
  stop();

  noInterrupts();
  RightRotation = 0;
  LeftRotation = 0;
  interrupts();

  while (LeftRotation < 40 && RightRotation < 20) { // A is left B is right
        if (LeftRotation < 40) {
            digitalWrite(MotorA1, LOW);
            analogWrite(MotorA2, 250);
        } else {
            digitalWrite(MotorA1, LOW);
            analogWrite(MotorA2, 0);
        }
      
        if (RightRotation < 20) {
            digitalWrite(MotorB2, LOW);
            analogWrite(MotorB1, 170);
        } else {
            digitalWrite(MotorB2, LOW);
            analogWrite(MotorB1, 0);
        }
        
  }
  stop();
}


void forwardRotation1()
{
  reset_Rotations();

  // Start moving forward
  analogWrite(MotorA2, 255);
  digitalWrite(MotorA1, LOW); 

  analogWrite(MotorB1, 255);
  digitalWrite(MotorB2, LOW);  

  // Wait until both motors have completed 20 rotations
  while (RightRotation < 30 && LeftRotation < 30) 
  {
    // Do nothing, just wait
    delay(10);  // Small delay to prevent CPU hogging
  }

  // Stop motors after reaching 20 rotations
  stop();
}

void forwardRotation2()
{
  reset_Rotations();
  
  // Start moving forward
  analogWrite(MotorA2, 255);
  digitalWrite(MotorA1, LOW); 

  analogWrite(MotorB1, 255);
  digitalWrite(MotorB2, LOW);  

  // Wait until both motors have completed 20 rotations
  while (RightRotation < 7 && LeftRotation < 7) 
  {
    // Do nothing, just wait
    delay(10);  // Small delay to prevent CPU hogging
  }

  // Stop motors after reaching 20 rotations
  stop();
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