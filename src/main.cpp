//------------------------------------------------------------------------
// Motor - USING PWM
//-------------------------------------------------------------------------
#define MotorA1 8 // MotorA1 at port 8 digital
#define MotorA2 10 // MotorA2 at port 10 analog
#define MotorB1 11 // MotorB1 at port 11 analog 
#define MotorB2 9 // MotorB2 at port 9 analog

// A1 left backward
// A2 left forward
// B1 right forward
// B2 right backward

int pwm;
int amount = 0;

void setup()
{
 pinMode(MotorA1, OUTPUT); // MotorA1 is output
 pinMode(MotorA2, OUTPUT); // MotorA2 is output
 pinMode(MotorB1, OUTPUT); // MotorB1 is output
 pinMode(MotorB2, OUTPUT); // MotorB2 is output
}

void loop()
{
  // delay(1000);
  // forward();
  // stop();
  // backward();
  // stop();
  // spin_Left();
  // stop();
  // spin_Right();
  // stop();
  // turn_Left();
  // stop();
  // turn_Right();
  // stop();
}

void forward()
{
  digitalWrite(MotorA1, 0);
  analogWrite(MotorB2, 0);
  analogWrite(MotorA2, 250); // Left motor runs at full speed

  for(int pwm = 255; pwm < 256; pwm++)
  {
    analogWrite(MotorB1, pwm - 12); // Right motor with PWM control, slightly reduced
    delay(200);
  }
  delay(5000);
}

void backward()
{
  analogWrite(MotorA2, 0);
  analogWrite(MotorB1, 0);
  analogWrite(MotorA1, 250); // Left motor runs at full speed

  for(int pwm = 255; pwm < 256; pwm++)
  {
    analogWrite(MotorB2, pwm -17); // Right motor with PWM control, slightly reduced
    delay(200);
  }
  delay(5000);
}

void stop() // stop all motors
{
  analogWrite(MotorA1, 0);
  analogWrite(MotorA2, 0);
  analogWrite(MotorB1, 0);
  analogWrite(MotorB2, 0);
  delay(1000);
}

void spin_Right() // spin right on its axle in a 90 degree angle
{
  analogWrite(MotorA1, 0); // left backward
  analogWrite(MotorA2, 255); // left forward full speed
  analogWrite(MotorB1, 0); // right forward
  analogWrite(MotorB2, 255); // right backward full speed
  delay(510); // 455
}

void spin_Left() // spin left on its axle in a 90 degree angle
{
  analogWrite(MotorA1, 250); // left backward full speed
  analogWrite(MotorA2, 0); // left forward
  analogWrite(MotorB1, 250); // right forward full speed
  analogWrite(MotorB2, 0); // right backward 
  delay(550); //495
}

void turn_Left() // turn left with a 90 degree angle
{
  analogWrite(MotorA2, 255);
  analogWrite(MotorB1, 255);
  delay(100);
  analogWrite(MotorA1, 0); // left backward 
  analogWrite(MotorA2, 140); // left forward
  analogWrite(MotorB1, 255); // right forward 
  analogWrite(MotorB2, 0);   // right backward 
  delay(1800);
}

void turn_Right() // turn right with a 90 degree angle
{
  analogWrite(MotorA2, 255);
  analogWrite(MotorB1, 255);
  delay(100);
  analogWrite(MotorA1, 0); // left backward 
  analogWrite(MotorA2, 255); // left forward
  analogWrite(MotorB1, 140); // right forward 
  analogWrite(MotorB2, 0);   // right backward 
  delay(2625);
}

