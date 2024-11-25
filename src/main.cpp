// #include <Arduino.h>

// #define trig 4
// #define echo 7
// #define MotorA1 8 // MotorA1 at port 8 digital
// #define MotorA2 10 // MotorA2 at port 10 analog
// #define MotorB1 11 // MotorB1 at port 11 analog
// #define MotorB2 9 // MotorB2 at port 9 analog
 
// void forward();
// void backward();
// void stop();
// void spin_Right();
// void spin_Left();
// void turn_Right();
// void turn_Left();
// void sonar();
// void sonarCheck();
// void forwardCheck();
// void backwardCheck();
// void stopCheck();
// void spin_RightCheck();
// void spin_LeftCheck();
// void turn_RightCheck();
// void turn_LeftCheck();


// int pwm;
// int amount = 0;
 
// void setup()
// {
//  Serial.begin(9600);
//  pinMode(MotorA1, OUTPUT); // MotorA1 is output
//  pinMode(MotorA2, OUTPUT); // MotorA2 is output
//  pinMode(MotorB1, OUTPUT); // MotorB1 is output
//  pinMode(MotorB2, OUTPUT); // MotorB2 is output
//  pinMode(trig, OUTPUT);
//  pinMode(echo, INPUT);
// }
 
// void loop()
// {
//   // delay(1000);
//   // forward();
//   // stop();
//   // backward();
 
// // 1st
//   // turn_Left();
//   // spin_Right();
//   // turn_Right();
//   // spin_Left();
//   // stop();
 
//   sonar();
// }

// void sonarCheck()
// {
//   static unsigned long timer;
//   if (millis() > timer)
//   {
//     //sonar();
//     timer = millis() + 250;
//   }
// }

// void forwardCheck()
// {
//   static unsigned long timer;
//   if (millis() > timer)
//   {
//     //forward();
//     timer = millis() + 200;
//   }
// }

// void backwardCheck()
// {
//   static unsigned long timer;
//   if (millis() > timer)
//   {
//     //backward();
//     timer = millis() + 200;
//   }
// }
 
// void stopCheck()
// {
//   static unsigned long timer;
//   static bool state;
//   if (millis() > timer)
//   {
//     //stop();
//     timer = millis() + 1000;
//   }
// }
 
// void spin_RightCheck()
// {
//   static unsigned long timer;
//   if (millis() > timer)
//   {
//     //spin_Right();
//     timer = millis() + 510;
//   }
// }

// void spin_LeftCheck()
// {
//   static unsigned long timer;
//   if (millis() > timer)
//   {
//     //spin_Left();
//     timer = millis() + 550;
//   }
// }

// void turn_RightCheck()
// {
//   static unsigned long timer;
//   if (millis() > timer)
//   {
//     //turn_Right();
//     timer = millis() + 2625;
//   }
// }

// void turn_LeftCheck()
// {
//   static unsigned long timer;
//   if (millis() > timer)
//   {
//     //turn_Left();
//     timer = millis() + 1800;
//   }
// }
 
 
// void sonar() // printing distance
// {
//   digitalWrite(trig, LOW);
//   delayMicroseconds(2);
//   digitalWrite(trig, HIGH);
//   delayMicroseconds(10);
//   digitalWrite(trig, LOW);
 
//   long duration = pulseIn(echo, HIGH);
//   int distance = (duration * 0.017); // Distance in cm
 
//   Serial.print("Distance: ");
//   Serial.print(distance);
//   Serial.println(" cm");
 
//   if (distance <= 40)
//   {
//     forward();
//     if (distance <= 35)
//     {
//       forward();
//       if (distance <= 30)
//       {
//         spin_Left();
//         turn_Right();
//         turn_Right();
//         spin_Left();
//         stop();
//       }
//     }
//   }
//   else
//   {
//     forward();
//   }
//   return;
// }
 
// void forward()
// {
//   digitalWrite(MotorA1, 0);
//   analogWrite(MotorB2, 0);
//   analogWrite(MotorA2, 250); // Left motor runs at full speed
 
//   for(int pwm = 255; pwm < 256; pwm++)
//   {
//     analogWrite(MotorB1, pwm - 12); // Right motor with PWM control, slightly reduced
//     //delay(200);
//     forwardCheck();
//   }
//   // delay(5000);
// }
 
// void backward()
// {
//   analogWrite(MotorA2, 0);
//   analogWrite(MotorB1, 0);
//   analogWrite(MotorA1, 250); // Left motor runs at full speed
 
//   for(int pwm = 255; pwm < 256; pwm++)
//   {
//     analogWrite(MotorB2, pwm -17); // Right motor with PWM control, slightly reduced
//     backwardCheck();
//   }
//   //delay(5000);
// }
 
// void stop() // stop all motors
// {
//   analogWrite(MotorA1, 0);
//   analogWrite(MotorA2, 0);
//   analogWrite(MotorB1, 0);
//   analogWrite(MotorB2, 0);
//   stopCheck();
// }
 
// void spin_Right() // spin right on its axle in a 90 degree angle
// {
//   analogWrite(MotorA1, 0); // left backward
//   analogWrite(MotorA2, 255); // left forward full speed
//   analogWrite(MotorB1, 0); // right forward
//   analogWrite(MotorB2, 255); // right backward full speed
//   //delay(510); // 455
//   spin_RightCheck();
// }
 
// void spin_Left() // spin left on its axle in a 90 degree angle
// {
//   analogWrite(MotorA1, 250); // left backward full speed
//   analogWrite(MotorA2, 0); // left forward
//   analogWrite(MotorB1, 250); // right forward full speed
//   analogWrite(MotorB2, 0); // right backward
//   //delay(550); //495
//   spin_LeftCheck();
// }
 
// void turn_Left() // turn left with a 90 degree angle
// {
//   analogWrite(MotorA2, 255);
//   analogWrite(MotorB1, 255);
//   delay(100);
//   analogWrite(MotorA1, 0); // left backward
//   analogWrite(MotorA2, 140); // left forward
//   analogWrite(MotorB1, 255); // right forward
//   analogWrite(MotorB2, 0);   // right backward
//   //delay(1800);
//   turn_LeftCheck();
// }
 
// void turn_Right() // turn right with a 90 degree angle
// {
//   analogWrite(MotorA2, 255);
//   analogWrite(MotorB1, 255);
//   delay(100);
//   analogWrite(MotorA1, 0); // left backward
//   analogWrite(MotorA2, 255); // left forward
//   analogWrite(MotorB1, 140); // right forward
//   analogWrite(MotorB2, 0);   // right backward
//   //delay(2625);
//   turn_RightCheck();
// }

#include <Arduino.h>

#define trig 4
#define echo 7
#define MotorA1 8 // MotorA1 at port 8 digital
#define MotorA2 10 // MotorA2 at port 10 analog
#define MotorB1 11 // MotorB1 at port 11 analog
#define MotorB2 9 // MotorB2 at port 9 analog
 
void forward();
void backward();
void stop();
void spin_Right();
void spin_Left();
void turn_Right();
void turn_Left();
void sonar();


int pwm;
int amount = 0;
 
void setup()
{
 pinMode(MotorA1, OUTPUT); // MotorA1 is output
 pinMode(MotorA2, OUTPUT); // MotorA2 is output
 pinMode(MotorB1, OUTPUT); // MotorB1 is output
 pinMode(MotorB2, OUTPUT); // MotorB2 is output
 Serial.begin(9600);
 pinMode(trig, OUTPUT);
 pinMode(echo, INPUT);
 
}
 
void loop()
{
  // delay(1000);
  // forward();
  // stop();
  // backward();
 
// 1st
  // turn_Left();
  // spin_Right();
  // turn_Right();
  // spin_Left();
  // stop();
 
  sonar();
}
 
void sonar() // printing distance
{
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
 
  long duration = pulseIn(echo, HIGH);
  int distance = (duration * 0.034) / 2; // Distance in cm
 
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
  digitalWrite(MotorA1, 0);
  analogWrite(MotorB2, 0);
  analogWrite(MotorA2, 250); // Left motor runs at full speed
 
  for(int pwm = 255; pwm < 256; pwm++)
  {
    analogWrite(MotorB1, pwm - 12); // Right motor with PWM control, slightly reduced
    delay(200);
  }
  // delay(5000);
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


