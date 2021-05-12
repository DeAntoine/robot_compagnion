#include <Servo.h>

Servo servo_pan;
Servo servo_tilt;
int positionx;
int positiony;
int k = 2;

void setup() {
  Serial.begin(9600);
  servo_pan.attach(10);
  servo_tilt.attach(9);
  positionx = 100;
  positiony = 120;
  servo_pan.write(positionx);
  servo_tilt.write(positiony);
}

void loop() {
  
  char incomingByte;
  if (Serial.available()) {
    incomingByte = Serial.read();
    Serial.println(incomingByte );
    Serial.println(positionx );
    Serial.println(positiony );
    if (incomingByte == 'b') {
      positiony = decr(positiony);
      servo_tilt.write(positiony);
    }

    else if (incomingByte == 'i')
    {
      positiony = 160;
      positionx = 100;
      servo_tilt.write(positiony);
      servo_pan.write(positionx);
    }

    else if (incomingByte == 'h')
    {
      positiony = incr(positiony);
      servo_tilt.write(positiony);
    }

    else if (incomingByte == 'd')
    {
      positionx = incr(positionx);
      servo_pan.write(positionx);
    }

    else if (incomingByte == 'g')
    {
      positionx = decr(positionx);
      servo_pan.write(positionx);
    }
  }
}

int incr(int position)
{
  if(position>=180)
  {
    return 180;
  }
  return position+k;
}  

int decr(int position)
{
  if(position<=0)
  {
    return 0;
  }
  return position-k;
} 
