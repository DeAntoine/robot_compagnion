#include <Wire.h>

# define I2C_SLAVE1_ADDRESS 11

int value = -1;
int GA = 11, GB = 10, DA = 5, DB = 6;

void ar() //arriére
{
  digitalWrite(DA, HIGH);
  digitalWrite(DB, LOW);
  digitalWrite(GA, HIGH);
  digitalWrite(GB, LOW);
}

void av() //avant
{
  digitalWrite(DA, LOW);
  digitalWrite(DB, HIGH);
  digitalWrite(GA, LOW);
  digitalWrite(GB, HIGH);
}

void g()//gauche
{
  digitalWrite(DA, LOW);
  digitalWrite(DB, HIGH);
  digitalWrite(GA, HIGH);
  digitalWrite(GB, LOW);
}

void d()//droite
{
  digitalWrite(DA, HIGH);
  digitalWrite(DB, LOW);
  digitalWrite(GA, LOW);
  digitalWrite(GB, HIGH);
}

void stop()//arret
{
  digitalWrite(DA, LOW);
  digitalWrite(DB, LOW);
  digitalWrite(GA, LOW);
  digitalWrite(GB, LOW);
}

char requestDataFromRaspberry() {

  char incomingByte;
  if (Serial.available()) {
    incomingByte = Serial.read();
    Serial.println(incomingByte );
  }
  return incomingByte;
}

char requestCaptorsValues() {

  Wire.requestFrom(I2C_SLAVE1_ADDRESS, 1);

  while (value == -1)
    value = Wire.read();

  switch (value) {

    case 0:
      return 'a';
    case 1:
      return 'd';
    case 2:
      return 'g';
    case 3:
      return 'e';
  }
}

char choose() {

  requestCaptorsValues();
  char c = requestDataFromRaspberry();
  char v = requestCaptorsValues();
  if (v == 0) {
    return c;
  } else {
    return v;
  }
}

void action() {

  char dec = choose();

  if (dec == 'd') {

    d();
    delay(500);
  } else if (dec == 'g') {

    g();
    delay(500);
  } else if (dec == 'a') {

    av();
    delay(500);
  } else if (dec == 'e') {

    ar();
    delay(500);
    d();
    delay(500);
  } else {
    stop();
  }

}

void setup()
{

  Wire.begin();
  Serial.begin(9600);
  pinMode(DA, OUTPUT);
  pinMode(DB, OUTPUT);
  pinMode(GA, OUTPUT);
  pinMode(GB, OUTPUT);
}


void loop()
{
  action();
}
