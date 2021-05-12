#include <HCSR04.h>
#include <SoftwareSerial.h>
#include <Wire.h>

# define I2C_SLAVE_ADDRESS 11

SoftwareSerial Serial1(2, 3);
HCSR04 hc(2, new int[3] {5, 6, 7}, 3); //initialisation class HCSR04 (trig pin , echo pins, number of sensors)
int uart[9];
int dist;
int distUS[3] = { 0 };

void getUSValues() {

  for (int i = 0; i < 3; i++ ) {
    distUS[i] = hc.dist(i);
  }
}

void getLidarValues() {

  if (Serial1.available()) {
    for (int i = 2; i < 9; i++)
    { //save data in array
      uart[i] = Serial1.read();
    }
    dist = uart[2] + uart[3] * 256;
  }
}

void requestEvents()
{
  getUSValues();
  getLidarValues();
  Wire.write(obstacleDecision());
}

int obstacleDecision(){

  getUSValues();
  getLidarValues();

  /*
  0-> gauche
  1-> en face
  2-> a droite
  3-> lidar
  */

  int val[4] = {distUS[0], distUS[1], distUS[2], dist};
  
  int seuil = 40;

  /*
    mode = 0 -> ras
    mode = 1 -> aller a droite
    mode = 2 -> aller a gauche
    mode = 3 -> evitement
  */
  
  int mode = -1;
  
  if (val[1] <= seuil) { // obstacle devant

    if (val[0] <= seuil && val[2] <= seuil) { // obstacle partout
      mode = 3;
    }
    else if (val[0] <= seuil) { // obstacle gauche
      mode = 1;
    }
    else if (val[2] <= seuil) { // obstacle droite
      mode = 2;
    }
  } else {
    mode = 0;
  }

  return mode;
}

void setup() {

  Wire.begin(I2C_SLAVE_ADDRESS);
  Serial.begin(9600);
  Serial1.begin(115200);
  Wire.onRequest(requestEvents);
}

void loop(){}
