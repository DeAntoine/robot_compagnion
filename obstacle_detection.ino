#include <HCSR04.h>
#include <Arduino.h>     // Every sketch needs this
#include <Wire.h>        // Instantiate the Wire library
#include <TFLI2C.h>      // TFLuna-I2C Library v.0.1.0

int16_t  tfDist;    // distance in centimeters
int16_t  tfAddr = TFL_DEF_ADR;  // Default I2C address
                                // Set variable to your value

UltraSonicDistanceSensor distanceSensor1(13, 12);  // Initialize sensor that uses digital pins 13 and 12.
int distanceMax;//à définir
int distanceMin;//à définir
int ditanceCourant;
int distanceIntermediaire;
int led =11;
int switch2uson = 0;//passage du lidar aux utrason, validé par prevState
int prevState = 0;//pS=0 & switch2uson= 1 -> passage lidar à ultrason 
/***                pS=1 & switch2uson= 0 -> passgae ultrason à lidar
                    pS=0 & switch2uson= 0 -> on utilise le lidar
                    pS=1 & switch2uson= 1 -> on utilise l'ultrason
*/
//int distanceIntermediaire;
void setup () {
    //Serial.begin(9600);  // We initialize serial connection so that we could print values from sensor.
    pinMode(led,OUTPUT);
    Serial.begin( 115200);  // Initalize serial port, for lidar
    Wire.begin();           // Initalize Wire library
    Serial.println( "TFLI2C example code simplified");
    switch2uson = 0;
    prevState = 0;
    
}

void loop () {

    if(switch2uson == 0){
      if(prevState == 1){
        Serial.begin( 115200);
      }
      if( tflI2C.getData( tfDist, tfAddr)) // If read okay...{
        distanceCourant = tfDist;
        if(distanceCourant>distanceMax){
        distanceMax = distanceCourant;
        digitalWrite(led,LOW);

        if(distanceCourant <distanceIntermediaire){
          //on utilisera les ultrasons à la prochaine iteration
          switch2uson =1;
        }
        prevState = 0;
      }


        Serial.print("Dist: ");
        Serial.println(tfDist);         // print the data...
      }
    }
    // Every 500 miliseconds, do a measurement using the sensor and print the distance in centimeters.
    else if(switch2uson == 1){
      if(prevState == 0){
        Serial.begin(9600);
      }
      distanceCourant = distanceSensor.measureDistance;
      Serial.println(distanceSensor.measureDistanceCm());
      delay(500);
      if(distanceCourant>distanceMax){
        distanceMax = distanceCourant;
        digitalWrite(led,LOW);
        switch2uson = 0;
      }
      else if(distanceCourant<distanceMin){
        //allumer LED
        digitalWrite(led,HIGH);
        switch2uson = 1;
    
    
    
      }
      prevState = 1;
      /*else{
        //éteindre LED
        digitalWrite(led,HIGH);
        delay(100);
        digitalWrite(led,LOW);
        delay(100);
      }*/
    }  
}
