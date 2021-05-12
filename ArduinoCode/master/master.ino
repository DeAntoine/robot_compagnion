#include <Wire.h>

#define BUFFER_SIZE  2
#define MAX_NUMBER_OF_SLAVES 24
#define FIRST_SLAVE_ADDRESS 1
#define READ_CYCLE_DELAY 1000

byte buffer[BUFFER_SIZE];
char incomingByte;

void setup()
{
  Serial.begin(9600);
  Serial.println("MASTER READER");
  Serial.println("*************");

  Wire.begin();        // Activate I2C link
}

void loop()
{
  for (int slaveAddress = FIRST_SLAVE_ADDRESS; slaveAddress <= MAX_NUMBER_OF_SLAVES; slaveAddress++)
  {
    Wire.requestFrom(slaveAddress, BUFFER_SIZE);    // request data from the slave

    if (Wire.available() == BUFFER_SIZE)
    {
      // Reads the buffer the slave sent
      for (int i = 0; i < BUFFER_SIZE; i++)
      {
        buffer[i] = Wire.read();  // gets the data
      }

      // Prints the income data
      Serial.print("Slave address ");
      Serial.print(slaveAddress);
      Serial.print(": ultrasons = ");
      Serial.print(buffer[0]);
      Serial.print("; lidar = ");
      Serial.println(buffer[1]);
    }
  }
  
  Serial.println("*************************");

  if (Serial.available()) {
    incomingByte = Serial.read();

  delay(READ_CYCLE_DELAY);
}
