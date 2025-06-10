//Transmitter code
//This code transmits mpu and heartrate data

#include <MPU6050_tockn.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define button 4
int debounce = 300;
long int st, et, dt;

MPU6050 mpu6050(Wire);
RF24 radio(7, 8); // CE, CSN

MAX30105 particleSensor;

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;
const byte address[6] = "00001";
float data[8] = {};

void setup() {
  pinMode(button, INPUT_PULLUP);
  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED

  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
}

void loop() {
  //receiving data from MPU 6050
  dt = et - st;
  if((digitalRead(button) == 0) && (dt > debounce)){
    st = millis();
    data[7] = 1;
    Serial.println(data[8]);
  }
  mpu6050.update();
  data[0] = mpu6050.getAccX();
  data[1] = mpu6050.getAccY();
  data[2] = mpu6050.getAccZ();
  data[3] = mpu6050.getGyroX();
  data[4] = mpu6050.getGyroY();
  data[5] = mpu6050.getGyroZ();

  long irValue = particleSensor.getIR();

  if (checkForBeat(irValue) == true) {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20) {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

  data[6] = beatAvg;
  if (irValue < 50000)
    data[6] = 0;
  // Serial.println(data[9]);

  radio.write(&data, sizeof(data));
  data[7] = 0;
  et = millis();
}