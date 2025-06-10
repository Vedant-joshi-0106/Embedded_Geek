//Receiver code
//This code receives mpu data

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define offButton 6
#define buzzer 4
#define heartRate 3
#define mpu 2
int debounce = 300;
long int st, et, dt;

RF24 radio(9, 10); // CE, CSN

const byte address[6] = "00001";
float data[8];

void setup() {
  Serial.begin(115200);
  pinMode(offButton, INPUT_PULLUP);
  pinMode(buzzer, OUTPUT);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}

void loop() {
  if (radio.available()) {
    
    radio.read(&data, sizeof(data));
    for(int i = 0; i < 8; i++)
    {
      Serial.print(data[i]);
      if(i == 8)
      {
        break;
      }
      Serial.print(",  ");
    }
    Serial.println();
  }

  dt = et - st;
  if((data[7] == 1) || ((data[6] >= 60) && ((data[3] >= 200)))){
    digitalWrite(buzzer, HIGH);
  }
  if(data[6] >= 60){
    digitalWrite(heartRate, HIGH);
  }
  if(data[3] >= 200){
    digitalWrite(mpu, HIGH);
  }
  else if((digitalRead(offButton) == 0) && (dt > debounce)){
    st = millis();
    digitalWrite(buzzer, LOW);
    digitalWrite(heartRate, LOW);
    digitalWrite(mpu, LOW);
  }
  et = millis();
}