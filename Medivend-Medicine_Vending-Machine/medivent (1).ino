//manuscript ieee format, copyright, 
//aoft copy datasheet, research paper reffered, signed copy of log book, symopsis, video demonstaration 
#include <Vector.h>
#include <AccelStepper.h>
#include <Servo.h>

Servo servo;
AccelStepper stepper(1, 3, 6); // use functions to step
// Vector<Vector<byte>>data;

bool flag = false;
bool flagg = false;
bool servoFlag = false;
bool nextFlag = false;
bool stackFlag = false;
String daata = "";
int data[6] = {0};
int data1;
int num[6] = {0};
int st, st2;
int j = 0;
int k = 0;

bool moveStepper(int stackNo, bool flag){
  int steps[6] = {1000, 2000, 3000};
  if(flag && k < j){
    if((stackNo == 0) || (stackNo == 1)){
      stepper.moveTo(steps[0]);
      // Serial.println("step 0");
    }
    if((stackNo == 2) || (stackNo == 3)){
      stepper.moveTo(steps[1]);
      // Serial.println("step 1");
    }
    if((stackNo == 4) || (stackNo == 5)){
      stepper.moveTo(steps[2]);
      // Serial.println("step 2");
    }
    // if((data[num][0] == 7) || (data[num][0] == 8))
    //   stepper.moveTo(steps[3]);
    // if((data[num][0] == 9) || (data[num][0] == 10))
    //   stepper.moveTo(steps[4]);
    // if((data[num][0] == 11) || (data[num][0] == 12))
    //   stepper.moveTo(steps[5]);
    // Serial.println(String("k value ") + k);
    // Serial.println(String("stackNo: ") + stackNo);
    // Serial.println("nextFlag false");
    return false;
  }
  return false;
}

void moveServo(int var){
  static bool flag1 = false;
  int deflectAngle = 20;
  static int angle = 90;
  int leftAngle = 180;
  int rightAngle = 10;

  if(servoFlag && (millis() - st2 > 1000)){
    if(var % 2 == 0){
      if(data[var] > 0){           //use ternary
        servo.write(leftAngle);
        angle = leftAngle;
        data[var]--;
        flag1 = true;
        st = millis();
        // Serial.println("even, flag1 true");
      }
    }
    else{
      if(data[var] > 0){
        servo.write(rightAngle);
        angle = rightAngle;
        data[var]--;
        flag1 = true;
        st = millis();
        // Serial.println("odd, flag1 true");
      }
    }
    servoFlag = false;
    // Serial.println("servoFlag false");
  }

  if(flag1 && (millis() - st > 1000)){
    if(angle < 90){
      angle += deflectAngle;
      // Serial.println("Deflect < 90");
    }
    else if(angle > 90){
      angle -= deflectAngle;
      // Serial.println("Deflect > 90");
    }
    servo.write(angle);
    flag1 = false;
    // Serial.println("flag1 false");
    servoFlag = true;
    if(data[var] == 0){
      k++;
      flagg = true;        //////////////
      // Serial.println("flagg true");
      servoFlag = false;
    }
    // Serial.println("servoFlag true");
    st2 = millis();
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);           // set up Serial library at 9600 bps
  // Serial.println("Stepper test!");

  pinMode(13, OUTPUT);
  stepper.setMaxSpeed(2000);	
  stepper.setAcceleration(700);
  servo.attach(9, 405, 2550);
  servo.write(90);
}


void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()){
    daata = Serial.readString();
    // data1 = Serial.parseInt();
    // data[0] = data1;
    // Serial.println(String("Data: ") + daata);
    // byte extra = Serial.read();
    // daata[2] = 50;
    for(int i = 0; i < 6; i++){
      if(daata[i] > 48){
        data[i] = daata[i] - 48;
      }
      // Serial.print(data[i]);
      // Serial.print("  ");
      // Serial.println(daata[i]);
    }
    // data[2] = 2;
    nextFlag = true;
    stackFlag = true;
    flag = true;
  }
  if(data[5] == 2){
    digitalWrite(13, HIGH);
    // data[0] = 0;
  }
  // if(data[0] == 1){
  //   digitalWrite(13, LOW);
  //   data[0] = 0;
  // }

  if(stackFlag){
    k = 0;
    j = 0;
    for(int i = 0; i < 6; i++){
      // Serial.println("Num:");
      // Serial.print(data[0]);
      if(data[i] > 0){
        num[j] = i;
        // Serial.println(String("  Num ") + num[j]);
        j++;
      }
      // Serial.println();
    }
    stackFlag = false;
  }
  // if(Serial.available()){
  //   num = 1;
  //   data[num][0] = Serial.read();
  //   nextFlag = true;
  // }

  if((flagg) && ((millis() - st2) > 1000)){
    nextFlag = true;
    // Serial.println("nextFlag true");
    flagg = false;
  }

  nextFlag = moveStepper(num[k], nextFlag);

  if(stepper.distanceToGo() != 0){
    stepper.run();
    flag = true;
    // Serial.println("yess");
  }
  else if(flag){
    // Serial.println("ServoFlag is true");
    servoFlag = true;
    flag = false;
  }
  // else if(stepper.distanceToGo() == 0)
  //   servoFlag = true;
  
  moveServo(num[k]);
}
