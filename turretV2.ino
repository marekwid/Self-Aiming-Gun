
/*Credit to satinder147 on github (SATINDER SINGH) for his method of steering servo motors, with byte values representing direction
https://github.com/satinder147/Automatic-sentry-gun-using-image-processing */

#include <Servo.h>

Servo servoA; //rotates turret horizontally, controls yaw
Servo servoB; //rotates turret vertically, controls pitch
Servo servoC; //activates switch to fire turret

// Arduino pin numbers
int angleX = 2;
int angleY = 2;

const int servoApin = 10;
const int servoBpin = 11;
const int servoCpin = 5;

const int SW_pin = 2; // digital pin connected to switch output
const int XY_pin = A0; // analog pin connected to X output, rotates turret along XY plane
const int YZ_pin = A1; // analog pin connected to Y output, rotates turrent along YZ plane

int posA = 90;
int posB = 90;

int value1;
int value2;



void setup() {
  pinMode(SW_pin, INPUT);
  digitalWrite(SW_pin, HIGH);
  
  Serial.begin(9600);
}

void slowServo(Servo servo, int angle){ //slows Servo motors
 int starting_pos = servo.read();
 if(angle >= starting_pos){
  for(int i = 0; i < angle - starting_pos; i++){
    servo.write(starting_pos + i);
    delay(30);  //changing the time value will alter the motor's speed
  }
 }
 else {
   for(int i = 0; i < starting_pos - angle; i++){
     servo.write(starting_pos - i);
     delay(30);
  }
 }
}

void runServo(Servo servo, int angle, int servopin){  //runs an indivdual servo and then detaches it
  //Serial.print("servo connected\n");
  servo.attach(servopin);
  slowServo(servo, angle); 
  servo.detach();
}



void loop() {
  if(!Serial.available()){ //no connection to python established, control manually
    int valXY = analogRead(XY_pin); //steer turret according to joystick values
    int valYZ = analogRead(YZ_pin);

    valXY = map(valXY, 0, 1023, 0, 180);
    valYZ = map(valYZ, 0, 1023, 0, 180);

    runServo(servoA, valXY, servoApin);
    runServo(servoB, valYZ, servoBpin);

    if(digitalRead(SW_pin) == LOW){  //press joystick to fire, start DC motor
      //servo is attached, then ran slowly, and detached
      //running servos individually like this reduces jittering
      servoC.attach(servoCpin); 
      servoC.write(180);
      delay(1000);
      servoC.write(0);
      delay(200);
      servoC.detach();
    }
  }

  if (Serial.available()) {

    servoA.attach(servoApin);
    servoB.attach(servoBpin);
    
  
    value1 = Serial.read(); //read 1 byte from serial bus
    value2 = Serial.read();

    Serial.write(byte(value1)); //send same to bit to python file
    Serial.write(byte(value2));

    if(value1 > '5'){
      angleX = 5;  //enable faster targeting
      value1 -= 5;
    }
    if(value2 > '5'){
      angleY = 5;
      value2 -= 5;
    }

    if(value1 == '1'){  //move servo in appropriate direction
      posA += angleX;
      servoA.write(posA);
      delay(100);
    }
    if(value1 == '3'){
      posA -= angleX;
      servoA.write(posA);
      delay(100);
    }
    if(value2 == '2'){
      posB += angleY;
      servoB.write(posB);
      delay(100);
    }
    if(value2 == '4'){
      posB -= angleY;
      servoB.write(posB);
      delay(100);
    }

    angleX = 2;
    angleY = 2;


    if(value1 == '5' && value2 == '5'){ //if error is low enough, fire turret
      servoC.attach(servoCpin);
      servoC.write(180);
      delay(1000);
      servoC.write(0);
      delay(200);
      servoC.detach();
    }

    servoA.detach();  //detach servos
    servoB.detach();
  }
}
