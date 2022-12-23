#include <Servo.h>

Servo servoA; //rotates turret horizontally, controls yaw
Servo servoB; //rotates turret vertically, controls pitch
Servo servoC; //activates switch to fire turret

String x; //recieve data from python
String y;

// Arduino pin numbers
int angleX = 2;
int angleY = 2;

const int servoApin = 10;
const int servoBpin = 11;
const int servoCpin = 9;

const int SW_pin = 2; // digital pin connected to switch output
const int XY_pin = A0; // analog pin connected to X output, rotates turret along XY plane
const int YZ_pin = A1; // analog pin connected to Y output, rotates turrent along YZ plane

int value1;
int value2;



void setup() {
  pinMode(SW_pin, INPUT);
  digitalWrite(SW_pin, HIGH);
  
  Serial.begin(9600);
}

void slowServo(Servo servo, int angle){ //slows Servo motors, can be used for smoother motor motion
 int starting_pos = servo.read();
 if(angle >= starting_pos){
  for(int i = 0; i < angle - starting_pos; i++){
    servo.write(starting_pos + i);
    delay(15);  //changing the time value will alter the motor's speed
  }
 }
 else {
   for(int i = 0; i < starting_pos - angle; i++){
     servo.write(starting_pos - i);
     delay(15);
  }
 }
}

void runServo(Servo servo, int angle, int servopin){  //runs an indivdual servo and then detaches it
  //Serial.print("servo connected\n");
  servo.attach(servopin);
  slowServo(servo, angle);
  servo.detach();
}

//determines the number of degrees that a servo must move forward/backwards depending on the gun's distance from its target
int determineAngle(int error){  
  int angle = 5;
  
  if(error < 200){
    angle = 4;
  }
  if(error < 100){
    angle = 3;
  }
  if(error < 50){
    angle = 2;
  }
  if(error < 25){
    angle = 1;
  }
    
  return angle;
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
  
    x = Serial.readStringUntil('\r'); //read horizonatal (x) distance from target, measured in pixels
    //Serial.println(x);
    delay(10);
    while(Serial.available() == 0){
    }
    y = Serial.readStringUntil('\r'); //read vertical (y) distance from target, measured in pixels
    //Serial.println(y);
    delay(10);
  }

  long errX = x.toInt();  //convert string data into interger form
  int errorX = int(errX);
  int angleX = determineAngle(abs(errorX));

  //(optional) send angle back to main computer to verify code
  //Serial.println(x + " " + String(angleX));
  //delay(10);

  long errY = y.toInt();  //convert string data into interger form
  int errorY = int(errY);
  int angleY = determineAngle(abs(errorY));

  //(optional) send angle back to main computer to verify code
  //Serial.println(y + " " + String(angleY));
  //delay(10);

  if(angleX == 1 && angleY == 1){ //if distance between target is suffciently low, fire
    servoC.attach(servoCpin);
      servoC.write(180);
      delay(1000);
      servoC.write(0);
      delay(200);
      servoC.detach();
  }

  if(errorX < 0){ //change sign of angle depending on direction the servo should move
    angleX = -1 * angleX;
  }

   if(errorY < 0){
    angleY = -1 * angleY;
  }

  servoA.attach(servoApin); //move yaw servo by appropriate angle
  int posA = int(servoA.read() + angleX);
  //Serial.println("posA " + String(posA));
  //delay(10);
  servoA.write(posA);
  if(posA >= 0 && angleX >= 0 || posA <= 180 && angleX <= 0){
    //servoA.write(posA + angleX);
  }
  servoA.detach();

  servoB.attach(servoBpin); //move pitch servo by appropriate angle
  int posB = int(servoB.read() + angleY);
  //Serial.println("posB " + String(posB));
  //delay(10);
  servoB.write(posB);
  if(posB >= 0 && angleY >= 0 || posB <= 180 && angleY <= 0){
    //servoB.write(posB + angleY);
  }
  servoB.detach();
}
