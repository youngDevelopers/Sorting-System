#include <Servo.h>
#include<Arduino.h>

//defining robotic arm pins
#define BASESERVOPIN 9
#define ARM1SERVOPIN 10
#define ARM2SERVOPIN 11
#define GRIPSERVOPIN 12

#define buz 6
#define led 5

//defining ultrasonic sensor as a proximity sensor for metal sensor
#define Echo1 7
#define Trigger1 8

//defining ultrasonic sensor as a proximity sensor for color sensor
#define Echo2 4
#define Trigger2 3

//motor
#define motorPin1 2
#define motorPin2 13

//TC230 module color sensor
#define potPin A0 //Conveyor speed 
#define S0 A1 //"S0" labeled pin of TC230 attach to pin A1 on arduino uno board
#define S1 A2 //"S1" labeled pin of TC230 attach to pin A1 on arduino uno board
#define S2 A3 //"S2" labeled pin of TC230 attach to pin A1 on arduino uno board
#define S3 A4 //"S3" labeled pin of TC230 attach to pin A1 on arduino uno board
#define sensorOut A5 //"OUT" labeled pin of TC230 attach to pin A5 on arduino uno board
#define capPin A6 //metal detector capacitor
#define pulsePin A7 //pulse pin for metal detector


long sumExpect=0; //running sum of 64 sums 
long ignor=0;   //number of ignored sums
long diff=0;        //difference between sum and avgsum
long pTime=0;
long buzPeriod=0; 
char buzState=0;

//color frequencies variables
int mapped_green_freq;
int mapped_red_freq;
int mapped_blue_freq;

//colors
int Red;
int Green;
int Blue;

//fixed position
double x_grip_position=10;
double y_grip_position=10;
double z_grip_position=10;

double x_red_position=10;
double y_red_position=10;
double z_red_position=10;

double x_blue_position=10;
double y_blue_position=10;
double z_blue_position=10;

double x_green_position= 10;
double y_green_position= 10;
double z_green_position=10;

//Waste disposal servo
Servo myServo;

//arm servos
Servo arm1servo;
Servo arm2servo;
Servo baseservo;
Servo gripservo;

//arm rest position 
double x_rest= 7;
double y_rest= 0;
double z_rest = 75;
double gripAngle=67;

float Pi = 3.141592653589793;

int h = 10;

//object detection function for metal sensor
float distance_Metal () {
  float duration, distance;
  digitalWrite(Trigger1, LOW);
  delayMicroseconds(2);
  digitalWrite(Trigger1, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trigger1, LOW);
  duration = pulseIn(Echo1, HIGH);
  distance = duration * 0.034 / 2;
  return distance;
}

//object detection function for color sensor
float distance_Color () {
  float duration, distance;
  digitalWrite(Trigger2, LOW);
  delayMicroseconds(2);
  digitalWrite(Trigger2, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trigger2, LOW);
  duration = pulseIn(Echo2, HIGH);
  distance = duration * 0.034 / 2;
  return distance;
}

//convert angle to microseconds for pwm
int angleToMicroseconds(double angle) {
  double val = 460.0 + (((2400.0 - 460.0) / 180.0) * angle);
  return (int)val;
}

void moveToAngle(double b, double a1, double a2, double g) {
  arm1servo.writeMicroseconds(angleToMicroseconds(188 - a1));
  arm2servo.writeMicroseconds(angleToMicroseconds(a2+101));
  baseservo.writeMicroseconds(angleToMicroseconds(b+90));
  delayMicroseconds(100);
  gripservo.writeMicroseconds(angleToMicroseconds(g));
}

void moveToPos(double x, double y, double z, double g) {
  double b = atan2(y,x) * (180 / Pi); // base angle

  double l = sqrt(x*x + y*y); // x and y extension bgu  nm   mn                     m                                                                 `````                                                                                                             44hhhhh (l*l + z*z);

  double phi = atan(z/l) * (180 / Pi);

  double theta = acos((h/2)/75) * (180 / Pi);
  
  double a1 = phi + theta; // angle for first part of the arm
  double a2 = phi - theta; // angle for second part of the arm

  moveToAngle(b,a1,a2,g);
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //arm servo motors
  baseservo.attach(BASESERVOPIN,460 ,2400);
  arm1servo.attach(ARM1SERVOPIN,460 ,2400);
  arm2servo.attach(ARM2SERVOPIN,460 ,2400);
  gripservo.attach(GRIPSERVOPIN,460 ,2400);
  //HC SR04 
  pinMode(Trigger1, OUTPUT);
  pinMode(Echo1, INPUT);
  pinMode(Trigger2, OUTPUT);
  pinMode(Echo2, INPUT);
  //TCS230
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, OUTPUT);
  //Color sensor setup
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  //motor
  pinMode(potPin, INPUT);
  pinMode(motorPin1, OUTPUT); 
  pinMode(motorPin2, OUTPUT);

  //metal detection
  pinMode(pulsePin, OUTPUT); 
  digitalWrite(pulsePin, LOW);
  pinMode(capPin, INPUT);  
  pinMode(buz, OUTPUT);
  digitalWrite(buz, LOW);
  pinMode(led, OUTPUT);
  myServo.attach(1);
}

void startConveyorBelt(int speed) {
  analogWrite(motorPin1, speed);
  analogWrite(motorPin2, speed);
  delay(1000);
}


//apply 3 pulses to capacitor for metal detection
void applyPulses()
{
    for (int i=0;i<3;i++) 
    {
      digitalWrite(pulsePin,HIGH); //take 3.5 uS
      delayMicroseconds(3);
      digitalWrite(pulsePin,LOW);  //take 3.5 uS
      delayMicroseconds(3);
    }
}

int metal_detection () {
  int minval=1023;
  int maxval=0;
  long unsigned int sum=0;
  for (int i=0; i<256; i++)
  {
    //reset the capacitor to zero charge
    pinMode(capPin,OUTPUT);
    digitalWrite(capPin,LOW);
    delayMicroseconds(20);
    pinMode(capPin,INPUT);
    //apply 3 pulses to the capacitor
    applyPulses();
    
    //read the charge of capacitor
    int val = analogRead(capPin); //takes 13x8=104 microseconds
    minval = min(val,minval);
    maxval = max(val,maxval);
    sum+=val;
    
    long unsigned int cTime=millis();
    if (cTime<pTime+10) { 
      if (diff>0)
        buzState=1;
      else if(diff<0) 
      buzState=2; 
    } 
    
    if (cTime>pTime+buzPeriod)
    {
      if (diff>0)
      buzState=1;
      else if (diff<0) 
      buzState=2; 
    pTime=cTime; 
    } if (buzPeriod>300)
    buzState=0;

    if (buzState==0)
    {
      digitalWrite(led, LOW);
      digitalWrite(buz, LOW);
    }  
    else if (buzState==1)
    {
      digitalWrite(buz,HIGH);
      digitalWrite(led, HIGH);
      delay(1000);
    }
    
    else if (buzState==2)
    {
      digitalWrite(buz,HIGH);
      digitalWrite(led, HIGH);
      delay(1000);
    }
  }

  //subtract minimum and maximum value to remove spikes
  sum-=minval; 
  sum-=maxval;
  
  if (sumExpect==0) 
  sumExpect=sum<<6; //set sumExpect to expected value 
  long int avgsum=(sumExpect+32)>>6; 
  diff=sum-avgsum;
  if (abs(diff)>10)
  {
    sumExpect=sumExpect+sum-avgsum;
    ignor=0;
  } 
  else 
    ignor++;
  if (ignor>64)
  { 
    sumExpect=sum<<6;
    ignor=0;
  }
  if (diff==0) 
    buzPeriod=1000000;
  else 
  buzPeriod=avgsum/(2*abs(diff));

  return buzState;    
}

void stopBelt() {
  //stop belt
  analogWrite(motorPin1, 0);
  analogWrite(motorPin2, 0);
  delay(1000);
}

void disposeMetal() {
  myServo.write(0);
  delay(1000);
  myServo.write(90);
  delay(1000);
  myServo.write(0);
}



void loop() {
  // put your main code here, to run repeatedly
  int spd = analogRead(potPin);
  int speed = map(spd, 0, 1023, 0, 255);  //mapping analog input values
  //start conveyor belt
  startConveyorBelt(speed);

  //check if the first ultrasonic sensor detect anything
  float first_detection = distance_Metal();
  if (first_detection <=  10){
    stopBelt();
    delay(100);
    int metal = metal_detection();
    if(metal == 1) {
      disposeMetal();
      delay(100);
      startConveyorBelt(speed);
    } else {
      startConveyorBelt(speed);
    }
  } else {
    float second_detection = distance_Color();//color detection position sensor
    //if no object is detected
    if (second_detection > 10.00) {
      moveToPos(x_rest, y_rest, z_rest, 150);
      delay(100);
    }
    //blue object detected
    else if(second_detection <= 10.00  && mapped_blue_freq < 150 && mapped_red_freq > 150 && mapped_green_freq >150){
      stopBelt();
      //move to gripping object position with gripper open
      moveToPos(x_grip_position, y_grip_position, z_grip_position, 90);
      delay(100);
      //grip object
      gripservo.writeMicroseconds(angleToMicroseconds(120));
      //move to designated blue position
      moveToPos(x_blue_position, y_blue_position, z_blue_position, 120);
      delay(100);
      //open gripper
      gripservo.writeMicroseconds(angleToMicroseconds(90));
      delay(100);
      //add blue + 1
      Blue +=1;
    } else if (second_detection <= 10.00 && mapped_blue_freq < 180 && mapped_red_freq < 200 && mapped_green_freq < 150){
      //green object detected
      stopBelt();
      //move to gripping object position with gripper open
      moveToPos(x_grip_position, y_grip_position, z_grip_position, 90);
      delay(100);
      //grip object
      gripservo.writeMicroseconds(angleToMicroseconds(120));
      //move to designated blue position
      moveToPos(x_green_position, y_green_position, z_green_position, 120);
      delay(100);
      //open gripper
      gripservo.writeMicroseconds(angleToMicroseconds(90));
      delay(100);
      //add Green + 1
      Green +=1;
    } else if (second_detection <= 10.00 && mapped_blue_freq < 180 && mapped_red_freq < 155 && mapped_green_freq < 210){
      //red object detected
      stopBelt();
      //move to gripping object position with gripper open
      moveToPos(x_grip_position, y_grip_position, z_grip_position, 90);
      delay(100);
      //grip object
      gripservo.writeMicroseconds(angleToMicroseconds(120));
      //move to designated blue position
      moveToPos(x_red_position, y_red_position, z_red_position, 120);
      delay(100);
      //open gripper
      gripservo.writeMicroseconds(angleToMicroseconds(90));
      delay(100);
      //add red + 1
      Red +=1;
    }
  }
}

//dimension
//Red , Blue, Green= 3.1 x 3.4 x 5.1 ie l x w x h
