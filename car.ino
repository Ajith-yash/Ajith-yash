#include <AFMotor.h>
#include <Servo.h>
#include <LiquidCrystal.h>

LiquidCrystal lcd(7,6,5,8,9,11);

AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

Servo scanner;

// -------- PINS --------
#define IR_LEFT A0
#define IR_RIGHT A1
#define TRIG A2
#define ECHO A3
#define BATTERY A4

#define CLK 2
#define DT 3

// -------- VARIABLES --------
int mode = 0;
int lastCLK;
int baseSpeed = 150;

float Kp=25, Ki=0.01, Kd=15;
float error=0, prevError=0, integral=0;

unsigned long lastMotionTime=0;

// -------- PATH MEMORY --------
#define MAX_STEPS 200
int movementType[MAX_STEPS];
unsigned long movementTime[MAX_STEPS];
int stepCount=0;

unsigned long actionStartTime;

// 0 = forward
// 1 = left
// 2 = right

// -------- SETUP --------
void setup() {
  lcd.begin(16,2);

  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(BATTERY, INPUT);

  pinMode(CLK, INPUT);
  pinMode(DT, INPUT);

  scanner.attach(10);
  scanner.write(90);

  lastCLK=digitalRead(CLK);

  lcd.print("SMART CAR V2");
  delay(2000);
  lcd.clear();
}

// -------- LOOP --------
void loop() {

  handleEncoder();
  showBattery();

  if(mode==0) obstacleMode();
  if(mode==1) humanMode();
  if(mode==2) returnPath();
}

// -------- BATTERY MONITOR --------
void showBattery() {

  int raw = analogRead(BATTERY);
  float voltage = raw * (5.0 / 1023.0);

  // Reverse voltage divider math
  float actualVoltage = voltage * 3.1; 

  int percentage = map(actualVoltage*100, 650, 840, 0, 100);
  percentage = constrain(percentage,0,100);

  lcd.setCursor(0,0);
  lcd.print("BAT:");
  lcd.print(percentage);
  lcd.print("% ");
}

// -------- ENCODER --------
void handleEncoder(){
  int currentCLK=digitalRead(CLK);

  if(currentCLK!=lastCLK){
    if(digitalRead(DT)!=currentCLK) mode++;
    else mode--;

    if(mode>2) mode=0;
    if(mode<0) mode=2;

    lcd.clear();
  }
  lastCLK=currentCLK;
}

// -------- DISTANCE --------
long getDistance(){
  digitalWrite(TRIG,LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG,HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG,LOW);
  long duration=pulseIn(ECHO,HIGH);
  return duration*0.034/2;
}

// -------- MOTOR --------
void moveForward(int speed){
  motor1.setSpeed(speed);
  motor2.setSpeed(speed);
  motor3.setSpeed(speed);
  motor4.setSpeed(speed);

  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void turnLeft(){
  motor1.run(BACKWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(FORWARD);
}

void turnRight(){
  motor1.run(FORWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(BACKWARD);
}

void stopCar(){
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

// -------- STORE MOVEMENT --------
void storeStep(int type){
  if(stepCount<MAX_STEPS){
    movementType[stepCount]=type;
    movementTime[stepCount]=millis()-actionStartTime;
    stepCount++;
  }
}

// -------- OBSTACLE MODE --------
void obstacleMode(){

  lcd.setCursor(8,0);
  lcd.print("OBS ");

  long d=getDistance();

  if(d<20){
    stopCar();
    turnLeft();
    actionStartTime=millis();
    delay(400);
    storeStep(1);
  }
  else{
    actionStartTime=millis();
    moveForward(baseSpeed);
    storeStep(0);
  }
}

// -------- HUMAN MODE --------
void humanMode(){

  lcd.setCursor(8,0);
  lcd.print("HUM ");

  int L=digitalRead(IR_LEFT);
  int R=digitalRead(IR_RIGHT);

  if(L==LOW && R==LOW){
    lcd.setCursor(0,1);
    lcd.print("HUMAN FOUND   ");
    actionStartTime=millis();
    moveForward(baseSpeed);
    storeStep(0);
  }
  else if(L==LOW){
    error=-1;
    applyPID();
    storeStep(1);
  }
  else if(R==LOW){
    error=1;
    applyPID();
    storeStep(2);
  }
  else{
    stopCar();
  }
}

// -------- PID --------
void applyPID(){
  integral+=error;
  float derivative=error-prevError;
  float output=Kp*error+Ki*integral+Kd*derivative;

  int left=baseSpeed+output;
  int right=baseSpeed-output;

  left=constrain(left,0,255);
  right=constrain(right,0,255);

  motor1.setSpeed(left);
  motor3.setSpeed(left);
  motor2.setSpeed(right);
  motor4.setSpeed(right);

  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);

  prevError=error;
}

// -------- RETURN EXACT PATH --------
void returnPath(){

  lcd.setCursor(8,0);
  lcd.print("RET ");

  stopCar();
  delay(1000);

  for(int i=stepCount-1; i>=0; i--){

    if(movementType[i]==0){
      motor1.run(BACKWARD);
      motor2.run(BACKWARD);
      motor3.run(BACKWARD);
      motor4.run(BACKWARD);
    }
    else if(movementType[i]==1){
      turnRight();
    }
    else if(movementType[i]==2){
      turnLeft();
    }

    delay(movementTime[i]);
  }

  stopCar();
  stepCount=0;
}

// -------- MODE DISPLAY --------
void showMode(){
  lcd.setCursor(13,0);
  if(mode==0) lcd.print("O ");
  if(mode==1) lcd.print("H ");
  if(mode==2) lcd.print("R ");
}