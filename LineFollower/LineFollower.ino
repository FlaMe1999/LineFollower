//PINS FOR DRIVER NEEDS UPDATING
//NEED TO UPDATE THE RELAY WIRES PHYSICALLY
#define left_dir1 3//defining all the motor pins for the motor driver
#define left_dir2 2
#define left_enable 10

#define right_dir1 4
#define right_dir2 5
#define right_enable 12

/**
#define left_dir1 4//defining all the motor pins for the motor driver
#define left_dir2 5
#define left_enable 10

#define right_dir1 3
#define right_dir2 2
#define right_enable 12*/
#define trig  8  //defining the Trigger and Echo pin of Ultrasonic Sensor
#define echo  9

#define rack1 6
#define rack2 7  //This part is for handling the Rack and Pinion Mechanism


const int irPins[6] = {A1, A2, A3, A4, A5, A6}; //Though 8 sensors are there but 6 sensor are used in this code
//const int irPins[6] = {A6,A5,A4,A3,A2,A1};
int irSensorDigital[6] = {0, 0, 0, 0, 0, 0}; //Set the basic value to 0 for all the sensors
int irSensors = B000000; //The value of the sensors are stored in the variable isSensors
int count = 0; //For PID calculation
int error = 0; //For PID calculation
int errorLast = 0;  //For PID calculation
int correction = 0; //For PID calculation
int sensorsOnLine =0;
int lastSensOn =0;

int maxSpeed = 65; //adjust for oyur own bot, if your bot is driving too fast reduce this else increase this
int motorLSpeed = 0;
int motorRSpeed = 0;
int duration;
unsigned int dist;
unsigned int obstkl = 10;

int rev=0;
void setup(){
  Serial.begin(9600);
  pinMode(left_dir1, OUTPUT); //Set the pins for controlling the motor as OUTPUT
  pinMode(left_dir2, OUTPUT);
  pinMode(right_dir1, OUTPUT);
  pinMode(right_dir2, OUTPUT);
  pinMode(left_enable, OUTPUT);
  pinMode(right_enable, OUTPUT);
  pinMode(rack1, OUTPUT);
  pinMode(rack2, OUTPUT);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  digitalWrite(trig, LOW);
  digitalWrite(rack1, HIGH);
  digitalWrite(rack2, HIGH);
  

  dist = 0;
  dist -= 1;

  /* Set-up IR sensor pins as input */
  for (int i = 0; i < 6; i++)
  {
    pinMode(irPins[i], INPUT);
  }
motorLSpeed=65;
Drive();
}

void loop() {
  ultrasonic(); //For measuring the distance
  Scan(); //for scanning the sensors
  UpdateError(); //for PID calculation
  UpdateCorrection();////for PID calculation
  if (dist < obstkl) {
    motorLSpeed = 0;
    motorRSpeed = 0;
  }

  Drive(); //For driving the motor
  rack(); // For movement of rack and pinion
  //Serial.println();
}

void Scan()
{
  count = 0;

  irSensors = B000000;

  for (int i = 0; i < 6; i++)
  {
//CHANGED for event!
    if (digitalRead(irPins[i]) == 1)
      irSensorDigital[i] = 1;
    if (digitalRead(irPins[i]) == 0)
      irSensorDigital[i] = 0;

    count = count + irSensorDigital[i];
    int b = 5 - i;
    irSensors = irSensors + (irSensorDigital[i] << b); //For summing up the sensor value as a binary input
    //Serial.print(irSensorDigital[i]);
  }
  //Serial.println();
}

void UpdateError() { //This whole part is for PID calculation, the error is minimized, and the speed of motor is determined

  errorLast = error;
  lastSensOn = sensorsOnLine;

  switch (irSensors) {

    case B000000:                         //No sensor is on the line
      if (errorLast < 0) {
        error = -180;
        sensorsOnLine=0;
      }
      else if (errorLast > 0) {
        error = 180;
        sensorsOnLine=0;
      }
    /*  else
      {
    /*analogWrite(right_enable, abs(55));
    digitalWrite(right_dir1, LOW);
    digitalWrite(right_dir2, HIGH);
    analogWrite(left_enable, abs(55));
    digitalWrite(left_dir1, LOW);
    digitalWrite(left_dir2, HIGH);
    rev=1;  
    //Serial.print("REVERSEDDDDD ");*/
    //  }
    
      break;

    case B100000: // leftmost sensor on the line
      error = -150;
      sensorsOnLine=1;
      break;

    case B010000: //2nd left sensor on the line
      error = -90;
      sensorsOnLine=1;
      break;

    case B001000: // 3rd left sensor on the line
      error = -30;
      sensorsOnLine=1;
      break;

    case B000100: // 3rd right sensor on the line
      error = 30;
      sensorsOnLine=1;
      break;

    case B000010: // 2nd right sensor on the line
      error = 90;
      sensorsOnLine=1;
      break;

    case B000001: // rightmost sensor on the line
      error = 150;
      sensorsOnLine=1;
      break;

    /* 2 Sensors on the line */

    case B110000:
      error = -120;
      sensorsOnLine=2;
      break;

    case B011000:
      error = -60;
      sensorsOnLine=2;
      break;

    case B001100:
      error = 0;
      sensorsOnLine=2;
      break;

    case B000110:
      error = 60;
      sensorsOnLine=2;
      break;

    case B000011:
      error = 120;
      sensorsOnLine=2;
      break;

    /* 3 Sensors on the line */

    case B111000:
    case B011100:
      error = -150;
      sensorsOnLine=3;
      break;

    case B000111:
    case B001110:
      error = 150;
      sensorsOnLine=3;
      break;

    /* 4 Sensors on the line */
    case B111100:
      error = -150;
      sensorsOnLine=4;
      break;

    case B111010:
      error = -150;
      sensorsOnLine=4;
      break;

    case B001111:
      error = 150;
      sensorsOnLine=4;
      break;

    case B010111:
      error = 150;
      sensorsOnLine=4;
      break;

    /* 5 Sensors on the line */
    case B111110:
      error = -150;
      sensorsOnLine=5;
      break;

    case B011111:
      error = +150;
      sensorsOnLine=5;
      break;

    case B111111:
      error = 0;
      sensorsOnLine=6;
      break;

    default:
      error = errorLast;
      sensorsOnLine=lastSensOn;
  }
}

void UpdateCorrection() { //error are corrected here

  if (error >= 0 && error < 30) {
    correction = 0;
  }

  else if (error >= 30 && error < 60) {
    correction = 15;
  }

  else if (error >= 60 && error < 90) {
    correction = 40;
  }

  else if (error >= 90 && error < 120) {
    correction = 55;
  }

  else if (error >= 120 && error < 150) {
    correction = 75;
  }

  else if (error >= 150 && error < 180) {
    correction = 85;
  }

  else if (error >= 180) {
    correction = 305;
  }

  if (error <= 0 && error > -30) {
    correction = 0;
  }

  else if (error <= -30 && error > -60) {
    correction = -15;
  }

  else if (error <= -60 && error > -90) {
    correction = -40;
  }

  else if (error <= -90 && error > -120) {
    correction = -55;
  }

  else if (error <= -120 && error > -150) {
    correction = -75;
  }

  else if (error <= -150 && error > -180) {
    correction = -85;
  }

  else if (error <= -180) {
    correction = -305;
  }

  if (correction >= 0) {
    motorRSpeed = maxSpeed;
    motorLSpeed = maxSpeed - correction;
  }

  else if (correction < 0) {
    motorRSpeed = maxSpeed + correction;
    motorLSpeed = maxSpeed;
  }
  //Serial.print("ERROR:");
  //Serial.print(error);
  //Serial.print("   ");
  //Serial.print("CORRECTION:");
  //Serial.print(correction);
  //Serial.print("   ");
  //Serial.print("RSPEDD: ");
  //Serial.print(motorRSpeed);
  //Serial.print("   ");
  //Serial.print("LSPEDD: ");
  //Serial.print(motorLSpeed);
  //Serial.print("   ");
}

void Drive() { //This function will drive the motor
  if (rev==1){
    return;
  }
  if (motorRSpeed > 255) {
    motorRSpeed = 255;
  }
  else if (motorRSpeed < -255) {
    motorRSpeed = -255;
  }

  if (motorLSpeed > 255) {
    motorLSpeed = 255;
  }
  else if (motorLSpeed < -255) {
    motorLSpeed = -255;
  }

    if (motorRSpeed <15&& motorRSpeed>=0) {
    motorRSpeed = 15;
  }
  else if (motorRSpeed >-15&& motorRSpeed<=0) {
    motorRSpeed = -15;
  }
    if (motorLSpeed <15&& motorLSpeed>=0) {
    motorLSpeed = 15;
  }
  else if (motorLSpeed >-15&& motorLSpeed<=0) {
    motorLSpeed = -15;
  }

  if (motorRSpeed < -maxSpeed){
    motorRSpeed = -maxSpeed;
  }
  else if (motorRSpeed >maxSpeed){
    motorRSpeed = maxSpeed;
  }
  
  if (motorLSpeed < -maxSpeed){
    motorLSpeed = -maxSpeed;
  }  
  else if (motorLSpeed >maxSpeed){
    motorLSpeed = maxSpeed;
  }

  if (abs(error)>=120){
    motorLSpeed=-motorLSpeed;
    motorRSpeed=-motorRSpeed;
  }
//-305 to 560
/*
motorRSpeed = map(motorRSpeed,maxSpeed-305,maxSpeed+305,-maxSpeed,maxSpeed)+10;
motorLSpeed = map(motorLSpeed,maxSpeed-305,maxSpeed+305,-maxSpeed,maxSpeed)+10;
*/

  if (motorRSpeed > 0) { // right motor forward (using PWM)
    analogWrite(right_enable, motorRSpeed);
    digitalWrite(right_dir1, HIGH);
    digitalWrite(right_dir2, LOW);
  }

  else if (motorRSpeed < 0) {// right motor reverse (using PWM)
    analogWrite(right_enable, abs(motorRSpeed));
    digitalWrite(right_dir1, LOW);
    digitalWrite(right_dir2, HIGH);
  }

  else if (motorRSpeed == 0) { // right motor fast stop
    analogWrite(right_enable, HIGH);
    digitalWrite(right_dir1, LOW);
    digitalWrite(right_dir2, LOW);
  }

  if (motorLSpeed > 0) { // right motor forward (using PWM)
    analogWrite(left_enable, motorLSpeed);
    digitalWrite(left_dir1, HIGH);
    digitalWrite(left_dir2, LOW);
  }

  else if (motorLSpeed < 0) { // right motor reverse (using PWM)
    analogWrite(left_enable, abs(motorLSpeed));
    digitalWrite(left_dir1, LOW);
    digitalWrite(left_dir2, HIGH);
  }

  else if (motorLSpeed == 0) { // left motor fast stop
    digitalWrite(left_enable, HIGH);
    digitalWrite(left_dir1, LOW);
    digitalWrite(left_dir2, LOW);
  }
  
  //Serial.print("AL:");
  //Serial.print(motorLSpeed);
  //Serial.print("   ");
  //Serial.print("AR:");
  //Serial.print(motorRSpeed);
  //Serial.print("   ");
}
void ultrasonic() {
  dist = 0;
  digitalWrite(trig, LOW);
delayMicroseconds(2);
digitalWrite(trig, HIGH);
delayMicroseconds(10);
digitalWrite(trig, LOW);
duration = pulseIn(echo, HIGH);
dist= duration*0.034/2;
  Serial.println(dist);
}

void rack() {
  if(dist >= obstkl)
    return;
  unsigned long tm = millis();
  digitalWrite(rack1, LOW);
  do {
    ultrasonic();
  } while(dist < obstkl);
  tm = millis() - tm;
  digitalWrite(rack1, HIGH);
  digitalWrite(rack2, LOW);
  delay(tm);
  digitalWrite(rack1, HIGH);
  digitalWrite(rack2, HIGH);
}
