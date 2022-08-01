
#include <Wire.h>;
#include <Servo.h>;
#include <Math.h>
#include "Adafruit_VL53L0X.h"
#include <basicMPU6050.h> 

// serial read var setup
int controlInput = 0;

// sensor var setups
int FLechoPin = 26; // front left sensor
int FLtrigPin = 27;

int LechoPin = 8; // left sensor
int LtrigPin = 9;

int RechoPin = 49; // right sensor
int RtrigPin = 48;

int FRechoPin = 46; // front right sensor
int FRtrigPin = 47;

// servo var setups
int LservoPin = 28; // left servo
int RservoPin = 44; // right servo
int CservoPin = 45; // center servo
Servo Lservo;
Servo Rservo;
Servo Cservo;

long duration1, duration2, duration3, duration4, in1, in2, in3, in4;

// motor var setups
int rightIn1 = 22; //front
int rightIn2 = 23; // front
int rightIn3 = 24; // back
int rightIn4 = 25; // back
int rightENA = 2; // front
int rightENB = 3; // back

int leftIn1 = 50; // front
int leftIn2 = 51; // front
int leftIn3 = 52; // back
int leftIn4 = 53; // back
int leftENA = 4; // front
int leftENB = 5; // back

// Create gyro instance
basicMPU6050<> imu;

double cur_gyro = 0;
int t_prev;

void set_timer(){
  t_prev = millis();
  }

int get_dt(){
  return millis() - t_prev;
  }

void reset_gyro(){
  cur_gyro = 0;
  }

void update_gyro(){
  imu.updateBias();
  cur_gyro += 0.001 * get_dt() * imu.gz();
  }

double get_gyro(){
  return cur_gyro;
  }

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
Servo radar_servo;

const int radar_servo_pin = 3;

int theta = 90;
boolean radar_pos = true;

int data[2];
int point[2];


void setup() {
  Serial.begin(115200);
  Serial.println("Starting setup");
  // sensor pin setups
  pinMode(FLechoPin, INPUT);  // echo front left
  pinMode(FLtrigPin, OUTPUT); // trig front right
  
  pinMode(LechoPin, INPUT); // echo left
  pinMode(LechoPin, OUTPUT); // trig left
  
  pinMode(RechoPin, INPUT); // echo right
  pinMode(RtrigPin, OUTPUT); // trig right
  
  pinMode(FRechoPin, INPUT); // echo front right
  pinMode(FRechoPin, OUTPUT); // trig front right
  
  // motor pin setups
  pinMode(leftIn1, OUTPUT); // front left
  pinMode(leftIn2, OUTPUT); // front left
  pinMode(leftIn3, OUTPUT); // back left
  pinMode(leftIn4, OUTPUT); // back left
  
  pinMode(rightIn1, OUTPUT); // front right
  pinMode(rightIn2, OUTPUT); // front right
  pinMode(rightIn3, OUTPUT); // back right
  pinMode(rightIn4, OUTPUT); // back right

  pinMode(leftENA, OUTPUT); // front left speed
  pinMode(leftENB, OUTPUT); // back left speed
  
  pinMode(rightENA, OUTPUT); // front right speed
  pinMode(rightENB, OUTPUT); // back right speed

  Serial.println("IMU setup");
  // setups for servos
  Rservo.attach(RservoPin);
  Lservo.attach(LservoPin);
  Cservo.attach(CservoPin);
  
  // Set registers - Always required
  imu.setup();

  // Initial calibration of gyro
  imu.setBias();

  set_timer();
  Serial.println("done!");

  // wait until serial port opens for native USB devices
//  while (! Serial) {
//    delay(1);
//  }
  
  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    Serial.println(F("Proceeding without laser sensor"));
  }
  
  // power 
  radar_servo.attach(radar_servo_pin);
  radar_servo.write(theta);
  delay(5000);

  Serial.print("Setup complete");
}

void trigOnOff(int trigPinNum, int trigPinNumVar) {
  digitalWrite(trigPinNum, LOW); // trig off
  delayMicroseconds(5);
  digitalWrite(trigPinNum, HIGH); // trig on
  delayMicroseconds(10);
  digitalWrite(trigPinNumVar, LOW);
}

void Move(int left, int right){
  if (left > 0){
    digitalWrite(leftIn1, HIGH);
    digitalWrite(leftIn2, LOW);
    digitalWrite(leftIn3, HIGH);
    digitalWrite(leftIn4, LOW);
  }
  else{
    digitalWrite(leftIn1, LOW);
    digitalWrite(leftIn2, HIGH);
    digitalWrite(leftIn3, LOW);
    digitalWrite(leftIn4, HIGH);
    
  }
    if (right > 0){
    digitalWrite(rightIn1, HIGH);
    digitalWrite(rightIn2, LOW);
    digitalWrite(rightIn3, HIGH);
    digitalWrite(rightIn4, LOW);
  }
  else{
    digitalWrite(rightIn1, LOW);
    digitalWrite(rightIn2, HIGH);
    digitalWrite(rightIn3, LOW);
    digitalWrite(rightIn4, HIGH);
  }
  analogWrite(leftENA, int(abs(double(left))/1000*255));
  analogWrite(leftENB, int(abs(double(left))/1000*255));
  analogWrite(rightENA, int(abs(double(right))/1000*255));
  analogWrite(rightENB, int(abs(double(right))/1000*255));
}

int ports[] = {22, 24, 26, 28};

double ultrasonicDistance(int N){
  int port = ports[N];
  digitalWrite(23, LOW); // trig off
  delayMicroseconds(5);
  digitalWrite(23, HIGH); // trig on
  delayMicroseconds(10);
  digitalWrite(23, LOW);
  int duration = pulseIn(port, HIGH);
  pinMode(port, INPUT);
  return double((duration/2)/74);
  
}

int lazer_measure(){
  VL53L0X_RangingMeasurementData_t measure;
    
//  Serial.print("Reading a measurement... ");
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
//    Serial.print("Distance (mm): "); 
//    Serial.println(measure.RangeMilliMeter);
  } else {
//    Serial.println(" out of range ");
      return 0;
  }
  return measure.RangeMilliMeter;
}

void update_radar(){
  if(radar_pos){
    theta++;
  }else{
    theta--;  
  }

  radar_servo.write(theta);
//  Serial.println(theta);

  if(theta == 180 && radar_pos){
    radar_pos = false;
    Serial.println("c");
    Serial.println("c");
    }

  if(theta == 0 && !radar_pos){
    radar_pos = true;
    Serial.println("c");
    Serial.println("c");
    }

  int op[2];
  data[0] = theta;
  data[1] = lazer_measure();
  Serial.print(String(point[0]) + " " + String(point[1]) + " ");
  
  return;
}

int polar_to_cartesian(){
  double theta = M_PI * data[0] / 180;
  int y = data[1] * sin(theta);
  int x = data[1] * cos(theta);
  int op[2];
  point[0] = x;
  point[1] = y;

  return op;
}

boolean turnStarted = false;
double tarAng;

double ang_prevError = 0;

const double kp = 0.1;
const double kd = 0.05;

boolean turn(int deg){
  if(!turnStarted){
      tarAng = get_gyro() + deg;
    }

  double error = get_gyro() - tarAng;

  double de = error - ang_prevError;

  double pid = kp * error + kd * de;

  Move(-pid, pid);

  if(abs(de) < 0.5){
    turnStarted = false;
    return true;
  }else{
    return false;
  }
  
}

void testing_periodic(){
  
//  //conversions
//  in1 = (duration1/2)/74; // front
//  in2 = (duration2/2)/74; // left
//  in3 = (duration3/2)/74; // right
//  in4 = (duration4/2)/74; // arm
//  analogWrite(26, 100); // front ENA pin
//  analogWrite(27, 100); // front ENB pin
//  analogWrite(47, 100); // back ENA pin
//  analogWrite(48, 100); // back ENB pin

//  Serial.print(String(ultrasonicDistance(0)) + " ");
//  Serial.print(String(ultrasonicDistance(1)) + " ");
//  Serial.print(String(ultrasonicDistance(2)) + " ");
//  Serial.println(String(ultrasonicDistance(3)) + " ");
//  delay(500);

//  Serial.println(get_gyro());
//  Serial.println(get_dt());

  Move(500, 500);
  delay(2000);
  Move(-500, 500);
  delay(2000);
  Move(500, -500);
  delay(2000);
  Move(-500, -500);
  delay(2000);

  Serial.println("go");

  if (controlInput == "w"){
   Move(500, 500);
   delay(2000); 
  }
  else if (controlInput == "s"){
    Move(-500, -500);
    delay(2000); 
  }
  else if (controlInput == "a"){
    Move(-500, 500);
    delay(2000);
  }
  else if (controlInput == "d"){
    Move(500, -500);
    delay(2000);
  }
  else if (controlInput == "q"){
    Rservo.write(90);
    Lservo.write(90);
  }
  else if (controlInput == "e"){
    Rservo.write();
    Lservo.write();
  }
  }

void ultrasonic_logic(){
  // Part 1: Obstacle Course
  // First, measure the distance from obstacles using ultrasonic sensors as it goes forwards
//  trigOnOff(23, trigPin1); // front
//  trigOnOff(25, trigPin2); // left
//  trigOnOff(27, trigPin3); // right
//  trigOnOff(29, trigPin4); // arm
//  
//  pinMode(8, INPUT); //recieve signal echo
//  duration1 = pulseIn(8, HIGH); //front sensor
//  pinMode(4, INPUT);
//  duration2 = pulseIn(4, HIGH); // left sensor
//  pinMode(2, INPUT);
//  duration3 = pulseIn(2, HIGH); // right sensor
//  pinMode(5, INPUT);
//  duration4= pulseIn(5, HIGH); // arm sensor
//
  }
  
void routine_periodic(){
  update_gyro();
  set_timer();
  int d = lazer_measure();

  for(int i=0; i<1 ;i++){
  update_radar();
  polar_to_cartesian();
  Serial.println();
  delay(100);
  }
}

void competition_logic(){
  int state = 0;

  switch(state){

    case 0:
      Move(0, 0);

    case 1:
      if(turn(90)){
          state = 0;
        }

    default:
      Move(0, 0);
    
  }
  
}

void loop() {
  routine_periodic();
  testing_periodic();

  competition_logic();
  
}
