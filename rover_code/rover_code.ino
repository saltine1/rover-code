  
#include <Wire.h>;
#include <Servo.h>;
#include <Math.h>
#include "Adafruit_VL53L0X.h"
#include <basicMPU6050.h> 

// sensor var setups
int FLechoPin = 26; // front left sensor
int FLtrigPin = 27;

int LechoPin = 10; // left sensor
int LtrigPin = 11;

int RechoPin = 8; // right sensor
int RtrigPin = 9;

int FRechoPin = 48; // front right sensor
int FRtrigPin = 49;

int FLduration, Lduration, Rduration, FRduration, FLinches, Linches, Rinches, FRinches;

// servo var setups
int LservoPin = 3; // left servo
int RservoPin = 2; // right servo
int CservoPin = 12; // center servo
Servo Lservo;
Servo Rservo;
Servo Cservo;

// motor var setups
int rightIn1 = 49; //front
int rightIn2 = 50; // front
int rightIn3 = 51; // back
int rightIn4 = 52; // back
int rightENA = 4; // front
int rightENB = 5; // back

int leftIn1 = 22; // front
int leftIn2 = 23; // front
int leftIn3 = 24; // back
int leftIn4 = 25; // back
int leftENA = 6; // front
int leftENB = 7; // back

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

void setup() {
  Serial.begin(9600);
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
  //imu.setup();

  // Initial calibration of gyro
  //imu.setBias();
//
//  set_timer();
//  Serial.println("done!");
//
//  // wait until serial port opens for native USB devices
////  while (! Serial) {
////    delay(1);
////  }
//  
//  Serial.println("Adafruit VL53L0X test");
//  if (!lox.begin()) {
//    Serial.println(F("Failed to boot VL53L0X"));
//    Serial.println(F("Proceeding without laser sensor"));
//  }
//
//  set_claw(0);
//  
//  delay(5000);
//
//  Serial.print("Setup complete");
}

//void trigOnOff(int trigPinNum) {
//  digitalWrite(trigPinNum, LOW); // trig off
//  delayMicroseconds(5);
//  digitalWrite(trigPinNum, HIGH); // trig on
//  delayMicroseconds(10);
//  digitalWrite(trigPinNum, LOW);
//}

void Move(int left, int right){
  Serial.println("moving");
  if (left < 0){
    digitalWrite(leftIn1, LOW);
    digitalWrite(leftIn2, HIGH);
    digitalWrite(leftIn3, LOW);
    digitalWrite(leftIn4, HIGH);
  }
  else{
    digitalWrite(leftIn1, HIGH);
    digitalWrite(leftIn2, LOW);
    digitalWrite(leftIn3, HIGH);
    digitalWrite(leftIn4, LOW);
    
  }
    if (right < 0){
     digitalWrite(rightIn1, LOW);
    digitalWrite(rightIn2, HIGH);
    digitalWrite(rightIn3, LOW);
    digitalWrite(rightIn4, HIGH);
  }
  else{
    digitalWrite(rightIn1, HIGH);
    digitalWrite(rightIn2, LOW);
    digitalWrite(rightIn3, HIGH);
    digitalWrite(rightIn4, LOW);
  }
  analogWrite(leftENA, int(abs(double(left))/1000*255));
  analogWrite(leftENB, int(abs(double(left))/1000*255));
  analogWrite(rightENA, int(abs(double(right))/1000*255));
  analogWrite(rightENB, int(abs(double(right))/1000*255));
}

int ports[] = {LechoPin, FLechoPin, FRechoPin, RechoPin};

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

int theta = 90;
boolean radar_pos = true;

int data[181];
int point[2];

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

void measure_dir(int theta){
  Cservo.write(theta);
  delay(10);
  data[theta] = lazer_measure();
  
  return;
}

void update_radar(){
  if(radar_pos){
    theta++;
  }else{
    theta--;  
  }

  measure_dir(theta);
  
//  if(theta == 180 && radar_pos){
//    radar_pos = false;
//    Serial.println("c");
//    Serial.println("c");
//    }
//
//  if(theta == 0 && !radar_pos){
//    radar_pos = true;
//    Serial.println("c");
//    Serial.println("c");
//    }
  
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

void set_claw(int ang){
  // 0 for stright forward
  // 90 for open
  // -90 for closed;
  int left = 90 - ang;
  int right = ang + 90;

  Lservo.write(left);
  Rservo.write(right);

//  printf("claw output: %d %d",left, right);

//  Serial.print(String(left) + " " + String(right));
//}

//boolean turnStarted = false;
//double tarAng;
//
//double ang_prevError = 0;
//
//const double kp = 0.1;
//const double kd = 0.05;
//
//boolean turn(int deg){
//  if(!turnStarted){
//      tarAng = get_gyro() + deg;
//    }
//
//  double error = get_gyro() - tarAng;
//
//  double de = error - ang_prevError;
//
//  double pid = kp * error + kd * de;
//
//  Move(-pid, pid);
//
//  if(abs(de) < 0.5){
//    turnStarted = false;
//    return true;
//  }else{
//    return false;
//  }
//
////  Serial.printf()
//  
//}
//
//void testing_periodic(){
  
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
//
//  Move(500, 500);
//  delay(2000);
//  Move(100, 500);
//  delay(2000);
//  Move(500, 100);
//  delay(2000);
//  Move(-500, -500);
//  delay(2000);
//
//  Serial.println("go");
//
//  set_claw(90);
//  delay(1000);
//
//  set_claw(45);
//  delay(1000);
// }


//int claw_ang = 0;
//
//// serial read var setup
//char controlInput = 0;

//void keyboard_control(){
//
//  controlInput = Serial.read();
//  
//  if (controlInput == 'w'){
//   Move(500, 500);
//   delay(100); 
//  }
//  else if (controlInput == 's'){
//    Move(-500, -500);
//   delay(100); 
//  }
//  else if (controlInput == 'a'){
//    Move(-500, 500);
//   delay(100); 
//  }
//  else if (controlInput == 'd'){
//    Move(500, -500);
//   delay(100); 
//  }
//  else if (controlInput == 'j'){
//    //claw open
//    claw_ang ++; 
//    set_claw(claw_ang);
//  }
//  else if (controlInput == 'k'){
//    // claw close;
//    claw_ang ++; 
//    set_claw(claw_ang);
//  }else{
//    Move(0, 0);
//  }
//
//  Serial.println(controlInput);
//
//}

//void ultrasonic_logic(){
//  // Measure the distance from obstacles using ultrasonic sensors as it goes forwards
//  trigOnOff(FRtrigPin); // front right
//  trigOnOff(LtrigPin); // left
//  trigOnOff(RtrigPin); // right
//  trigOnOff(FLtrigPin); // front left
//  
//  pinMode(FLechoPin, INPUT); //recieve signal echo
//  FLduration = pulseIn(FLechoPin , HIGH); //front left sensor
//  pinMode(LechoPin, INPUT);
//  Lduration = pulseIn(LechoPin, HIGH); // left sensor
//  pinMode(RechoPin, INPUT);
//  Rduration = pulseIn(RechoPin, HIGH); // right sensor
//  pinMode(FRechoPin, INPUT);
//  FRduration= pulseIn(FRechoPin, HIGH); // front right sensor
//
//  // convert duration to inches
//  FLinches = (FLduration/2)/74; // front left
//  Linches = (Linches/2)/74; // left
//  Rinches = (Rinches/2)/74; // right
//  FRinches = (FRinches/2)/74; // front right
//
//  }
////  
//void routine_periodic(){
//  update_gyro();
//  set_timer();
//  int d = lazer_measure();
//
//  for(int i=0; i<1 ;i++){
//  update_radar();
//  polar_to_cartesian();
////  Serial.println();
////  delay(100);
//  }
//}

int state = 1;

void competition_logic(){

  switch(state){

    case 0:
    //wait at the start
      Move(0, 0);

    case 1:
    //locate obstical
      
    case 2:
    //avoid obstical
      if(turn(90)){
          state = 0;
        }
    case 3:
    //find object

    case 4:
    //goto object

    case 5:
    //move into circle

    caes 6:
    //stop at circle

    default:
      Move(0, 0);
    
  boolean courseEnd = false;
  while (FLinches >= 5 && FRinches >= 5){
    Move(500,500);
  }
  if (FLinches < 5 && FRinches < 5){
    if (Rinches <= Linches){
      Move(250, 500);
      delay(2000);
    }
    else{
      Move(500,250);
    }
    while (Linches < 4){
      Move(500,500);
    }
    if (Linches >= 4){
      Move(250, 500);
      delay(2000);
    }
    else if (Linches < 4 && FLinches >= 5 && FRinches >= 5){
      // open claw
      courseEnd = true;
    }
    if (courseEnd){
      Serial.println("wahoo");
    }
  }

  
}

void loop(){
//  routine_periodic();
//  testing_periodic();
//
////  competition_logic();
//
////  keyboard_control();
  Move(500, 500);
  
}
