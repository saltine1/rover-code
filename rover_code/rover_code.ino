#include <Wire.h>;
#include <Servo.h>;
#include <Math.h>
#include "Adafruit_VL53L0X.h"
#include <basicMPU6050.h> 

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
int Lservo = 28; // left servo
int Rservo = 44; // right servo
int Cservo = 45; // center servo

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
int leftENA = 4;
int leftENB = 5;

// Create instance
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
  Serial.begin(9600);
  // sensor pin setups
  pinMode(echoPin1, INPUT);  //echo front 
  pinMode(trigPin1, OUTPUT); //trig front
  
  pinMode(echoPin2, INPUT); // echo left
  pinMode(echoPin2, OUTPUT); // trig left
  
  pinMode(2, INPUT); // echo right
  pinMode(5, OUTPUT); // trig right
  
  pinMode(12, INPUT); // echo arm
  pinMode(10, OUTPUT); // trig arm
  
  // motor pin setups
  pinMode(22, OUTPUT); // front left
  pinMode(23, OUTPUT); // front left
  pinMode(24, OUTPUT); // front right
  pinMode(25, OUTPUT); // front right
  
  pinMode(2, OUTPUT); // back left
  pinMode(3, OUTPUT); // back left
  pinMode(5, OUTPUT); // back right
  pinMode(6, OUTPUT); // back right

  pinMode(26, OUTPUT); // front left speed
  pinMode(27, OUTPUT); // front right speed

  pinMode(4, OUTPUT); // back left speed
  pinMode(7, OUTPUT); // back right speed

  // Set registers - Always required
  imu.setup();

  // Initial calibration of gyro
  imu.setBias();

  set_timer();

   Serial.begin(115200);

  delay(500);

  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }
  
  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  // power 
  Serial.println(F("VL53L0X API Simple Ranging example\n\n")); 

  radar_servo.attach(radar_servo_pin);
  radar_servo.write(theta);
  delay(5000);
}

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
    digitalWrite(frontIn1, HIGH);
    digitalWrite(frontIn2, LOW);
    digitalWrite(backIn1, HIGH);
    digitalWrite(frontIn2, LOW);
  }
  else{
    digitalWrite(frontIn1, LOW);
    digitalWrite(frontIn2, HIGH);
    digitalWrite(backIn1, LOW);
    digitalWrite(backIn2, HIGH);
    
  }
    if (right > 0){
    digitalWrite(frontIn3, HIGH);
    digitalWrite(frontIn4, LOW);
    digitalWrite(backIn3, HIGH);
    digitalWrite(frontIn4, LOW);
  }
  else{
    digitalWrite(frontIn3, LOW);
    digitalWrite(frontIn4, HIGH);
    digitalWrite(backIn3, LOW);
    digitalWrite(backIn4, HIGH);
  }
  analogWrite(frontENA, int(abs(double(left))/1000*255));
  analogWrite(backENA, int(abs(double(left))/1000*255));
  analogWrite(frontENB, int(abs(double(right))/1000*255));
  analogWrite(backENB, int(abs(double(right))/1000*255));
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

void loop() {
  // Part 1: Obstacle Course
  // First, measure the distance from obstacles using ultrasonic sensors as it goes forwards
  trigOnOff(23, trigPin1); // front
  trigOnOff(25, trigPin2); // left
  trigOnOff(27, trigPin3); // right
  trigOnOff(29, trigPin4); // arm
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
//  //conversions
//  in1 = (duration1/2)/74; // front
//  in2 = (duration2/2)/74; // left
//  in3 = (duration3/2)/74; // right
//  in4 = (duration4/2)/74; // arm
//  analogWrite(26, 100); // front ENA pin
//  analogWrite(27, 100); // front ENB pin
//  analogWrite(47, 100); // back ENA pin
//  analogWrite(48, 100); // back ENB pin

  Serial.print(String(ultrasonicDistance(0)) + " ");
  Serial.print(String(ultrasonicDistance(1)) + " ");
  Serial.print(String(ultrasonicDistance(2)) + " ");
  Serial.println(String(ultrasonicDistance(3)) + " ");
  delay(500);

  // Update gyro calibration
  imu.updateBias();

  update_gyro();

  Serial.println(get_gyro());
//  Serial.println(get_dt());

  set_timer();

  int d = lazer_measure();

  for(int i=0; i<1 ;i++){
  update_radar();
  polar_to_cartesian();
  Serial.println();
  delay(100);
  
//  Move(500, 500);
//  delay(2000);
//  Move(-500, 500);
//  delay(2000);
//  Move(500, -500);
//  delay(2000);
//  Move(-500, -500);
//  delay(2000);
//  
  // WIP BELOW
  
  // Use if/else statements to determine if there is an obstacle
//  if (in <= ___){
//    // WIP
//  }
//  else 
  

}
