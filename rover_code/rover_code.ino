void setup() {
  Serial.begin (10000);
  // sensor pin setups
  pinMode(8, INPUT);  //echo front 
  pinMode(9, OUTPUT); //trig front
  
  pinMode(4, INPUT); // echo left
  pinMode(11, OUTPUT); // trig left
  
  pinMode(2, INPUT); // echo right
  pinMode(5, OUTPUT); // trig right
  
  pinMode(12, INPUT); // echo arm
  pinMode(10, OUTPUT); // trig arm
  
  // motor pin setups
  pinMode(22, OUTPUT); // front left
  pinMode(23, OUTPUT); // front left
  pinMode(24, OUTPUT); // front right
  pinMode(25, OUTPUT); // front right
  
  pinMode(50, OUTPUT); // back left
  pinMode(51, OUTPUT); // back left
  pinMode(52, OUTPUT); // back right
  pinMode(53, OUTPUT); // back right
}

// sensor var setups
  int echoPin1 = 8; // front sensor
  int trigPin1 = 9;

  int echoPin2 = 4; // left sensor
  int trigPin2 = 11;

  int echoPin3 = 2; // right sensor
  int trigPin3 = 5;

  int echoPin4 = 12; // arm sensor
  int trigPin4 = 10;

  long duration1, duration2, duration3, duration4, in1, in2, in3, in4;
  
  // motor var setups
  int frontIn1 = 22; // In 1 and 2 are for the left motors, while In 3 and 4 are for the right motors
  int frontIn2 = 23;
  int frontIn3 = 24;
  int frontIn4 = 25;

  int backIn1 = 50;
  int backIn2 = 51;
  int backIn3 = 52;
  int backIn4 = 53;

void trigOnOff(int trigPinNum, int trigPinNumVar) {
  digitalWrite(trigPinNum, LOW); // trig off
  delayMicroseconds(5);
  digitalWrite(trigPinNum, HIGH); // trig on
  delayMicroseconds(10);
  return digitalWrite(trigPinNumVar, LOW);
}
  

void loop() {
  // Part 1: Obstacle Course
  // First, measure the distance from obstacles using ultrasonic sensors as it goes forwards
  trigOnOff(9, trigPin1); // front
  trigOnOff(11, trigPin2); // left
  trigOnOff(5, trigPin3); // right
  trigOnOff(10, trigPin4); // arm
  
  pinMode(8, INPUT); //recieve signal echo
  duration1 = pulseIn(8, HIGH); //front sensor
  pinMode(4, INPUT);
  duration2 = pulseIn(4, HIGH); // left sensor
  pinMode(2, INPUT);
  duration3 = pulseIn(2, HIGH); // right sensor
  pinMode(5, INPUT);
  duration4= pulseIn(5, HIGH); // arm sensor

  //conversions
  in1 = (duration1/2)/74; // front
  in2 = (duration2/2)/74; // left
  in3 = (duration3/2)/74; //right
  in4 = (duration4/2)/74; // arm
  
  // Use if/else statements to determine if there is an obstacle
  // If it's closer
  // Use if/else statements to determine if there is a wall
  // Use if/else statements to determine if the object is on the right or left
  // 
  
}
