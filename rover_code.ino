void setup() {
  Serial.begin (10000);
  
  pinMode(8, INPUT);  //echo front 
  pinMode(9, OUTPUT); //trig front
  
  pinMode(4, INPUT); // echo left
  pinMode(11, OUTPUT); // echo left
  
  pinMode(2, INPUT); // echo right
  pinMode(5, OUTPUT); // trig right
  
  pinMode(12, INPUT); // echo arm
  pinMode(10, OUTPUT); // trig arm

}

// Setup for ultrasonic sensors
int echoPin = 8; // front sensor
int trigPin = 9;

int echoPin2 = 4; // left sensor
int trigPin2 = 11;

int echoPin3 = 2; // right sensor
int trigPin3 = 5;

int echoPin4 = 12; // arm sensor
int trigPin4 = 10;

long duration1, duration2, duration3, duration4, in1, in2, in3, in4;
  

void loop() {
  // Part 1: Obstacle Course
  // First, measure the distance from obstacles using ultrasonic sensors as it goes forwards
  
    digitalWrite(9, LOW); //trig off front
    delayMicroseconds(5);
    digitalWrite(9, HIGH);//trig on front
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    digitalWrite(11, LOW); //trig off left
    delayMicroseconds(5);
    digitalWrite(11, HIGH);//trig on left
    delayMicroseconds(10);
    digitalWrite(trigPin2, LOW);

    digitalWrite(5, LOW); //trig off right
    delayMicroseconds(5);
    digitalWrite(5, HIGH);//trig on right
    delayMicroseconds(10);
    digitalWrite(trigPin3, LOW);

    digitalWrite(10, LOW); //trig off arm
    delayMicroseconds(5);
    digitalWrite(10, HIGH);//trig on arm
    delayMicroseconds(10);
    digitalWrite(trigPin4, LOW);
  
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
    if (in <= 10){
      
    }

  // Use if/else statements to determine if there is a wall
  // Use if/else statements to determine if the object is on the right or left
  // 
  
}
