#include <Wire.h>;
#include <Servo.h>;
#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
Servo radar_servo;

const int radar_servo_pin = 3;

int theta = 0;
boolean radar_pos = true;

int data[2];
int point[2];

void setup() {
  Serial.begin(9600);

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
  radar_servo.write(0);
  delay(1000);
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
    }

  if(theta == 0 && !radar_pos){
    radar_pos = true;
    }

  int op[2];
  data[0] = theta;
  data[1] = lazer_measure();
  
  return op;
}

int polar_to_cartesian(){
  int x = data[1] * cos(data[0]);
  int y = data[1] * sin(data[0]);
  int op[2];
  point[0] = x;
  point[1] = y;

  return op;
}

  void loop() {

  int d = lazer_measure();
  
  update_radar();

  polar_to_cartesian();

//  Serial.println(d);

//  Serial.println(String(data[0]) + " " + String(data[1]));
  Serial.println(String(point[0]) + " " + String(point[1]));

//  delay(2000);
}
