#include <Wire.h>;
#include <Servo.h>;
#include <Math.h>
#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
Servo radar_servo;

const int radar_servo_pin = 45;

int theta = 90;
boolean radar_pos = true;

int data[2];
int point[2];

void setup() {
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

  int d = lazer_measure();

  for(int i=0; i<1 ;i++){
    update_radar();
    polar_to_cartesian();
    Serial.println();
    delay(100);
  }

//  Serial.println(d);

//  Serial.println(String(data[0]) + " " + String(data[1]));


//  delay(2000);
}
