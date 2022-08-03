/*
 Get scaled and calibrated output of MPU6050
 */

#include <basicMPU6050.h> 

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

void setup() {
  // Set registers - Always required
  imu.setup();

  // Initial calibration of gyro
  imu.setBias();

  set_timer();
  
  // Start console
  Serial.begin(38400);
}

void loop() { 
  // Update gyro calibration 
  imu.updateBias();
  
//  // Gyro
//  Serial.print( imu.gx() );
//  Serial.print( " " );
//  Serial.print( imu.gy() );
//  Serial.print( " " );
//  Serial.print( imu.gz() );
//  Serial.print( "    " );  
//
//  // Temp
//  Serial.print( imu.temp() );
//  Serial.println(); 

  update_gyro();

  Serial.println(get_gyro());
//  Serial.println(get_dt());

  set_timer();
}
