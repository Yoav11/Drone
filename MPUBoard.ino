/*
 
  Hardware: GY-521 MPU-6050

  this code returns an approximation of the angle in degres in which the drone is 
  being orientated towards the X and Y axis.
  
*/

#include<Wire.h>
const int MPU_addr = 0x68; // I2C address of the MPU-6050

// When axis reads are storted in a list, the order is: X,Y,Z
float rawAcc[3];
float gAcc[3];
float accAngle[2];

float rawGyro[2];
float gyroAngle[2];

float rad_to_deg = 180/3.141592654;
float currentTime, previousTime, elapsedTime;
float totalAngle[2];


void setupMPU() {
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(250000);
}

void loopMPU() {

  //Keeps track of time elapsed between loop iterations, required for gyro angle formula
  previousTime = currentTime;
  currentTime = millis();
  elapsedTime = (currentTime - previousTime)/1000;

  
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 6, true);
  
  for(int i = 0; i <= 2; i++){
    rawAcc[i] = Wire.read() << 8 | Wire.read(); //reads two registers for each read, low and high bits.
    gAcc[i] = rawAcc[i]/16384; //converts raw data into units of g
  }
  
  accAngle[0] = atan((gAcc[1]/16384.0)/sqrt(pow((gAcc[0]/16384.0),2) + pow((gAcc[2]/16384.0),2)))*rad_to_deg; 
  accAngle[1] = atan(-1*(gAcc[0]/16384.0)/sqrt(pow((gAcc[1]/16384.0),2) + pow((gAcc[2]/16384.0),2)))*rad_to_deg; 

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x43);  // starting with register 0x43 (GYRO_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 4, true);
   
  for(int i = 0; i <= 1; i++){
    rawGyro[i] = Wire.read() << 8 | Wire.read();
    gyroAngle[i] = rawGyro[i]/131;  
  }

  //complimentary filter used to reduce noise and provide accurate angle reads
  totalAngle[0] = 0.98 *(totalAngle[0] + gyroAngle[0]*elapsedTime) + 0.02*accAngle[0];  
  totalAngle[1] = 0.98 *(totalAngle[1] + gyroAngle[1]*elapsedTime) + 0.02*accAngle[1];

  //debugging
  Serial.print("X:  ");Serial.print(totalAngle[0]);
  Serial.print(" |  Y:  ");Serial.println(totalAngle[1]);
}



