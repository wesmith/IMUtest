/* 

IMUtest.ino

Copyright (c) 2017 Warren E Smith  smiwarsky@gmail.com

Please see the MIT License in the project root directory for specifics.  

*/

#include <Wire.h>
#include "IMU.h" 

// arduino I2C slave address: ensure no conflict with LSM303 or L3G addresses
//const byte slaveAdd = 0x44; 
int registerAddr;
int16_t data[12];      // I2C data registers 0x00 to 0x0b

// speed settings
int  msecPerCycle = 20; // 50 Hz
long baud = 115200; // typically 9600, 57600, 115200

// generally have only one of the following set to TRUE, depending upon task
bool I2C          = false;   // run in I2C slave mode
bool PRINTSCREEN  = false;  // compare gyro fusion 'on' to gyro fusion 'off', 
                            // user-friendly print of roll, pitch, heading
bool RPH          = true;   // roll, pitch, heading, formatted for displayIMU.py
bool VEC          = false;   // vectors and roll, pitch, heading for visual python
bool RPH2         = false;  // compare gyro fusion 'on' to gyro fusion 'off', 
                            // for Processing RealTimePlotter format
bool CALIBACC     = false;  // generate accelerometer calibration data
bool CALIBMAG     = false;  // generate magnetometer  calibration data
bool CALIB_TEST   = false;  // test robustness of calibration, once platform calibrated: 
                            // CALIBACC or CALIBMAC should be TRUE for this
bool DEVTYPE      = false;  // print out device type

IMU withGyro;  // with accelerometer/gyro fusion
IMU noGyro;    // just accelerometer


void setup() {

  Serial.begin(baud);

  Wire.begin();

  // the following code when Arduino is I2C slave
  //Wire.begin(slaveAdd);  
  //Wire.onReceive(receiveRegister);
  //Wire.onRequest(respondData);
  // I2C clock freq; don't set this register on DUE: there are currently issues
  //TWBR=100000L; 
  
  withGyro.init();
  noGyro.init();
  
  delay(100);
  
  // true means gyro is used, followed by alpha 
  // 0 < alpha < 1; 
  // alpha ~ 1 weights gyro heavily, alpha ~ 0 weights accelerometer heavily
  // if false, alpha is ignored, and no gyro data is used
  withGyro.setWeightsAndGyro(true, 0.95); 
  noGyro.setWeightsAndGyro(false, 0.95); // no gyro, alpha is ignored
  
  // mag, acc, calibration methods take three offsets, three scales
  // 5/11/17 calibration, average of three 
  withGyro.setCalibrateMag(  -52, -158,   39,  1.00, 1.02, 1.16); 
  noGyro.setCalibrateMag(  -52, -158,   39,  1.00, 1.02, 1.16); 
  
  // 5/8/17 calibration values
  withGyro.setCalibrateAcc(-1020,  -84, -700,  1.07, 1.07, 1.00); 
  noGyro.setCalibrateAcc(-1020,  -84, -700,  1.07, 1.07, 1.00); 
  
  // new gyro calibration here: offset, noise value, calibrated rate for each axis
  //withGyro.setCalibrateGyro(off, off, off,  noise, noise, noise,  scal, scal, scal);

  // NOTE: the rollAxis, rollReference vectors must be orthogonal for meaningful 
  // results; they can be non-normal: they are normalized internally
  // WS 5/3/17: 'normal' chip settings, consistent with Pololu markings
  // x chip axis is 'x', y chip axis is 'y'
  // x axis is rollAxis, y axis is rollReference
  withGyro.setIMUAxes  (1, 0, 0, 0, 1, 0); 
  noGyro.setIMUAxes  (1, 0, 0, 0, 1, 0); 

  // 5/4/17 experiment: this worked, with correct signs for angles
  // -y chip axis is rollAxis 'x', x chip axis is rollReference 'y'
  //withGyro.setIMUAxes      (0, -1, 0, 1, 0, 0); 

  // 5/4/17 experiment: this worked, with correct signs for angles
  // z chip axis is rollAxis 'x', x chip axis is rollReference 'y'
  //withGyro.setIMUAxes      (0, 0, 1, 1, 0, 0);
}


void loop() {

  // do one or the other calibrations, not both together
  if (CALIBACC) {withGyro.doCalibrateAcc(CALIB_TEST);}
  if (CALIBMAG) {withGyro.doCalibrateMag(CALIB_TEST);}
 
  if (PRINTSCREEN || RPH  || RPH2 || DEVTYPE || I2C || VEC) {
    withGyro.getRollPitchHeading();  
    noGyro.getRollPitchHeading();               
    printResults();
  }
  
  delay(msecPerCycle);
}


void fillI2CRegisters() {
  data[ 0] = (int16_t)withGyro.roll  >>   8;  // MSB
  data[ 1] = (int16_t)withGyro.roll  & 0xFF;  // LSB
  data[ 2] = (int16_t)withGyro.pitch >>   8;
  data[ 3] = (int16_t)withGyro.pitch & 0xFF;
  data[ 4] = (int16_t)withGyro.head  >>   8;
  data[ 5] = (int16_t)withGyro.head  & 0xFF; 
  data[ 6] = (int16_t)noGyro.roll    >>   8;
  data[ 7] = (int16_t)noGyro.roll    & 0xFF;
  data[ 8] = (int16_t)noGyro.pitch   >>   8;
  data[ 9] = (int16_t)noGyro.pitch   & 0xFF;    
  data[10] = (int16_t)noGyro.head    >>   8;
  data[11] = (int16_t)noGyro.head    & 0xFF;
}


void receiveRegister(int x) {
  registerAddr = Wire.read();
}

void respondData(){  // debug
  Wire.write(registerAddr);  
}
/*
void respondData(){
  char dataValue = 0x01;  // default value, debug value 5
  if ((registerAddr >= 0x00) && (registerAddr < 0x0c)) {
    dataValue = data[registerAddr]; 
  }
  Wire.write(dataValue);
}
*/

void printResults() {

  if (I2C) {
    fillI2CRegisters();
    // debug follows: this works
    char report[120];
    snprintf(report, sizeof(report),
    "ROLL:%+4d  data[0]:%+4d  data[1]:%+4d  combined:%+4d", 
    (int)withGyro.roll, data[0], data[1], data[0]<<8 | data[1]);
    Serial.println(report);
  }

  if (PRINTSCREEN) {
   char report[120];   
   snprintf(report, sizeof(report), 
    "ROLL-NG:%+4d  ROLL:%+4d   PITCH-NG:%+4d  PITCH:%+4d   HEADING-NG:%+4d  HEADING: %+4d", 
    (int)noGyro.roll,  (int)withGyro.roll, 
    (int)noGyro.pitch, (int)withGyro.pitch, 
    (int)noGyro.head,  (int)withGyro.head);
   Serial.println(report);
  }

 // display of roll, pitch, heading formatted for displayIMU.py
 if (RPH) {
    Serial.print("RPH "); 
    Serial.print((int)withGyro.roll);
    Serial.print(",");
    Serial.print((int)withGyro.pitch);
    Serial.print(",");
    Serial.print((int)withGyro.head);
    Serial.print(",");    
    Serial.println();
  }

 if (VEC) {
    Serial.print("VEC");
    Serial.print(noGyro.acc_norm.x);
    Serial.print(",");
    Serial.print(noGyro.acc_norm.y);
    Serial.print(",");
    Serial.print(noGyro.acc_norm.z);
    Serial.print(",");
    Serial.print(noGyro.mag_norm.x);
    Serial.print(",");
    Serial.print(noGyro.mag_norm.y);
    Serial.print(",");
    Serial.print(noGyro.mag_norm.z);
    Serial.print(",");
    Serial.print(withGyro.acc_norm.x);
    Serial.print(",");
    Serial.print(withGyro.acc_norm.y);
    Serial.print(",");
    Serial.print(withGyro.acc_norm.z);
    Serial.print(","); 
    Serial.print(withGyro.roll);
    Serial.print(",");
    Serial.print(withGyro.pitch);
    Serial.print(",");
    Serial.print(withGyro.head);
    Serial.print(",");    
    Serial.println();
 }

 // comparison of gyro fusion and no gyro, format for realtimeplotter
 if (RPH2) {
    //Serial.print("RPH2 ");
    Serial.print((int)noGyro.roll);
    Serial.print(" ");
    Serial.print((int)withGyro.roll);
    Serial.print(" ");
    Serial.print((int)noGyro.pitch);
    Serial.print(" "); 
    Serial.print((int)withGyro.pitch);
    Serial.print(" ");
    Serial.print((int)noGyro.head);
    Serial.print(" ");
    Serial.print((int)withGyro.head);
    //Serial.print(" ");   
    Serial.print("\r"); 
    Serial.println();
  }

 if (DEVTYPE) {
   Serial.print("Accelerometer & Magnetometer: ");
   Serial.print(withGyro.getMagAccType());
   Serial.print("    Gyro: ");
   Serial.print(withGyro.getGyroType());
   Serial.println();
 }
}

