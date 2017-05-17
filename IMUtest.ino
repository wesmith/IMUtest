/* 

IMUtest.ino

Copyright (c) 2017 Warren E Smith  smiwarsky@gmail.com

Please see the MIT License in the project root directory for specifics.  

*/

#include <Wire.h>
#include "IMU.h" 

// arduino I2C slave address: ensure no conflict with LSM303 or L3G addresses
const byte slaveAdd = 0x44; 
int registerAddr;
int data[12];      // I2C data registers 0x00 to 0x0b

// speed settings
int  msecPerCycle = 50; 
long baud = 115200; // typically 9600, 57600, 115200

// generally have only one of the following set to TRUE, depending upon task
bool PRINTSCREEN  = true;   // compare gyro fusion 'on' to gyro fusion 'off', 
                            // user-friendly print of roll, pitch, heading
bool RPH          = false;  // roll, pitch, heading, formatted for visual python
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
  
  Wire.begin(slaveAdd);
  Wire.onReceive(receiveRegister);
  Wire.onRequest(respondData);
  // I2C clock freq; don't set this register on DUE: there are currently issues
  TWBR=100000L; 
  
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
 
  if (PRINTSCREEN || RPH  || RPH2 || DEVTYPE) {
    withGyro.getRollPitchHeading();  
    noGyro.getRollPitchHeading();               
    printResults();
  }
  
  delay(msecPerCycle);
}


void receiveRegister(int x) {
  registerAddr = Wire.read();
}


void respondData(){
  byte dataValue = 0x00;  // default value
  if ((registerAddr >= 0x00) && (registerAddr < 0x0b)) {
    dataValue = data[registerAddr]; 
  }
  Wire.write(dataValue);
}


void printResults() {

  if (PRINTSCREEN) 
  {
   char report[120];   
   snprintf(report, sizeof(report), 
    "ROLL-NG:%+4d  ROLL:%+4d   PITCH-NG:%+4d  PITCH:%+4d   HEADING-NG:%+4d  HEADING: %+4d", 
    (int)noGyro.roll,  (int)withGyro.roll, 
    (int)noGyro.pitch, (int)withGyro.pitch, 
    (int)noGyro.head,  (int)withGyro.head);
   Serial.println(report);
  }
  
 if (RPH)
  {
    Serial.print("RPH ");
    Serial.print((int)withGyro.roll);
    Serial.print(",");
    Serial.print((int)withGyro.pitch);
    Serial.print(",");
    Serial.print((int)withGyro.head);
    Serial.print(",");    
    Serial.println();
  }

 // comparison of gyro fusion and no gyro, format for realtimeplotter
 if (RPH2)  
  {
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

 if (DEVTYPE) 
 {
   Serial.print("Accelerometer & Magnetometer: ");
   Serial.print(withGyro.getMagAccType());
   Serial.print("    Gyro: ");
   Serial.print(withGyro.getGyroType());
   Serial.println();
 }
}

