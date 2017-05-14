/* 

IMU.cpp

Copyright (c) 2017 Warren E Smith  smiwarsky@gmail.com

Please see the MIT License in the project root directory for specifics.  

*/

#include "IMU.h"
#include <math.h>

IMU::IMU(void) {}

bool IMU::init()
{
  accmag.init();
  
  accmag.enableDefault();

  delay(500);   // needed?

  // FIX the below once working, to make more general
  // the following 30Hz speedup for LSM303 DLHC only:
  // speed up magnetometer rate from 7.5 Hz to 30 Hz, 5/3/17
  // set DO = 101 (30 Hz) (DO = 011 for 7.5 Hz)
  // from p. 37 of data sheet: 
  // CRA_REG_M: TEMP_EN 0 0 DO2 DO1 DO0 0 0 
  // = 000 101 00 = 00010100 = 0x14
  //accmag.writeMagReg(LSM303::CRA_REG_M, 0x14);  TURNED OFF AT PRESENT

  setCalibrateAcc(0, 0, 0, 1, 1, 1); // default offsets, scale factors
  setCalibrateMag(0, 0, 0, 1, 1, 1);

  gyro.init();

  delay(200);   // needed?
  
  // update the below settings to set them from IMU class in the future
  // 2000 dps full scale
  gyro.writeReg(L3G::CTRL_REG4, 0x20);
  
  // normal power mode, all axes enabled, 100 Hz
  gyro.writeReg(L3G::CTRL_REG1, 0x0F);
  
  // for 2000 deg full scale, 1 digit = 0.070 deg/sec
  setCalibrateGyro(0, 0, 0, 0, 0, 0, 0.07, 0.07, 0.07);

  setIMUAxes(1, 0, 0, 0, 1, 0);
  
  getEstAcc();  // initializes est_acc, est_mag vectors

  return true;
}


void IMU::setWeightsAndGyro(bool isGyroUsed, float aa, float mm)
{
  gyroUsed = isGyroUsed;
  alpha = gyroUsed ? aa : 0;
  alphaMag = mm;
  prevTime = millis();  // set initial time for gyro calculations
}


void IMU::setCalibrateAcc(int    xoff, int    yoff, int    zoff,
			  float xscal, float yscal, float zscal)
{
  a_off = { xoff,  yoff,  zoff};
  a_sca = {xscal, yscal, zscal};
}

void IMU::setCalibrateMag(int    xoff, int    yoff, int    zoff,
			  float xscal, float yscal, float zscal)
{
  m_off = { xoff,  yoff,  zoff};
  m_sca = {xscal, yscal, zscal};
}

void IMU::setCalibrateGyro(int    xoff,   int  yoff,   int  zoff,
			   float xnois, float ynois, float znois,
			   float xscal, float yscal, float zscal)
{
  g_off = { xoff,  yoff,  zoff};
  g_noi = {xnois, ynois, znois};
  g_sca = {xscal, yscal, zscal};
}


/*
The first three numbers define the rollAxis (the axis that defines the pitch
angle: usually (1, 0, 0)). The second three numbers define the rollReference 
(the axis that defines the roll angle: usually (0, 1, 0)).
NOTE that rollAxis and rollReference should be orthogonal or the IMU results
will be unreliable (orthogonality isn't checked here).
*/
void IMU::setIMUAxes(float x1, float y1, float z1, float x2, float y2, float z2)
{
  rollAxis      = {x1, y1, z1};
  rollReference = {x2, y2, z2};

  vector_normalize(&rollAxis);      // normalization needed for pitch calc
  vector_normalize(&rollReference); // normalization needed for roll  calc
}


// Reads accelerator, magnetometer, gyro inputs, and conditions the data
void IMU::read()
{
  accmag.readAcc();
  
  acc_norm = {(float)accmag.a.x,
	      (float)accmag.a.y,
	      (float)accmag.a.z};  // float to normalize
  // subtract offset and scale
  acc_norm.x -= a_off.x; acc_norm.y -= a_off.y; acc_norm.z -= a_off.z;
  acc_norm.x *= a_sca.x; acc_norm.y *= a_sca.y; acc_norm.z *= a_sca.z;

  // save off a test acceleration vector to validate calibration,
  // before normalization
  acc_test = {acc_norm.x, acc_norm.y, acc_norm.z};
  
  vector_normalize(&acc_norm);

  accmag.readMag();

  mag_norm = {(float)accmag.m.x,
	      (float)accmag.m.y,
	      (float)accmag.m.z};  // float to normalize
  // subtract offset and scale
  mag_norm.x -= m_off.x; mag_norm.y -= m_off.y; mag_norm.z -= m_off.z;
  mag_norm.x *= m_sca.x; mag_norm.y *= m_sca.y; mag_norm.z *= m_sca.z;

  mag_test = {mag_norm.x, mag_norm.y, mag_norm.z}; 
  
  vector_normalize(&mag_norm);

  if (gyroUsed){
    gyro.read();
    omega = {(float)gyro.g.x, (float)gyro.g.y, (float)gyro.g.z};
    omega.x -= g_off.x; omega.y -= g_off.y; omega.z -= g_off.z;

    // ignore g_noi noise vector for now: will use later to ignore low values

    omega.x *= g_sca.x * DEG2RAD;
    omega.y *= g_sca.y * DEG2RAD;
    omega.z *= g_sca.z * DEG2RAD;

    // DO NOT NORMALIZE gyro vector: its magnitude and direction are essential
  }
}


// initialize estimated acc and mag vectors
void IMU::getEstAcc() 
{
  read();  // get acc, mag, gyro

  // if gyro not used, this is just a pass-through, and est_acc = acc_norm
  if (initAccMagVec || !gyroUsed) {
    
    est_acc.x = acc_norm.x;
    est_acc.y = acc_norm.y;
    est_acc.z = acc_norm.z;
    est_mag.x = mag_norm.x;
    est_mag.y = mag_norm.y;
    est_mag.z = mag_norm.z;
    
    initAccMagVec = false;  // initialization is finished
    return;
  }

  // calculate deltaT: current time - previous time, normalize to seconds
  long currTime = millis();
  float deltaT = ((float)currTime - prevTime)/1000; // normalize to seconds

  // form deltaVec = omega x est_acc; omega (angular velocity from gyro)
  // must be in radians/sec in this calculation
  LSM303::vector<float> deltaVec;
  vector_cross(&omega, &est_acc, &deltaVec); 

  // est_acc = est_acc - deltaVec(n) * deltaT
  est_acc.x -=  deltaVec.x * deltaT;
  est_acc.y -=  deltaVec.y * deltaT;
  est_acc.z -=  deltaVec.z * deltaT;

  // normalize est_acc
  vector_normalize(&est_acc);

  // est_acc = alpha * est_acc + (1 - alpha) * acc_norm  : complementary approach
  est_acc.x = alpha * est_acc.x + (1 - alpha) * acc_norm.x;
  est_acc.y = alpha * est_acc.y + (1 - alpha) * acc_norm.y;
  est_acc.z = alpha * est_acc.z + (1 - alpha) * acc_norm.z;

  // normalize est_mag
  vector_normalize(&est_mag); 
  
  // est_mag calcs here: simple filter
  est_mag.x = alphaMag * est_mag.x + (1 - alphaMag) * mag_norm.x;
  est_mag.y = alphaMag * est_mag.y + (1 - alphaMag) * mag_norm.y;
  est_mag.z = alphaMag * est_mag.z + (1 - alphaMag) * mag_norm.z;

  // normalize vectors again (overkill?)
  vector_normalize(&est_acc);
  vector_normalize(&est_mag);  

  // update previous time to current time
  prevTime = currTime;
}


/*
Returns the angular difference in the horizontal plane between the
rollAxis vector and magnetic (NOT true) north, in degrees. This step uses
the updated accelerometer-gyro fused vector est_acc. 
*/
void IMU::getHeading(void) 
{
  LSM303::vector<float> E; // east
  LSM303::vector<float> N; // north

  vector_cross(&est_mag, &est_acc, &E); // east = mag vector x gravity (up)

  vector_normalize(&E); 
  
  vector_cross(&est_acc, &E, &N);        // north = gravity (up) x east
  vector_normalize(&N);
  
  // compute heading: using '-' sign to get z-axis convention consistent with x- and
  // y-axis conventions: i.e., looking along z-axis towards increasing z (consider
  // the observer is under the x-y plane, looking up) the CW direction is positive;
  // WARNING: with this convention, if z is up, positive heading will be
  // north-to-west, which may be counter-intuitive
  head = -atan2(vector_dot(&E, &rollAxis),
		vector_dot(&N, &rollAxis)) * RAD2DEG;
}


/*
- Pitch is defined as the angle between the rollAxis vector and the horizontal. 
- Horizontal is defined as perpendicular to the gravity (acceleration) vector.
  Normally the rollAxis vector will be {1, 0, 0}, representing the body x-axis 
  (i.e., the direction of the body's 'nose').
- Roll is the angle between the rollReference vector and the vector in the 
  horizontal plane that is perpendicular to the rollAxis vector. This maintains 
  roll independent of pitch. Normally the rollReference vector will be {0, 1, 0}, 
  representing the body y-axis (i.e., the direction of the body's 'left wing').
- The default setting is rollAxis = {1, 0, 0} and rollReference = {0, 1, 0}. 
*/
void IMU::getRollPitch(void) 
{
  // using sin() instead of cos() to have pitch zero when rollAxis is
  // perpendicular to gravity (i.e., when rollAxis is in the horizontal plane);
  // the minus sign is for the positive CW convention for the pitch angle, looking
  // along the pitch axis; the fused gyro/accel vector is used
  pitch = -asin( vector_dot( &est_acc, &rollAxis )) * RAD2DEG;
  
  // roll: is preserved as pitch changes, by normalizing the 'horizontal' vector
  LSM303::vector<float> horizontal; // vector in the horiz plane, perp to rollAxis
  vector_cross(&est_acc, &rollAxis, &horizontal);
  vector_normalize( &horizontal ); // potential problem when pitch is at poles
  roll = acos( vector_dot( &horizontal, &rollReference )) * RAD2DEG;
  // get the sign of the roll angle
  roll = vector_dot( &est_acc, &rollReference ) >= 0 ? roll: -roll; 
}


void IMU::getRollPitchHeading(void)
{
  getEstAcc();  // this calls read(), necessary before heading and roll, pitch
  getHeading();
  getRollPitch();
}



char* IMU::getMagAccType()
{
  char dtype[20];
  switch (accmag.getDeviceType())
    {
      case LSM303::device_DLH:
        snprintf(dtype, sizeof(dtype), "LSM303 DLH");
        break;
        
      case LSM303::device_DLM:
        snprintf(dtype, sizeof(dtype), "LSM303 DLM");
        break;

      case LSM303::device_DLHC:
        snprintf(dtype, sizeof(dtype), "LSM303 DLHC");
        break;
        
      case LSM303::device_D:
        snprintf(dtype, sizeof(dtype), "LSM303 D");
        break;
    }
  return dtype;
} 

char* IMU::getGyroType()
{
  char dtype[20];
  switch (gyro.getDeviceType())
    {
      case L3G::device_4200D:
        snprintf(dtype, sizeof(dtype), "L3G 4200D");
        break;
        
      case L3G::device_D20:
        snprintf(dtype, sizeof(dtype), "L3G D20");
        break;

      case L3G::device_D20H:
        snprintf(dtype, sizeof(dtype), "L3G D20H");
        break;
    }
  return dtype;
}


void IMU::getRawAccMagGyro() // for calibration
{
  accmag.readAcc();
  acc_raw = {accmag.a.x, accmag.a.y, accmag.a.z}; 
  mag_raw = {accmag.m.x, accmag.m.y, accmag.m.z};

  gyro.read();
  gyr_raw = {gyro.g.x, gyro.g.y, gyro.g.z};
}


// IMPORTANT: use this vector_normalize instead of
// LSM303::vector_normalize: the latter doesn't test for
// divide by zero
void IMU::vector_normalize(LSM303::vector<float> *a)
{
  double mag = sqrt(vector_dot(a, a));
  if (mag >= EPS)
    { a->x /= mag;  a->y /= mag;  a->z /= mag; }
  else
    { a->x = a->y = a->z = 0.; }
}

// stand-alone function to print magnetic-calibration info
void IMU::doCalibrateMag(bool TEST)  
{
  if (TEST) { // test robustness of calibration on normalized values
    read();   // creates 'test' values after offset and scaling
    minX = min(minX, (int)mag_test.x);
    minY = min(minY, (int)mag_test.y);
    minZ = min(minZ, (int)mag_test.z);
    maxX = max(maxX, (int)mag_test.x);
    maxY = max(maxY, (int)mag_test.y);
    maxZ = max(maxZ, (int)mag_test.z);
  }
  else {  // get calibration params from raw data
    accmag.readMag();
    minX = min(minX, accmag.m.x);
    minY = min(minY, accmag.m.y);
    minZ = min(minZ, accmag.m.z);
    maxX = max(maxX, accmag.m.x);
    maxY = max(maxY, accmag.m.y);
    maxZ = max(maxZ, accmag.m.z);
  }
  strncpy(txt, "MAG", 4); txt[4] = '\0';

  printCalibrate();
}


void IMU::doCalibrateAcc(bool TEST)
{
  if (TEST) { // test robustness of calibration on normalized values
    read();   // creates 'test' values after offset and scaling
    minX = min(minX, (int)acc_test.x);
    minY = min(minY, (int)acc_test.y);
    minZ = min(minZ, (int)acc_test.z);
    maxX = max(maxX, (int)acc_test.x);
    maxY = max(maxY, (int)acc_test.y);
    maxZ = max(maxZ, (int)acc_test.z);
  }
  else {  // get calibration params from raw data
    accmag.readAcc();
    minX = min(minX, accmag.a.x);
    minY = min(minY, accmag.a.y);
    minZ = min(minZ, accmag.a.z);
    maxX = max(maxX, accmag.a.x);
    maxY = max(maxY, accmag.a.y);
    maxZ = max(maxZ, accmag.a.z);
  }
  strncpy(txt, "ACC", 4); txt[4] = '\0';

  printCalibrate();
}

// assumes that a Serial object is created and initialized in
// the calling program
void IMU::printCalibrate() 
{
  int offX, offY, offZ; 
  float rangeX, rangeY, rangeZ, rangeMax; 
  float scaleX, scaleY, scaleZ;
  char report[120];

  offX = (minX + maxX)/2; 
  offY = (minY + maxY)/2; 
  offZ = (minZ + maxZ)/2; 

  rangeX = (float)maxX - minX;
  rangeY = (float)maxY - minY;
  rangeZ = (float)maxZ - minZ;
  // avoid putting functions inside max(): from arduino page
  rangeMax = max(rangeX,   rangeY); 
  rangeMax = max(rangeMax, rangeZ);

  // boost the smaller axes to match the largest axis
  scaleX = rangeMax/rangeX; 
  scaleY = rangeMax/rangeY;
  scaleZ = rangeMax/rangeZ;
    
  snprintf(report, sizeof(report), 
  "%s min:{%+6d,%+6d,%+6d}  max:{%+6d,%+6d,%+6d}  off:{%+6d,%+6d,%+6d} scal: ",
  txt, minX, minY, minZ, maxX, maxY, maxZ, offX, offY, offZ); 
  Serial.print(report); 
  // use dtostrf: issues with printing floats on arduino
  Serial.print(dtostrf(scaleX, 9, 4, report)); 
  Serial.print(dtostrf(scaleY, 9, 4, report)); 
  Serial.print(dtostrf(scaleZ, 9, 4, report)); 
  Serial.println();
}


template <typename Ta,
          typename Tb,
          typename To> void IMU::vector_cross(const LSM303::vector<Ta> *a,
					      const LSM303::vector<Tb> *b,
					            LSM303::vector<To> *out)
{
  out->x = (a->y * b->z) - (a->z * b->y);
  out->y = (a->z * b->x) - (a->x * b->z);
  out->z = (a->x * b->y) - (a->y * b->x);
}


template <typename Ta,
          typename Tb> float IMU::vector_dot(const LSM303::vector<Ta> *a,
					     const LSM303::vector<Tb> *b)
{
  return (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
}

