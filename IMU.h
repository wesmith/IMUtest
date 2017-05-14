/* 

IMU.h

Copyright (c) 2017 Warren E Smith  smiwarsky@gmail.com

Please see the MIT License in the project root directory for specifics.  

*/

#ifndef IMU_h
#define IMU_h

#include <LSM303.h>
#include <L3G.h>

#define DEG2RAD 0.01745329 // PI / 180
#define RAD2DEG 57.2957795 // 180 / PI
#define EPS 0.0000001      // for vector_normalize() check


class IMU
{
 public:

  ////////////////////////////

  IMU(void);

  bool init();

  void setWeightsAndGyro(bool isGyroUsed, float aa, float mm = 0);

  char* getMagAccType(void);

  char* getGyroType(void);

  void setCalibrateAcc(int    xoff, int    yoff, int    zoff,
		       float xscal, float yscal, float zscal);

  void setCalibrateMag(int    xoff, int    yoff, int    zoff,
		       float xscal, float yscal, float zscal);

  void setCalibrateGyro(int    xoff,   int  yoff,   int  zoff,
			float xnois, float ynois, float znois,
			float xscal, float yscal, float zscal);

  void setIMUAxes(float x1, float y1, float z1, float x2, float y2, float z2);

  void getRollPitchHeading(void);

  void getRawAccMagGyro(void);

  void doCalibrateAcc(bool TEST);

  void doCalibrateMag(bool TEST);


  ////////////////////////////

  float roll, pitch, head;

  LSM303::vector<int16_t> acc_raw;
  LSM303::vector<int16_t> mag_raw;
  LSM303::vector<int16_t> gyr_raw;

  LSM303::vector<float>   acc_test;
  LSM303::vector<float>   mag_test;
  LSM303::vector<float>   gyr_test;

  LSM303::vector<float>   acc_norm;
  LSM303::vector<float>   mag_norm;


 private:

  ////////////////////////////

  void read(void);
  
  void getHeading(void);

  void getRollPitch(void); 

  void getEstAcc(void);

  void printCalibrate(void);

  void vector_normalize(LSM303::vector<float> *a);

  template <typename Ta,
            typename Tb,
            typename To> void vector_cross(const LSM303::vector<Ta> *a,
					   const LSM303::vector<Tb> *b,
					         LSM303::vector<To> *out);
  template <typename Ta,
            typename Tb> float vector_dot(const LSM303::vector<Ta> *a,
					  const LSM303::vector<Tb> *b);

  
  ////////////////////////////
 
  LSM303 accmag;  // accelerometer and magnetometer

  L3G    gyro;    // gyro

  float alpha, alphaMag;

  long prevTime;

  bool gyroUsed, initAccMagVec = true;  

  LSM303::vector<float>   est_acc;
  LSM303::vector<float>   est_mag; 
  LSM303::vector<float>   omega;  

  LSM303::vector<float>   rollAxis;
  LSM303::vector<float>   rollReference;

  LSM303::vector<int16_t> a_off;
  LSM303::vector<float>   a_sca;
  
  LSM303::vector<int16_t> m_off;
  LSM303::vector<float>   m_sca;
  
  LSM303::vector<int16_t> g_off;
  LSM303::vector<float>   g_noi;
  LSM303::vector<float>   g_sca;

  int minX =  32767; int minY =  32767; int minZ =  32767; 
  int maxX = -32767; int maxY = -32767; int maxZ = -32767;
  char txt[4];
};


#endif
