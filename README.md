IMUTest

The purpose of this project is to develop a simple utility class to run on Arduino processors to generate roll,
pitch, and heading information from a combination of magnetometer/accelerometer and gyro data. A straightforward
complementary-fusion algorithm to combine the accelerometer and gyro data is implemented.

The IMU class includes the following public methods:

- getMagAccType(): Gets the magnetometer/accelerometer chip type.

- getGyroType(): Gets the gyro chip type.

- setWeightsAndGyro(): Turns the gyro fusion on or off, and sets the gyro/accelerometer fusion ratio 'alpha'. The
                       alpha parameter is a float from 0.0 to 1.0: alpha ~ 1 weights the gyro heavily, and alpha ~ 0
		       weights the accelerometer heavily. Note that two separate IMU classes can be instantiated
		       simultaneously, to directly compare the gyro-accelerometer fusion results with the
		       accelerometer-only results, for the roll, pitch, and heading angles. This is demonstrated
		       in IMUtest.ino.  

- doCalibrateAcc(): Accelerometer calibration data (max, min x, y, z values, offsets, scale factors) are printed to
                    the screen via the Arduino serial connection as the user manipulates the IMU carrier in three-space;
		    the user uses the offset and scale values as the inputs to setCalibrateAcc().
		    NOTE: it is important that the 9-axis (accelerometer, magnetometer, gyro) carrier be moved
		    without jostling, to avoid artificial spiking in the accelerometer data. Smooth
		    carrier motion to sample the gravity vector along x, -x, y, -y, and z, -z is important for accurate
		    calibration data. It is suggested that the process be repeated multiple times, and the average of
		    the offsets and scale factors be used in setCalibrateAcc() below. 

- setCalibrateAcc(): Allows the user to set the offsets and scales of the accelerometer, determined by doCalibrate().
                     This is a manual step, so that the user can be certain of the values being used. See the
		     IMUTest.ino example code. Note that this is a 6-parameter calibration. 

- doCalibrateMag(): Magnetometer calibration data (max, min x, y, z values, offsets, scale factors) are printed to
                    the screen via the Arduino serial connection as the user manipulates the IMU carrier in three-space;
		    the user uses these values as the inputs to setCalibrateMag(). This data is more robust than the
		    accelerometer data, in that it is not as subject to jolting. However, the carrier should be as far
		    away as possible from laptops and other magnetic-field-generating equipment for this calibration
		    to produce accurate results. Also, the x, -x, y, -y, and z, -z axes of the carrier need to be
		    pointed consecutively in the direction of the local Earth magnetic vector. The orientation of
		    this vector has a horizontal component in the direction of observer's magnetic north, and
		    a dip angle (also called magnetic inclination) with respect to the horizontal that depends upon
		    the user's location on the Earth. See

                    https://en.wikipedia.org/wiki/Magnetic_dip
 
- setCalibrateMag(): Allows the user to set the offsets and scales of the magnetometer. Again, this is a manual,
                     6-parameter calibration. 

- setCalibrateGyro(): Currently not implemented. An estimate of the gyro scale factor (deg/sec) is set by default
                      in IMU.init(). 

- setIMUAxes(): Allows the user to define the roll and pitch axes of the IMU carrier. The default is that the +X axis
                of the carrier is the roll axis (i.e., the +X axis is the 'nose' of the carrier), and the +Y axis of
		the carrier is the roll reference axis (from which roll is measured: i.e., the 'left wing' of the
		carrier). A right-handed system is taken, so the +Z axis is up. These directions correspond to the
		markings on the particular Pololu chip used. For all axes, positive rotation is clockwise,
		looking along the axis. Thus positive roll is 'right wing' down, positive pitch is 'nose down', and
		positive heading is from north to west, with zero heading due magnetic north. 

- getRollPitchHeading(): Generates the roll, pitch, heading values. 

- getRawAccMagGyro(): Returns raw (uncalibrated) accelerometer, magnetometer, and gyro data, for debugging purposes.


The IMU class requires the Pololu classes

https://github.com/pololu/lsm303-arduino

and

https://github.com/pololu/l3g-arduino

which must be installed separately. 

This class has been tested with IMUTest.ino using the Arduino board model UNO R2 for the LSM303HLHC
magnetometer/accelerometer and the L3GD20 gyro, on the (now discontinued) carrier:

https://www.pololu.com/product/1268





