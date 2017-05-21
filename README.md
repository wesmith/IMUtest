# IMUTest

The purpose of this project is to develop a simple utility class to run on Arduino processors to generate roll,
pitch, and heading information from a combination of magnetometer/accelerometer and gyro data. A straightforward
complementary-fusion algorithm to combine the accelerometer and gyro data is implemented.

**_5/21/17 UPDATE_**

#### Plotting IMU performance

A python program plotIMU.py has been developed to plot the roll, pitch, and heading angles from the IMU
as it is randomly moved in three-space, to analyze the effect of the gyro/accelerometer fusion and magnetnetometer
low-pass filtering. A series of plots have been produced which are linked to a discussion of the results below. The two 
parameters that were varied from one plot to the other were the *alphaACC* and the *alphaMAG*. 

**alphaACC**  This is a float between 0 and 1, whose value is defined in IMUtest.ino. _alphaACC_ ~ 1 weights the gyro very
heavily in the accelerometer/gyroscope complementary filter, and _alphaACC_ ~ 0 weights the accelerometer very heavily. Values
around 0.95 appear to be optimum: values lower than this produce noisy roll, pitch, and heading results, and values closer to 1 
produce values of roll, pitch, and heading that tend to drift over time due to gyroscope drift. 

**alphaMAG**  As with *alphaACC*, this is a float between 0 and 1, whose value is again defined in IMUtest.ino. Unlike the 
complementary filter parameter *alphaACC*, *alphaMAG* is a weighting factor for a simple low-pass magnetometer filter, since
there is no corresponding 'magnetic gyroscope' to use in smoothing the magnetometer data. _alphaMAG_ ~ 1 is a very strong
smoothing filter, and _alphaMAG_ ~ 0 is a very weak filter. 

**ANALYSIS** A series of experiments were performed to examine the effects of *alphaACC* and *alphaMAG* on roll, pitch, and heading
values. Each experiment lasted about 12 to 14 seconds, where the Pololu IMU (link at bottom) was manipulated by hand in a random, 
quasi-periodic fashion in roll, pitch, and heading. The goal here was to explore the effects of the two weighting parameters on the
smoothness of the roll, pitch, and heading output, and not to try to determine absolute accuracy of the IMU in three dimensions. To
determine such accuracy would require some kind of orientation-truth device (either a calibrated mechanical support system or an
optical 3D motion-mensuration system) which is not within the scope of this current effort. 

Each plot consists of three subplots: one each for roll, pitch, and heading, all in degrees. The horizontal axis is time in seconds over which the IMU carrier was randomly moved in 3D. Each subplot shows two curves. The "control" IMU processing curve (in red, labeled 'NO GYRO' for the roll and pitch plots, or 'NO GYRO or MAG SMOOTHING' for the heading plot) plots the roll, pitch, and heading angles as derived from the raw accelerometer and raw magnetometer values. The "test" IMU processing curve (in black, labeled 'WITH GYRO' for the roll and pitch plots, or "WITH GYRO and MAG SMOOTHING' for the heading plot) plots the roll, pitch, and heading angles as derived from the fused acceleromter/gyro and smoothed magnetometer values. In IMUtest.ino, the control data is generated by the **noGyro** object, and the test data is generated by the **withGyro** object. Each object is an instantiation of the IMU class. The independent objects run simultaneously in IMUtest, and each object is using their own samples of the accelerometer, magnetometer, and gyroscope. The only thing they have in common is that they are subject to the same physical motions at the same times via the IMU carrier. 

Each plot's title indicates the value of *alphaACC* and *alphaMAG* for the three subplots on that page, as well as the sample rate represented by the plot. A plot sample rate of 30 samples/sec was typical. A few experiments will now be discussed in detail. 

**alphaACC = 0.95, alphaMAG = 0.80** This is perhaps closest to an optimum weighting combination of all those tried so far today. The plot is [here](../master/python/results/170521-132428-plotIMU-results.pdf). It is evident that the gyro/accelerometer fusion is effectively smoothing the roll and pitch values. The roll and pitch values are derived independently of the magnetometer in this straightforward complementary approach: their calculation just requires the accelerometer vector and definition of the IMU platform coordinate axes. The heading value requires both the accelerometer and the magnetometer inputs, so the additional low-pass smoothing of the magnetometer is important in achieving a smoothed heading estimate. The next example shows the result of accelerometer/gyro fusion without magnetometer filtering, to clarify this point. 

**alphaACC = 0.95, alphaMAG = 0** The plot is [here](../master/python/results/170521-152330-plotIMU-results.pdf). Again, the roll and pitch angles have been significantly smoothed by the gyro/accelerometer fusion. The heading angle is very noisy, though, since there has been no magnetometer smoothing. Even though the key in the "heading" subplot indicates, for the black line, that "WITH GYRO and MAG SMOOTHING" has been applied, the alphaMAG parameter is 0 here, which means no magnetometer smoothing. Note however that in the "heading" suplot the black line may be showing some slight evidence of being less noisy than the red line, indicating that the gyro/accelerometer fusion has performed some smoothing. As stated above, the heading angle relies on both the accelerometer and magnetometer data. This example demonstrates the importance of smoothing the magnetometer data as well as the accelerometer data. The large spikes at approximately 7 and 8 seconds in the 'heading' subplot indicate that the IMU device pointed past due magnetic south at the two different occasions, where the sign of the heading abruptly changes from -180 deg to +180 deg. 





**_5/19/17 UPDATE_**

#### Added quaternion output.

#### I2C master/slave approach:

An attempt was made to run the Arduino as BOTH an I2C master and I2C slave (from a Raspberry Pi 3 (RPi3) master)
simultaneously, but initial results were unsuccessful: roll, pitch, heading data processed on the Arduino was not
successfully transmitted to the RPi3. This may be revisited at some point. The Arduino is flawless as a master I2C
device in running the three I2C devices on the Pololu carrier: the LSM303 accelerometer and magnetometer, and the
L3G gyro. However, when the Arduino was configured as an I2C slave to the RPi, in addition to running the three
carrier devices, there may have been I2C bus issues when coupled with RPi3 I2C protocols. TBD whether it is even
possible to run the Arduino UNO as both an I2C slave and master simultaneously: a quick web search didn't bring up
projects involving simultaneous master/slave I2C configurations on the Arduino.

As a result, the I2C slave components of IMUtest.ino will be disabled for the time being. 


#### Visual Python:

Additional python code (that requires the Visual Python library) has been added to facilitate the real-time
display of the IMU roll, pitch, and heading, either directly from roll, pitch, and heading values generated by the IMU
class, or from the quaternion generated by the IMU class. In addition, a set of axes are displayed showing the
real-time orientation of the chip in a fixed North, West, Up local frame, and the inverse transformation, showing
the local frame in the fixed chip X, Y, Z frame. 


**_END of 5/19/17 update_**


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
		    the user's location on the Earth. See <https://en.wikipedia.org/wiki/Magnetic_dip>

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

- getRollPitchHeadingQuat(): Generates the roll, pitch, heading, and quaternion values. 

- getRawAccMagGyro(): Returns raw (uncalibrated) accelerometer, magnetometer, and gyro data, for debugging purposes.


The IMU class requires the Pololu classes

https://github.com/pololu/lsm303-arduino

and

https://github.com/pololu/l3g-arduino

which must be installed separately. 

This class has been tested with IMUTest.ino using the Arduino board model UNO R2 for the LSM303HLHC
magnetometer/accelerometer and the L3GD20 gyro, on the (now discontinued) carrier:

https://www.pololu.com/product/1268





