'''
displayIMU.py

Copyright (c) 2017 Warren E Smith  smiwarsky@gmail.com

Please see the MIT License in the project root directory for specifics.  


This code is associated with the Pololu IMU chip 'MinIMU-9 v2', and Arduino 
program IMUtest.ino.

NOTE: The Arduino program must be running WITHOUT the Arduino's serial port 
      window open, on a known USB port (the port address is entered in MAIN below)
      BEFORE this program is started. If this program ends abnormally without
      closing the USB port, the USB port should be reset (e.g., by unplugging 
      and replugging the Arduino). DO NOT attempt to upload the Arduino code
      via the Arduino IDE using the USB port while this program is running, 
      to avoid USB bus contention. Also, do not open the Arduino serial port
      while this program is running. 

      Short version: kill this program before doing anything
      on the Arduino side (uploading, viewing Arduino serial port) to avoid
      more than one device attempting to access the USB port. 

This program requires visual python. 

Note that when the python code is restarted, it restarts the Arduino. This is
not an issue for this application, but it takes a few seconds for the python
graphics to become active because of the restart. 

For those interested, a way around this auto-reset feature is discussed here:

http://playground.arduino.cc/Main/DisablingAutoResetOnSerialConnection

'''

import numpy as np
from   visual import *
import serial
import string
import math
import pdb
import sys
import time


deg2rad = np.pi / 180.0 
rad2deg = 180. / np.pi  


class RPYDisplay():

    def __init__( self ):
      
        self.rollHead   = arrow(   pos=(-0.6,0,0), axis=( 0.2,0,0),
                               shaftwidth=0.02, fixedwidth=1, color=color.red)
        self.rollTail   = cylinder(pos=(-0.6,0,0), axis=(-0.2,0,0),
                               radius=0.01, color=color.red)
        self.pitchHead  = arrow(   pos=( 0.0,0,0), axis=( 0.2,0,0),
                               shaftwidth=0.02, fixedwidth=1, color=color.green)
        self.pitchTail  = cylinder(pos=( 0.0,0,0), axis=(-0.2,0,0),
                               radius=0.01,color=color.green)
        self.yawHead    = arrow(   pos=( 0.6,0,0), axis=( 0.2,0,0),
                               shaftwidth=0.02, fixedwidth=1, color=color.cyan,)

        self.R = label(pos=(-0.6, 0.3, 0), text="-", box=0, opacity=0)
        self.P = label(pos=( 0.0, 0.3, 0), text="-", box=0, opacity=0)
        self.Y = label(pos=( 0.6, 0.3, 0), text="-", box=0, opacity=0)

        self.l1 = label(pos=( 0.6,  0.22, 0), text="N",  box=0, opacity=0,
                        color=color.yellow)
        self.l2 = label(pos=( 0.6, -0.22, 0), text="S",  box=0, opacity=0,
                        color=color.yellow)
        self.l3 = label(pos=( 0.38,    0, 0), text="W",  box=0, opacity=0,
                        color=color.yellow)
        self.l4 = label(pos=( 0.82,    0, 0), text="E",  box=0, opacity=0,
                        color=color.yellow)
        self.l5 = label(pos=( 0.75, 0.15, 0), text="NE", box=0, opacity=0,
                        color=color.yellow, height=7)
        self.l6 = label(pos=( 0.45, 0.15, 0), text="NW", box=0, opacity=0,
                        color=color.yellow, height=7)
        self.l7 = label(pos=( 0.75,-0.15, 0), text="SE", box=0, opacity=0,
                        color=color.yellow, height=7)
        self.l8 = label(pos=( 0.45,-0.15, 0), text="SW", box=0, opacity=0,
                        color=color.yellow, height=7)

    def __call__( self, r, p, y ):
       
        # r, p, y in radians
 
        # roll, pitch, yaw angle signs are based upon Pololu chip 'MinIMU-9 v2':
        # looking towards the origin from the negative side of an axis in the
        # positive direction, a positive angle is clockwise
        # (see graphic on bottom of IMU chip)
        self.rollHead.axis   = (  0.2 * cos(r),  0.2 * sin(r), 0. )
        self.rollTail.axis   = ( -0.2 * cos(r), -0.2 * sin(r), 0. )
        self.pitchHead.axis  = (  0.2 * cos(p),  0.2 * sin(p), 0. )
        self.pitchTail.axis  = ( -0.2 * cos(p), -0.2 * sin(p), 0. )
        self.yawHead.axis    = ( -0.2 * sin(y),  0.2 * cos(y), 0. ) # puts N up in plot
        self.R.text      = 'ROLL %+7d'      % ( r * rad2deg ) 
        self.P.text      = 'PITCH %+7d'     % ( p * rad2deg )
        self.Y.text      = 'HEADING %+7d'   % ( y * rad2deg )


class Heading():
   
    def __init__( self, mode, txt='' ):

        # mode text format defined in IMUtest.ino;
        # zero arrays are to store last values read if error occurs
        self.allModes = {'VEC':np.zeros( 12 ),
                         'RPH':np.zeros(  6 )}

        self.mode     = mode

        # SCENE showing chip acc, mag vectors superposed on fixed IMU X, Y, Z axes
        scene         = display( title='%s  ACC, MAG Vectors' % txt,
                                 x=0, y=0, width=400, height=400 )
        scene.range   = ( 1.5, 1.5, 1.5 )
        scene.forward = ( 1.0,  1.0,  -1.0 ) # camera look direction 
        scene.up      = ( 0.0, 0.0, 1.0 )
        scene.select()
        # reference axes
        arrow( color=color.white, axis=(0,0,1), shaftwidth=0.02, fixedwidth=1 )
        arrow( color=color.white, axis=(0,1,0), shaftwidth=0.02, fixedwidth=1 )
        arrow( color=color.white, axis=(1,0,0), shaftwidth=0.02, fixedwidth=1 )
        # labels for reference axes
        label( pos=(0,0,1.1),   text="CHIP Z-AXIS", box=0, opacity=0, color=color.yellow ) 
        label( pos=(0,1.2,0),   text="CHIP Y-AXIS", box=0, opacity=0, color=color.yellow )
        label( pos=(1.2,0,0),   text="CHIP X-axis", box=0, opacity=0, color=color.yellow )

        label( pos=(0,-0.8,-0.8),
               text="RAW ACC: RED\nRAW MAG: GREEN\nEST ACC: MAGENTA\nEST MAG: CYAN",
               box=1, color=color.yellow, font='monospace', opacity=0 )
        # IMUtest arrows
        self.acc    = arrow( color=color.red,     shaftwidth=0.04, fixedwidth=1 )
        self.mag    = arrow( color=color.green,   shaftwidth=0.04, fixedwidth=1 )
        self.estAcc = arrow( color=color.magenta, shaftwidth=0.04, fixedwidth=1 )
        self.estMag = arrow( color=color.cyan,    shaftwidth=0.04, fixedwidth=1 )

        
        # SCENE showing Euler angles from raw accelerometer
        scene2        = display( title='EULER ANGLES: RAW ACCELEROMETER',
                                 x=0, y=400, width=400, height=200 )
        scene2.range   = ( 1, 1, 1 )
        scene2.forward = ( 0.0,  0.0,  -1.0 ) # camera look direction 
        scene2.up      = ( 0.0, 1.0, 0.0 )
        scene2.select()
        self.s2 = RPYDisplay()

        # SCENE showing Euler angles from gyro/accel fusion
        scene3        = display( title='EULER ANGLES: GYRO/ACCEL FUSION',
                                 x=0, y=600, width=400, height=200 )
        scene3.range   = ( 1, 1, 1 )
        scene3.forward = ( 0.0,  0.0,  -1.0 ) # camera look direction 
        scene3.up      = ( 0.0, 1.0, 0.0 )
        scene3.select()
        self.s3 = RPYDisplay()
 

    def __call__( self, line ):
        '''
        Run this class AFTER Arduino is already running IMUtest.ino. 
        First verify that baud rates are identical in this implementation 
        and in IMUtest.ino. 
        '''
        txt = self.mode
        num = len( self.allModes[txt] )
        
        if line.find( txt ) != -1: # check for lines starting with text leader

            line = line.replace( txt, "" )   # delete text leader
            
            # print line  # debug
            
            words = string.split(line,",")   # words will have \r\n at end, so compare to n-1
            words = words[:-1] # remove \r\n at the end

            # print words # debug

            vals = np.zeros( num ) # initialize, in case next test not passed

            if len(words) == num: 
                try:
                    vals = np.array([ float(k) for k in words ])
                except:
                    print "Invalid line"
                    print words
                    vals = self.allModes[txt]

            # self.allModes[txt] = vals  # explicitly set lastVals

            # print vals  # WS 4/27/17 debug
            if (txt == 'VEC'):
                self.acc.axis = vals[0:3] # raw acceleration vector
                self.mag.axis = vals[3:6] # raw magnetometer vector
                self.estAcc.axis = vals[6:9] # gyro/accel fused vector

                r = vals[9]  * deg2rad
                p = vals[10] * deg2rad
                h = vals[11] * deg2rad
                self.s2( r, p, h )

            if (txt == 'RPH'):
                r = vals[0] * deg2rad
                p = vals[1] * deg2rad
                h = vals[2] * deg2rad
                self.s2( r, p, h )
                r = vals[3] * deg2rad
                p = vals[4] * deg2rad
                h = vals[5] * deg2rad
                self.s3( r, p, h )
                
        
def run( ser, f, rateVal=100 ):
    '''
    ser = serial-port object
    f = IMU object to run vpython graphics
    rateVal = rate at which to run loop (Hz)
    '''

    while 1:

        rate( rateVal ) # critical for VPython 6 to get rate()
        
        # ensure serial port is closed properly when Arduino is disconnected
        try: 
            line = ser.readline()
        except:
            print 'lost serial connection'
            ser.close()
            sys.exit('LOST SERIAL CONNECTION')

        # print line  # debug

        ser.flushInput()  # flush input buffer
        
        f( line )


def displayIMU( mode, port, baudrate=115200, rateVal=200 ):

    ser = serial.Serial( port=port, baudrate=baudrate, timeout=1)

    txt  = 'displayIMUtest.py mode:%s' % mode
    f = Heading( mode, txt=txt ) # instantiate display class
    
    run( ser, f, rateVal=rateVal )
    

if __name__ == '__main__':

    # mode is defined in IMUtest.ino for a particular run: it must match the following:
    mode = 'VEC'
    #mode = 'RPH'

    port = '/dev/tty.usbmodemfd1341'  # Arduino serial port: the user must specify 

    # 57600, 115200 works; 9600 too slow
    # NOTE: the same baudrate must be defined in IMUtest.ino
    baudrate = 115200
    
    # refresh rate per second for main loop;
    # 200 works for 50 Hz setting in IMUtest.ino
    # 100 works for 20 Hz setting in IMUtest.ino, 50 is too slow
    rateVal  = 200 

    displayIMU( mode, port, baudrate=baudrate, rateVal=rateVal  )

    
