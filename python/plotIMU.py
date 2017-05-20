'''
plotIMU.py

Copyright (c) 2017 Warren E Smith  smiwarsky@gmail.com

Please see the MIT License in the project root directory for specifics.  

'''

import numpy as np
import pylab as py # go to matplotlib
import string
import pdb
import sys
import os
import time
import serial


# indicator for text stream: use in IMUtest.ino Serial.print statements
mark = "tt"

# mode text format defined in IMUtest.ino;
allModes = {'P3':np.zeros( 3 ),
                 'P6':np.zeros( 6 )}

nr = 3  # number of rows in subplots
nc = 1  # number of cols in subplots


def parseLine( line ):

    if line.find( mark ) != 1: # check for lines starting with text leader

        #print "1", line
        line = line.replace( mark, "" )   # delete text leader
        #print "2", line

        words = string.split(line,",") # words will have \r\n at end, compare to n-1
        #print "3", words
        words = words[:-1] # remove \r\n at the end
        #print "4", words

        #txt = words[1] # get data mode

        #num = len( allModes[txt] )
        num = 6

        words = words[2:] # strip off mode

        vals = np.zeros( num ) # initialize, in case next test not passed

        if len(words) == num: 
            try:
                vals = np.array([ float(k) for k in words ])
            except:
                print "Invalid line"
                print words
       
    return vals


def plotIMU( port, baudrate=115200, N=1000):

    ser = serial.Serial( port=port, baudrate=baudrate, timeout=1)

    vals = np.zeros([N,6])

    for k in range(N):

        try:
            line = ser.readline()
        except:
            print 'lost serial connection'
            ser.close()
            sys.exit('LOST SERIAL CONNECTION')

        vals[k,:] = parseLine( line )
        print k, vals[k,:]

    ser.close()

    
    fig = py.figure( figsize = (12,8) )

    ax = fig.add_subplot( nr, nc, 1 )
    
    ax.plot( vals )

    py.grid()
    
    py.show()
    

if __name__ == '__main__':

    port = '/dev/tty.usbmodemfd1341'  # Arduino serial port: the user must specify 

    # 57600, 115200 works; 9600 too slow
    # NOTE: the same baudrate must be defined in IMUtest.ino
    baudrate = 57600 #9600 #115200

    N = 1000  # the number of time samples to plot
    
    plotIMU( port, baudrate=baudrate, N=N )
