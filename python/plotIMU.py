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
mark = "PP"

num = 8 # number of items to parse in data stream

nr = 3  # number of rows in subplots
nc = 1  # number of cols in subplots

blurb = 'Copyright (c) 2017 Warren E Smith  https://github.com/wesmith/IMUtest'

def parseLine( line ):

    if line.find( mark ) != 1: # check for lines starting with text leader

        line = line.replace( mark, "" )   # delete text leader

        words = string.split(line,",") 
        words = words[:-1] # remove \r\n at the end
        words = words[1:]  # strip off leading null

        vals = np.zeros( num ) # initialize, in case 'try' fails in early stream
        
        if len(words) == num: 
            try:  # early stream from IMUtest.ino while booting can be corrupted 
                vals = np.array([ float(k) for k in words ])
            except:
                print "Invalid line"
                print words
       
    return vals
    

def plotIMU( port, baudrate=115200, N=1000, savDir=None):

    ser = serial.Serial( port=port, baudrate=baudrate, timeout=1)

    vals = np.zeros([N,num])
    tt   = np.zeros(N)  # time axis
    #t1   = time.time() # initialize time

    for k in range(N):

        try:
            line = ser.readline()
        except:
            print 'lost serial connection'
            ser.close()
            sys.exit('LOST SERIAL CONNECTION')

        vals[k,:] = parseLine( line )
        tt[k]     = time.time() # - t1
        print k  # observe sample progress while moving IMU
        

    ser.close()

    tt -= tt[nIgnore]  # subtract start time from all times

    freq = (N - nIgnore)/tt[-1]  # samples per second for the plot

    fig = py.figure( figsize = (12,8) )

    legendLabel = ['NO GYRO','WITH GYRO',
                   'NO GYRO','WITH GYRO',
                   'NO GYRO or MAG SMOOTHING','WITH GYRO and MAG SMOOTHING']
    yLabel = ['ROLL (DEG)', 'PITCH (DEG)', 'MAG HEADING (DEG)']
    colors = ['r','k']
    
    for k in range(3):
        ax = fig.add_subplot( nr, nc, k+1 )
        ax.plot( tt[nIgnore:], vals[nIgnore:,2*k+2],
                 '%s-' % colors[2*k     % len(colors)],
                 label=legendLabel[2*k  ] )
        ax.plot( tt[nIgnore:], vals[nIgnore:,2*k+3],
                 '%s-' % colors[(2*k+1) % len(colors)],
                 label=legendLabel[2*k+1] )
        ax.legend( loc=2, prop={'size':10})
        ax.set_xlim( tt[nIgnore], tt[-1] )
        ax.set_ylabel(yLabel[k])
        ax.grid()
        if (k == 2):
            ax.text(0.99, 0.01, blurb,
                    horizontalalignment='right', verticalalignment='bottom',
                    transform=ax.transAxes, fontsize=8,
                    #bbox=dict(facecolor='red', alpha=0.5)
            )

    tim = time.strftime('%y%m%d-%H%M%S')
    fnam = '%s-plotIMU-results' % tim
    
    txt = '%s\nROLL, PITCH, MAG HEADING vs TIME'  % fnam
    txt += '\nalphaACC: %.2f  alphaMAG: %.2f' % (vals[-1,0], vals[-1,1])
    txt += '   SAMPLE RATE: %.2f SAMP/SEC' % freq
    py.suptitle(txt)
    py.xlabel('TIME (sec)')

    if savDir:
        fnam = os.path.join( savDir, ('%s.pdf' % fnam) )
        print 'Saving file %s' % fnam
        py.savefig( fnam, bbox_inches='tight' )
        py.close()
    else:
        py.show()
    

if __name__ == '__main__':

    savDir = './results' # save image file here; set to None for screen plot

    port = '/dev/tty.usbmodemfd1341'  # Arduino serial port: the user must specify 

    # NOTE: the same baudrate must be defined in IMUtest.ino
    baudrate = 57600 #9600 #115200

    nIgnore = 100 # number of samples to ignore as Arduino is restarting
              # (Arduino is reset when this program is run)

    N = 500  # the number of time samples to plot
    
    if (N < 2 * nIgnore):
        sys.exit('N must be larger than %d' % (2 * nIgnore))
    
    plotIMU( port, baudrate=baudrate, N=N, savDir=savDir )
