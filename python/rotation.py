'''
rotation.py 

Copyright (c) 2017 Warren E Smith  smiwarsky@gmail.com

Please see the MIT License in the project root directory for specifics.  

This class implements various transformations that can be used to represent 
arbitrary rotations in 3-space. 

REF:

Paul D Groves (2013) 'Principles of GNSS, Inertial, and Multisensor Integrated
Navigation Systems', second edition.

'''

import numpy as np
import sys
import pdb


DEG_2_RAD = np.pi / 180.0 
RAD_2_DEG = 180.  / np.pi
eps = np.finfo(np.float).eps  # smallest representable positive number 


def getRotFromEuler(e):

    cr = np.cos(e[0]); sr = np.sin(e[0])  # roll
    cp = np.cos(e[1]); sp = np.sin(e[1])  # pitch
    cy = np.cos(e[2]); sy = np.sin(e[2])  # yaw

    rr = np.array( [ [ 1., 0.,  0.], [  0., cr, sr], [0., -sr, cr] ] )
    rp = np.array( [ [cp,  0., -sp], [  0., 1., 0.], [sp,  0., cp] ] )
    ry = np.array( [ [cy,  sy,  0.], [ -sy, cy, 0.], [0.,  0., 1.] ] )

    # yaw applied first: see Groves eq 2.22, p. 38
    return np.dot( rr, np.dot( rp, ry ) )


def getEulerFromRot(r):

    # ensure that arcsin() limit not violated by small roundoff errors
    if r[0,2] >  1.0: r[0,2] =  1.0
    if r[0,2] < -1.0: r[0,2] = -1.0

    # ensure that arctan2(+/- 0, - 0) = +/- pi condition doesn't introduce pi
    # where result should be 0; 
    # i.e., force values in the machine noise to be positive
    if abs(r[0,0]) < eps: r[0,0] = abs(r[0,0]) 
    if abs(r[2,2]) < eps: r[2,2] = abs(r[2,2])
    
    # using Groves eq. 2.23, p. 38
    roll  = np.arctan2( r[1,2], r[2,2] )
    pitch = np.arcsin( -r[0,2] )
    yaw   = np.arctan2( r[0,1], r[0,0] )

    return np.array([roll, pitch, yaw]) # in radians

    
def getQuatFromEuler(e):

    # Groves eq. 2.38, p. 42

    r, p, y = e/2.
    cr = np.cos(r); sr = np.sin(r)
    cp = np.cos(p); sp = np.sin(p)
    cy = np.cos(y); sy = np.sin(y)

    q0 = cr * cp * cy + sr * sp * sy
    q1 = sr * cp * cy - cr * sp * sy
    q2 = cr * sp * cy + sr * cp * sy
    q3 = cr * cp * sy - sr * sp * cy

    return np.array([q0, q1, q2, q3])


def getRotFromQuat(qq):

    # Groves eq. 2.34, p. 41

    q0, q1, q2, q3 = qq
    
    r00 = q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3
    r11 = q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3
    r22 = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3
    
    r01 = 2. * (q1 * q2 + q3 * q0)
    r10 = 2. * (q1 * q2 - q3 * q0)

    r02 = 2. * (q1 * q3 - q2 * q0)
    r20 = 2. * (q1 * q3 + q2 * q0)
    
    r12 = 2. * (q2 * q3 + q1 * q0)
    r21 = 2. * (q2 * q3 - q1 * q0)

    return np.array( [ [r00, r01, r02], [r10, r11, r12], [r20, r21, r22] ] )


def getQuatFromVec(vv):

    # Groves eq. 2.41, p. 42

    mv = np.linalg.norm(vv)  # magnitude

    q0 = np.cos( mv/2. ); sv = np.sin( mv/2. )

    if mv < eps:
        q1 = q2 = q3 = 0.
    else:
        q1, q2, q3 = sv * vv / mv

    return np.array([q0, q1, q2, q3])


def getVecFromQuat(qq):

    # Groves eqs. 2.33, p. 40, and 2.39, p. 42

    q0, q1, q2, q3 = qq

    mu = 2. * np.arccos(q0)

    if mu < eps:
        v0 = v1 = v2 = 0.
    else:
        v0, v1, v2 = mu * qq[1:] / np.linalg.norm(qq[1:]) # normalize over last three

    return np.array([v0, v1, v2])


class Rotation(object): # must inherit from 'object' to get @property to work!!!

    def __init__(self, R=None, E=None, Q=None, V=None):
        '''
        A convenience class to retrieve an object representing a 3-D rotation
        in any one of four ways. The inverse of the object can also be obtained.

        Only one of R, E, Q, or V can be used to instantiate this class. 

        R: a 3x3 ndarray representing an orthonormal rotation matrix

        E: a 3-tuple of floats representing the roll, pitch, yaw Euler angles
           (radians). The Groves convention used: yaw applied first, then pitch, 
           then roll. 

        Q: a 4-tuple of floats representing quaternion q0, q1, q2, q3 values
           (Q is normalized to 1.0 internally if it isn't already normalized)

        V: a 3-tuple of floats representing the vx, vy, vz rotation vector
           (the magnitude of V is always the rotation angle in radians: 
            V should not be normalized)

        Once the class has been instantiated (say as r in the calling program), 
        then any of the four representations of the rotation can be accessed as:
        r.Rot  
        r.Euler (always output in radians)
        r.Quat
        r.Vec

        The inverses can be accessed as:
        r.invRot
        r.invEuler
        r.invQuat
        r.invVec

        The Rotation object r can be combined with another Rotation 
        object B using:
        r.combine(B) 
        which implies r.Rot * B.Rot matrix multiplication, in that order
        '''

        dd = len([x for x in [R, E, Q, V] if x is not None])
        if dd != 1:
            sys.exit('ERROR: class Rotation requires one input: %d supplied' % dd)

        if R is not None:
            if R.size is not 9:
                raise AttributeError('Rotation matrix is not 3x3')
                # still need to test for orthogonal matrix

        if E is not None:
            E = np.array(E)
            if E.size is not 3:
                raise AttributeError('Euler tuple does not have 3 elements')

        if Q is not None:
            Q = np.array(Q)            
            if Q.size is not 4:
                raise AttributeError('Quaternion tuple does not have 4 elements')
            if np.abs(1.0 - np.dot(Q, Q)) > eps:
                Q /= np.linalg.norm(Q)  # normalize
                print 'Performed normalization on quaternion input'
                      
        if V is not None:
            V = np.array(V)
            if len(V) is not 3:
                raise AttributeError('Rotation tuple does not have 3 elements')
        
        self._R = R
        self._E = E
        self._Q = Q
        self._V = V
        self._iR = None # inverse values
        self._iE = None
        self._iQ = None
        self._iV = None

        
    def noSet(self, val):
        txt =  'Cannot set attributes in ROTATION class after instantiation'
        raise AttributeError(txt)

        
    def Rot(self):

        if self._R is not None:
            return self._R

        if self._E is not None:
            self._R = getRotFromEuler(self._E)
            return self._R
        
        if self._Q is not None:
            self._R = getRotFromQuat(self._Q)
            return self._R
        
        if self._V is not None:
            self._R = getRotFromQuat(getQuatFromVec(self._V))
            return self._R

    Rot = property(Rot, noSet)  # getter, setter
    
        
    def Euler(self):

        if self._E is not None:
            return self._E

        if self._R is not None:
            self._E =  getEulerFromRot(self._R)
            return self._E
        
        if self._Q is not None:
            self._E = getEulerFromRot(getRotFromQuat(self._Q))
            return self._E
        
        if self._V is not None:
            self._E = getEulerFromRot(getRotFromQuat(getQuatFromVec(self._V)))
            return self._E

    Euler = property(Euler, noSet)

        
    def Quat(self):

        if self._Q is not None:
            return self._Q

        if self._R is not None:
            self._Q = getQuatFromEuler(getEulerFromRot(self._R))
            return self._Q
        
        if self._E is not None:
            self._Q =  getQuatFromEuler(self._E)
            return self._Q
        
        if self._V is not None:
            self._Q =  getQuatFromVec(self._V)
            return self._Q

    Quat = property(Quat, noSet)

        
    def Vec(self):

        if self._V is not None:
            return self._V

        if self._R is not None:
            self._V =  getVecFromQuat(getQuatFromEuler(getEulerFromRot(self._R)))

            return self._V

        if self._E is not None:
            self._V = getVecFromQuat(getQuatFromEuler(self._E))
            return self._V

        if self._Q is not None:
            self._V =  getVecFromQuat(self._Q)
            return self._V

    Vec = property(Vec, noSet)
    

    def invRot(self):

        if self._iR is not None:
            return self._iR
        else:
            self._iR = np.transpose(self.Rot)
            return self._iR

    invRot = property(invRot, noSet)
        

    def invEuler(self):

        if self._iE is not None:
            return self._iE
        else:
            self._iE = getEulerFromRot(self.invRot)
            return self._iE

    invEuler = property(invEuler, noSet)        

    
    def invQuat(self): # can do by simply negating last 3 values,
                       #but more fun this way

        if self._iQ is not None:
            return self._iQ
        else:
            self._iQ = getQuatFromEuler(self.invEuler)
            return self._iQ

    invQuat = property(invQuat, noSet)
        

    def invVec(self): # can do simply by negating the vector,
                      # but more fun this way

        if self._iV is not None:
            return self._iV
        else:
            self._iV = getVecFromQuat(self.invQuat)
            return self._iV

    invVec = property(invVec, noSet)


    def combine(self, B):
        # combine the current Rotation object A with the input
        # rotation object B, as in A * B matrix order
        return Rotation(R=np.dot(self.Rot, B.Rot))

    
    def __repr__(self):
    
        return 'Rotation Class\n'
        

    def __str__(self): # called with 'print' command

        txt =  '\nRotation Matrix \n %s \n'         % str(self.Rot)
        txt += '\nInverse Rotation Matrix \n %s \n' % str(self.invRot)
        txt += '\nEuler Angles (deg)\n %s \n'            % \
                                     str(np.array(self.Euler) * RAD_2_DEG)
        txt += '\nInverse Euler Angles (deg)\n %s \n'    % \
                                     str(np.array(self.invEuler) * RAD_2_DEG)
        txt += '\nQuaternion \n %s \n'              %  str(self.Quat)
        txt += '\nInverse Quaternion \n %s \n'      %  str(self.invQuat)

        txt += '\nRotation Vector \n %s \n'         %  str(self.Vec)
        txt += '\nInverse Rotation Vector \n %s \n' %  str(self.invVec)
        return txt
        
        
def test(R=None, E=None, Q=None, V=None, deg=False):

    if deg and E is not None:
        E = np.array(E) * DEG_2_RAD

    ee = Rotation(E=E)
    print '\nEuler is source:'
    print ee

    qq = Rotation(Q=ee.Quat)
    print '\nQuaternion is source:'
    print qq

    rr = Rotation(R=qq.Rot)
    print '\nRotation is source:'
    print rr

    vv = Rotation(V=rr.Vec)
    print '\nVector is source:'
    print vv

    tr = Rotation(R=np.transpose(vv.Rot))
    print '\nTranspose(Rotation) is source:'
    print tr

    # test combining two objects
    eeInv = Rotation(R=ee.invRot)
    print '\nCombining: ee * eeInv'
    print ee.combine(eeInv)
    print '\nCombining: eeInv * ee'
    print eeInv.combine(ee)

    return ee

    
if __name__ == '__main__':

    E = (30., 40., 50.) # roll, pitch, yaw
    E = (-90., -90., 90.) # this works: inverse Euler is (0, 90, 0)
    '''
    E = (90., 90., 90.)   # this works: inverse Euler is (0, -90, 0)
    E = (-90., -90., -90.) # 
    E = (0.1, 0.2, 0.3) # small angles so inverse is approx negative of angles
    E = (0., 0., 0.)  # this works: important to look for singularities
    E = (180., 180., 180.) # this works: get back to beginning
    E = (180., 0., 180.) # this works
    E = (90., 0., 0.)
    '''
    E = (-30., 30., 45.) # Groves example 2.1(a, b, c): I get same results
                         # (Vec not in Groves ex)


    res = test(E=E, deg=True)
