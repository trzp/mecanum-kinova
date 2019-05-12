#coding:utf-8
# by mrtang on 20160519
from ctypes import *

(NOMOVEMENT_POSITION,CARTESIAN_POSITION,ANGULAR_POSITION,RETRACTED,
PREDEFINED1,PREDEFINED2,PREDEFINED3,CARTESIAN_VELOCITY,ANGULAR_VELOCITY,
PREDEFINED4,PREDEFINED5,ANY_TRAJECTORY,TIME_DELAY) = range(13)

(HAND_NOMOVEMENT,POSITION_MODE,ELOCITY_MODE) = range(3)

class CartesianInfo(Structure):
    _fields_ = [('X',c_float),('Y',c_float),('Z',c_float),
                ('ThetaX',c_float),('ThetaY',c_float),('ThetaZ',c_float)]
    def init(self):
        self.X = self.Y = self.Z = self.ThetaX = self.ThetaY = self.ThetaZ = 0.0

class AngularInfo(Structure):
    _fields_ = [('Actuator1',c_float),('Actuator2',c_float),('Actuator3',c_float),
                ('Actuator4',c_float),('Actuator5',c_float),('Actuator6',c_float)]
    def init(self):
        self.Actuator1 = self.Actuator2 = self.Actuator3 = 0.0
        self.Actuator4 = self.Actuator5 = self.Actuator6 = 0.0

class FingersPosition(Structure):
    _fields_ = [('Finger1',c_float),('Finger2',c_float),('Finger3',c_float)]
    def init(self):
        self.Finger1 = self.Finger2 = self.Finger3 = 0.0

class Limitation(Structure):
    # in SDK document, only speedParameter1 is effective.
    _fields_ = [('speedParameter1',c_float),('speedParameter2',c_float)]
    def init(self):
        self.speedParameter1 = 0.1
        self.speedParameter2 = 0.1

class KinovaDevice(Structure):
    _fields_ = [('SerialNumber',c_char*20),	('Model', c_char*20),('VersionMajor',c_int),('VersionMinor',c_int),
            ('VersionRelease',c_int),('DeviceType',c_int),('DeviceID',c_int)]

class CartesianPosition(Structure):
    _fields_ = [('Coordinates',CartesianInfo),('Fingers',FingersPosition)]
    def init(self):
        self.Coordinates.init()
        self.Fingers.init()

class UserPosition(Structure):
    _fields_ = [('Type',c_int),('Delay',c_float),('CartesianPosition',CartesianInfo),
            ('Actuators',AngularInfo),('HandMode',c_int),('Fingers',FingersPosition)]
    def init(self):
        self.Type = CARTESIAN_POSITION
        self.Delay = 0.0
        self.CartesianPosition.init()
        self.Actuators.init()
        self.HandMode = POSITION_MODE
        self.Fingers.init()

class TrajectoryPoint(Structure):
    _fields_ = [('Position',UserPosition),('LimitationsActive',c_int),('Limitations',Limitation)]
    def init(self):
        self.Position.init()
        self.LimitationsActive = 1	#could work for SendAdvanceTrajectory
        self.Limitations.init()

class AngularPosition(Structure):
    _fields_ = [('Actuators',AngularInfo),('Fingers',FingersPosition)]
    def init(self):
        self.Actuators.init()
        self.Fingers.init()