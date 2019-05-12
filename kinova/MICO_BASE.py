# coding:utf-8
from __future__ import division
from MICO_TYPES import *
from math import pi
import numpy as np
import time

class MICO2:
    '''
        This class is developped for MICO2 driving by calling SDK API using ctypes package.
        developped by mrtang on 2016.05.19
        updated by mrtang on 2016.11.20
        email:mrtang@nudt.edu.cn
    '''
    def __init__(self):
        KDLL = CDLL('CommandLayerWindows.dll')
        self.__InitAPI = KDLL.InitAPI;
        self.__StartControlAPI = KDLL.StartControlAPI;
        self.__StopControlAPI = KDLL.StopControlAPI;
        self.__GetDevices = KDLL.GetDevices;
        self.__SetActiveDevice = KDLL.SetActiveDevice;
        self.__MoveHome = KDLL.MoveHome;
        self.__InitFingers = KDLL.InitFingers;
        self.__EraseAllTrajectories = KDLL.EraseAllTrajectories;
        self.__GetCartesianPosition = KDLL.GetCartesianPosition;
        self.__GetGripperStatus = KDLL.GetGripperStatus;
        self.__SendAdvanceTrajectory = KDLL.SendAdvanceTrajectory;
        self.__SendBasicTrajectory = KDLL.SendBasicTrajectory;
        self.__GetAngularVelocity = KDLL.GetAngularVelocity
        self.__GetCartesianForce = KDLL.GetCartesianForce
        self.__CloseAPI = KDLL.CloseAPI;

        if 	((self.__InitAPI == None)				|	(self.__StartControlAPI == None)		|	(self.__GetDevices == None)				|
            (self.__SetActiveDevice == None)		|	(self.__MoveHome == None)				|	(self.__InitFingers == None)			|
            (self.__InitFingers == None)			|	(self.__EraseAllTrajectories == None)	|	(self.__GetCartesianPosition == None)	|
            (self.__GetGripperStatus == None)		|	(self.__CloseAPI == None)				|	(self.__SendAdvanceTrajectory == None)	|
            (self.__StopControlAPI == None)			|	(self.__SendBasicTrajectory == None)    |   (self.__GetAngularVelocity==None)       |
            (self.__GetCartesianForce == None)):
            raise ImportError,'SDK import error!!!!'

        API_HANDL = c_int(self.__InitAPI())
        self.__StartControlAPI()
        curDevice = KinovaDevice()
        KList = (KinovaDevice*20)()
        DeviceCount = self.__GetDevices(KList,byref(API_HANDL))
        if DeviceCount:
            curDevice = KList[0]
            if not self.__SetActiveDevice(curDevice):		raise IOError,'Device active failure!!!!'
            if not self.__MoveHome():                       raise IOError,'Device movehome failure!!!!'
            if not self.__InitFingers():					raise IOError,'Fingers init failure!!!!'
        else:
            raise IOError,"could't Found any Devices!!!"
        self.FINGER('release')
        print '-/--\--/--\--/--\--/--\--/--\--/--\--/--\---'

    @property
    def POSITION(self):
        pos = CartesianPosition()
        self.__GetCartesianPosition(byref(pos))
        p = np.array([pos.Coordinates.X,pos.Coordinates.Y,pos.Coordinates.Z])
        t = np.array([pos.Coordinates.ThetaX,pos.Coordinates.ThetaY,pos.Coordinates.ThetaZ])
        f = np.array([pos.Fingers.Finger1,pos.Fingers.Finger2])
        return [p,t,f]

    def WAITTING(self,SendPoint,timeout=100):
        '''	Block the program until the mico is complete the current action.
        This function will run after a delay which defined by timeout parameter.
        It is necessary since the delay is always exist in robot starting.
        timeout: ms
        '''
        if timeout>1:
            time.sleep(timeout/1000.)
            tems = np.zeros(8)
            L = 1
            while L > 0.00005:
                self.__SendAdvanceTrajectory(SendPoint)
                p = self.POSITION
                k = np.append(np.append(p[0],(0.5/pi)*p[1]),(1/7000.)*p[2])
                L = np.sum(np.abs(k-tems))
                tems = k
                time.sleep(0.15)

    def MOVE(self,pos=np.zeros(3),pos_ctr=np.ones(3),theta=np.zeros(3),theta_ctr=np.ones(3),finger=np.zeros(2),finger_ctr=np.ones(2),block=1,vel=0.05):
        '''	CartesianPosition mode.
        Move mico to a expected position (defined by x,y,z,thetax,thetay,thetaz,finger1,finger2).
        dp: x(m),y(m),z(m),thetax(rad),thetay(rad),thetaz(rad),finger1(0-6800),finger2(0-6800)
        ctr:	np.array with size 8
                each bit could be 1/0 correspond to the bits in dp
                1:	this bit would be a value related to the current position
                0:	this bit would be the real postion
        block:	1/0  1:return atfter robot complete the task, 0:return immediately 
        vel:	m/s. the vel for robot running. The maximum of 0.2m/s is supportted.
        block: 0 return immediately
               1 block until action finished
        '''
        self.__EraseAllTrajectories()
        SendPoint = TrajectoryPoint()
        SendPoint.init()
        SendPoint.LimitationsActive = 0
        if vel<0:	vel = 0.05 
        if vel>0.2:	vel = 0.2
        if vel>0.05:    SendPoint.LimitationsActive = 1
        SendPoint.Limitations.speedParameter1 = vel
        SendPoint.Limitations.speedParameter2 = vel

        cpos,ctheta,cfinger = self.POSITION
        npos = cpos*pos_ctr+pos 
        ntheta = ctheta*theta_ctr+theta
        nfinger = cfinger*finger_ctr+finger

        SendPoint.Position.CartesianPosition.X = npos[0]
        SendPoint.Position.CartesianPosition.Y = npos[1]
        SendPoint.Position.CartesianPosition.Z = npos[2]
        SendPoint.Position.CartesianPosition.ThetaX = ntheta[0]
        SendPoint.Position.CartesianPosition.ThetaY = ntheta[1]
        SendPoint.Position.CartesianPosition.ThetaZ = ntheta[2]

        SendPoint.Position.Fingers.Finger1 = nfinger[0]
        SendPoint.Position.Fingers.Finger2 = nfinger[1]

        r = self.__SendAdvanceTrajectory(SendPoint)
        if block:    self.WAITTING(SendPoint,150)
        return self.POSITION

    def FINGER(self,type='release',level=3,block=1):
        ctr = [0,2000,4000,6500]
        if type=='release':
            p = 0
        elif type=='grip':
            level = int(level)
            if level not in [1,2,3]:   level = 1
            p = ctr[level]
        self.MOVE(finger=np.array([p,p]),finger_ctr=np.zeros(2),block=block)

    @property
    def TORQUE(self):
        '''get the force in Cartesian coordinate.'''
        torque = CartesianPosition()
        self.__GetCartesianForce(byref(torque))
        return [torque.Coordinates.X,torque.Coordinates.Y,torque.Coordinates.Z]

    def __del__(self):
        self.__StopControlAPI()
        self.__CloseAPI()


def demo1():
    from msvcrt	 import getch,kbhit
    m = MICO2()
    m.MOVE(pos=np.array([0.2103,-0.2628,0.477]),pos_ctr=np.zeros(3),theta=np.array([-2.3179,1.554,-2.392]),theta_ctr=np.zeros(3),finger=np.array([1250,1250]),finger_ctr=np.zeros(2),block=1,vel=0.2)
    m.MOVE(pos=np.array([0.2105,-0.26,-0.00625]),pos_ctr=np.zeros(3),theta=np.array([-2.63,1.558,-2.072]),theta_ctr=np.zeros(3),finger=np.array([1250,1250]),finger_ctr=np.zeros(2),block=1,vel=0.2)

def demo2():
    from msvcrt import kbhit
    m = MICO2()
    import time
    while True:
        print m.POSITION
        time.sleep(0.5)
        if kbhit():break
        
if __name__ == "__main__":
    demo2()