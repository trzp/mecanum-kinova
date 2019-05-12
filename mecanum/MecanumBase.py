#coding:utf-8
from __future__ import division
from struct import pack,unpack
import numpy as np
import numpy.linalg as npla

class MecanumBase():
    '''
    author:mrtang
    date:2015.8
    version:3.0
    email:mrtang@nudt.edu.cn

    note:
    The unit of AngleVel in stm32 chip is changed to 0.1rad/s.
    However, in the application level the unit of v is still rad/s.
    In another word, in application functions, the smallest v could be 0.1. 
    The charge port is on the back of Platform.
    '''

    def __encode__(self,vel,angle,angle_v,angle_vd):
        if vel<0:	vel = 0
        vel = int(vel)%3000
        angle_v = abs(int(angle_v))
        angle = abs(int(angle))%360		# safty parameters

        Lv = vel%256				#low byte of vel
        Hv = (vel>>8)%256			#hight byte of vel
        La = angle%256				#low byte of angle
        Ha = angle>>8				#hgih byte of angle
        av = angle_v%256			#angle vel. unit 0.1degree/s 
        avd = abs(angle_vd)%2		#direction of angle vel. 0 counter clockwise, 1 clockwise
        check = 255-(161+Lv+Hv+La+Ha+av+avd)%256	#check byte

        cmd = np.array([85,170,30,253,8,161,Lv,Hv,La,Ha,av,avd,0,check],dtype=np.uint8)
        buffer = cmd.tostring()
        return buffer

    def calib_v(self,v):  #标定平移速度
        return 0.815*v

    def calib_av(self,v): #标定旋转速度
        # return 0.78*v
        return 0.8*v

    def stop(self):
        return self.__encode__(0,0,0,0)

    def translate(self,pos,v=30):   #right:x,forward:y
        v = abs(v)
        distance = npla.norm(pos)
        if distance>0:
            u_dest = pos/distance
            angle = np.arccos(u_dest[1])*180/np.pi
            if u_dest[0]>0: angle = 360-angle
        else:
            angle = 0
        buf = self.__encode__(self.calib_v(v),angle,0,0)
        return buf,distance/v

    def translateV(self,d=0,v=30,): #v:mm/s d: 0-forward 90 left 180 back 270 right
        return self.__encode__(self.calib_v(abs(v)),d,0,0)

    def rotate_dir(self,v=1): #度/s positive->clockwise
        if v<0:		d = 0
        elif v>0:	d = 1
        else:	return self.__encode__(0,0,0,0)
        return self.__encode__(0,0,abs(v)*10,d)

    def rotate(self,orient=np.array([0,1]),v=1):
        v = abs(v)
        u_orient = orient/npla.norm(orient)
        angle = np.arccos(u_orient[1])*180/np.pi 
        ad = (np.sign(u_orient[0])+1)/2
        return self.__encode__(0,0,self.calib_av(v)*10,ad),abs(angle)/v

    def set_channel(self,channel='wireless'):
        if channel == 'wireless':  cmd = np.array([85,170,30,253,8,187,0,0,0,0,0,0,0,68],dtype=np.uint8)
        else:   cmd = np.array([85,170,30,253,8,188,0,0,0,0,0,0,0,67],dtype=np.uint8)
        return cmd.tostring()