#!user/bin/python
# -*-coding:utf-8-*-

#FileName: RmCar.py
#Version: 1.0
#Author: Jingsheng Tang
#Date: 2017/9/17
#Email: mrtang@nudt.edu.cn
#Git: trzp

from MecanumPro import H_MecanumPro
from HighlevelRM import HLRM
from pykinect import KinectClientV2019

import numpy as np
import socket
import subprocess

import os,sys
rootdir = os.path.dirname(os.path.abspath(__file__))
updir = os.path.dirname(rootdir)
sys.path.append(updir)

from params import RMCAR_HOST_ADDR

class RmCar_x86
    def __init__(self,mec_com):
        self.mec = H_MecanumPro(mec_com)
        self.mec.set_control_channel('wire')
        self.rm = HLRM()
        self.dev_que = []

    def pushtask(self,cmd,mode):    #external call
        if mode == 'manul':
            if cmd == 'left':       self.mec.gettask({'op':'TranslateV','orient':np.array([-1,0]),'mode':'manul'})
            elif cmd == 'right':    self.mec.gettask({'op':'TranslateV','orient':np.array([1,0]),'mode':'manul'})
            elif cmd == 'rleft':    self.mec.gettask({'op':'RotateV','orient':np.array([-1,1]),'mode':'manul'})
            elif cmd == 'rright':   self.mec.gettask({'op':'RotateV','orient':np.array([1,1]),'mode':'manul'})
            elif cmd == 'backward': self.mec.gettask({'op':'TranslateV','orient':np.array([0,-1]),'mode':'manul'})
            elif cmd == 'forward':  self.mec.gettask({'op':'TranslateV','orient':np.array([0,1]),'mode':'manul'})
            elif cmd == 'stop':     self.mec.gettask({'op':'stop','mode':'manul'})
            self.dev_que = []
        elif mode == 'auto':
            self.mec.gettask({'op':cmd,'mode':'auto'})
            self.dev_que = ['mec']
            if cmd['class']=='bottle':  self.dev_que.append('mico')
    
    def updatetask(self,obj,pointcloud):
        if len(self.dev_que)!=0:
            dev = self.dev_que[0]
            if dev=='mec':
                if self.mec.updatetask(obj,pointcloud): 
                    self.dev_que.pop(0)
                    if len(self.dev_que)>0 and self.dev_que[0]=='mico':
                        if obj == None:self.dev_que.pop(0)
                        else:self.rm.asy_gripbottle(obj['center'])
            elif dev=='mico':
                if self.rm.complete.isSet():  self.dev_que.pop(0)
        else:   return True
        return False

    def cleartask(self):
        self.dev_que = []
        self.rm.goHome()
        
class RmCar_x64:
    def __init__(self,external_start_server = False):   #允许从外部启动服务
        if not external_start_server:
            subprocess.Popen('python2 E:\MobileRobot\RmCar\RmCar_server.py') #启动服务

        self.s = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    
    def pushtask(self,cmd,mode):
        if mode == 'manul':
            buf = '##pushtask**manul**%s'%(cmd)
            self.s.sendto(buf,RMCAR_HOST_ADDR)
            self.s.recvfrom(128)
        elif mode == 'auto':
            clas = cmd['name']
            center = cmd['position']
            buf = '##pushtask**auto**%s**%s'%(clas,center.astype(np.float32).tostring())
            self.s.sendto(buf,RMCAR_HOST_ADDR)
            self.s.recvfrom(128)

    def updatetask(self,pos):
        buf = pos.astype(np.float32).tostring()
        self.s.sendto('##updatetask**%s'%(buf),RMCAR_HOST_ADDR)
        buf,addr = self.s.recvfrom(128)
        if int(buf):    return True
        else:   return False
    
    def cleartask(self):
        self.s.recvfrom(128)
        
    def release(self):
        self.s.sendto('##quitpro')

if __name__ == '__main__':
    rm = RmCar_x64()
    rm.release()




