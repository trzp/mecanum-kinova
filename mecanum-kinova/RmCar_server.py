#!user/bin/python
# -*-coding:utf-8-*-

#FileName: RmCar.py
#Version: 1.0
#Author: Jingsheng Tang
#Date: 2017/9/17
#Email: mrtang@nudt.edu.cn
#Git: trzp

from pykinect import KinectClientV2019
import numpy as np
import socket
import subprocess
from RmCar import RmCar_x86
import os,sys
rootdir = os.path.dirname(os.path.abspath(__file__))
updir = os.path.dirname(rootdir)
sys.path.append(updir)

from mr_params import WC_ADDR

def RmCar_Server():
    try:    #如果网络端口被占用，则说明已经启动了服务，无需再启动
        s = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        s.bind(WC_ADDR)
        rm = RmCar_x86(5)
    except:
        return
        
    print '[smart mobile robot server] running...'

    kinect = KinectClientV2019()
    ret = '0'
    END = False
    while not END:
        buf,addr = s.recvfrom(256)
        cmds = buf.split('##')
        for cmd in cmds:
            b = cmd.split('**')
            if b[0] == 'pushtask':
                if b[1] == 'manul':
                    cmd = b[2]
                    rm.pushtask(cmd,'manul')
                    print '\n>>> get manul task: %s'%(cmd)
                elif b[1] == 'auto':
                    clas = b[2]
                    p = np.fromstring(b[3],np.float32)
                    rm.pushtask({'class':clas,'center':p},'auto')
                    print '>>> get auto task: %s'%(clas)
                else:
                    pass
                s.sendto('0', addr)
            elif b[0] == 'updatetask':
                p = np.fromstring(b[1],np.float32)
                if rm.updatetask({'center':p},kinect.point_cloud):
                    s.sendto('1', addr)
                    print '>>> task completed'
                else:
                    s.sendto('0', addr)
            elif b[0] == 'stop':
                rm.dev_que = []
                rm.pushtask('stop','manul')
                rm.rm.goHome()
                print '>>> stop running'
                s.sendto('0', addr)
            elif b[0] == 'quitpro':
                END = True
                s.sendto('ok', addr)
            elif b[0] == 'request':
                s.sendto('running',addr)
            elif b[0] == 'clear':
                rm.cleartask()
                s.sendto('0', addr)
            else: pass
            
if __name__ == '__main__':
    RmCar_Server()