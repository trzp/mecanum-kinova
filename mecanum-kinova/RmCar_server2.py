#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time    : 2019/5/2 13:33
# @Version : 1.0
# @File    : demo_drivecar.py
# @Author  : Jingsheng Tang
# @Version : 1.0
# @Contact : mrtang@nudt.edu.cn   mrtang_cs@163.com
# @License : (C) All Rights Reserved

import pygame
from dragdraw_pgrect import DragDraw_pgRect
import os,sys
import numpy as np

from pykinect import KinectClientV2019
from pykinect import locate_obj,locate_bottle
from kcftracker_mul_pro import TrackerPro, tracker_pro
import multiprocessing
from multiprocessing import Queue,Event
from RmCar import RmCar_x64,RmCar_x86
import json


class rmRemoteClient():
    def __init__(self,hostaddr):
        self.s = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.s1 = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.s1.bind(hostaddr)
        self.s1.setblocking()

    def pushtask(self,cmd,mode):
        if mode == 'manul':
            buf = '##pushtask**manul**%s'%(cmd)
            self.s.sendto(buf,WC_ADDR)

        elif mode == 'auto':
            clas = cmd['name']
            center = cmd['position']
            box = np.asarray(cmd[box],dtype = np.int32)
            buf = '##pushtask**auto**%s**%s**%s'%(clas,center.astype(np.float32).tostring(),box.tostring())
            self.s.sendto(buf,WC_ADDR)
            
    def update_task(self):
        box = None
        try:
            buf,_ = self.s1.recvfrom(128)
            box = np.fromstring(buf,dtype=np.int32)
        except:
            pass
        return box

    def release(self):
        self.s.sendto('##quitpro')
    
        
class rmRemoteServer():
    def __init__(self,com=5):
        self.rm = RmCar_x86(com)
        self.s = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.s.bind(WC_ADDR)
        self.track = TrackerPro()
        tc_pro = multiprocessing.Process(target = tracker_pro, args = (self.track.args,))
        tc_pro.start()
        
        self.s1 = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        
    def main(self):
        going = 0
        while not END:
            try:
                buf,addr = self.s.recvfrom(128)
                cmds = buf.split('##')[-1]  #只接受最后一组命令
                b = cmd.split('**')
                if b[0] == 'pushtask':
                    if b[1] == 'manul':
                        cmd = b[2]
                        self.rm.pushtask(cmd,'manul')
                        print '\n>>> get manul task: %s'%(cmd)
                    elif b[1] == 'auto':
                        clas = b[2]
                        p = np.fromstring(b[3],dtype = np.float32)
                        x,y,w,h = box = np.fromstring(b[4],dtype = np.int32)
                        rec = ((x,y),(w,h))
                        self.track.init_tracker(box)
                        self.rm.pushtask({'class':clas,'center':p},'auto')
                        going = 1
                        print '>>> get auto task: %s'%(clas)
                    else:
                        pass
                elif b[0] == 'quitpro':
                    END = 1
                    self.rm.pushtask('stop','manul')
                    
            except:
                pass

            if going:
                box,pos = self.track.update()
                self.s1.sendto(np.asarray(box,dtype=np.int32).tostring(),MAIN_ADDR)

                if clas == 'bottle':
                    if pos['bottle'] is not None:   p = pos['bottle']
                else:
                    if pos['obj'] is not None:   p = pos['obj']

                if rm.updatetask(p):
                    going = 0
                    print 'task completed'
                    self.s1.sendto('**completed',self.BCI2000_addr)
                else:
                    pass

            fps_clock.tick(10)
        pygame.quit()

if __name__ == '__main__':
    s = rmRemoteServer()
    s.main()


