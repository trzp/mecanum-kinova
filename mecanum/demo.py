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
import socket
import numpy as np

from pykinect impor KinectClientV2019
from pykinect import locate_obj
from tracker_pro import track_pro
import multiprocessing
from multiprocessing import Queue,Event

def demo():
    pygame.init()
    screen = pygame.display.set_mode((1280,480))
    END = False
    dr = DragDraw_pgRect(screen)
    kk = KinectClientV2019()
    fps_clock = pygame.time.Clock()

    s = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    addr = ('127.0.0.1',9097)

    going = 0
    qqin = Queue()
    qqout = Queue()
    qqinf = Queue()
    ee = Event()

    q = multiprocessing.Process(target = track_pro,args = (qqin,qqout,ee,qqinf))
    q.start()

    while not END:
        screen.blit(kk.get_color_as_pgsurface(),(0,0))
        screen.blit(kk.get_depth_as_pgsurface(),(640,0))
        events = pygame.event.get()
        flg,rec = dr.update(events)

        for ev in events:
            if ev.type == pygame.QUIT:
                END = True
            elif ev.type == pygame.KEYUP and ev.key == 13:  # enter key
                if flg:
                    print 'you have seleted the area: %s'%(str(rec))
                    c = locate_obj(rec, kk.point_cloud)
                    print str(c)
                    s.sendto('##pushtask**auto**desk**%s'%(c.astype(np.float32).tostring()),addr)
                    b,_ = s.recvfrom(256)
                    going = 1
                    box = [rec[0][0],rec[0][1],rec[1][0],rec[1][1]]
                    qqin.put([1,box])
                else:
                    print "you havn't selected any region"
        if going:
            qqin.put([2,0])
            b,f,p = qqout.get()
            x,y,w,h = b
            pygame.draw.rect(screen,(0,255,0),((x,y),(w,h)),1)
            s.sendto('##updatetask**%s'%(p.astype(np.float32).tostring()),addr)
            buf,addr = s.recvfrom(128)
            if buf == '1':
                going = 0
                print 'task completed'
            else:
                pass
                
        pygame.display.update()
        fps_clock.tick(10)
    kk.release()
    pygame.quit()

if __name__ == '__main__':
    demo()




