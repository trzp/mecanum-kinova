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
from kcftracker_mul_pro import Tracker_Pro, tracker_pro
import multiprocessing
from multiprocessing import Queue,Event
from RmCar import RmCar_x64

def demo():
    pygame.init()
    screen = pygame.display.set_mode((1280,480))
    END = False
    dr = DragDraw_pgRect(screen)
    kk = KinectClientV2019(True)
    fps_clock = pygame.time.Clock()

    rm = RmCar_x64()
    track = Tracker_Pro()
    tc_pro = multiprocessing.Process(target = tracker_pro, args = (track.args,))
    tc_pro.start()
    going = 0
    p = np.zeros(3)
    
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
                    name = raw_input('select succeed\nplease enter the target name: \n>>> ')
                    if name == 'bottle':
                        c = locate_bottle(rec,kk.point_cloud)
                    else:
                        c = locate_obj(rec, kk.point_cloud)
                    
                    cmd = {'name':name,'position':c}
                    rm.pushtask(cmd,'auto')
                    (x,y),(w,h) = rec 
                    going = 1
                    track.init_tracker((x,y,w,h))
                else:
                    print "select failure"
        if going:
            box,pos = track.update()
            if name == 'bottle':
                if pos['bottle'] is not None:   p = pos['bottle']
            else:
                if pos['obj'] is not None:   p = pos['obj']
            x,y,w,h = box
            pygame.draw.rect(screen,(0,255,0),((x,y),(w,h)),1)
            
            if rm.updatetask(p):
                going = 0
                print 'task completed'
            else:
                pass
        pygame.display.update()
        fps_clock.tick(10)
    del kk
    pygame.quit()

if __name__ == '__main__':
    demo()




