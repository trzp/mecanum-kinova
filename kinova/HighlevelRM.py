#coding:utf-8
from __future__ import division
import time
import numpy as np
import pygame
from copy import copy
import thread
from threading import Event

import os,sys
rootdir = os.path.dirname(__file__)
sys.path.append(rootdir)

from MICO_BASE import MICO2

#坐标系说明：
#世界坐标系：
#朝向轮椅前方 z+, 右侧 x+, 向下 y+
#机械臂坐标系：
#朝向轮椅前方：x+,左侧 y+，向上 z+

#定义配准中心
# 2019/5/2
WORK_HOME_POS_MICO = np.array([0.4782,-0.25757,0.1])      #机械臂在配准中心位置时的坐标（机械臂坐标系）
WORK_HOME_POS_WORLD = np.array([-0.064,0.555,1.056])      #机械臂在配准中心位置时的世界坐标

# 摄像头标定完成后实际测量的轮椅扶手正前，扶手左2/3，高度70厘米桌面向上40cm范围测量的世界坐标范围
# 不在这个范围的目标将直接放

class WorkSpace:  #世界坐标系
    Xrange = [-300,100]
    Yrange = [300,700]   #主要确定桌面高度
    Zrange = [84,1400]
    
    def inws(self,p):
        if self.__in(p[0],self.Xrange) and self.__in(p[1],self.Yrange) and self.__in(p[2],self.Zrange):    return True
        else: return False
        
    def __in(self,v,ls):
        if v > ls[0] and v< ls[1]:  return True
        else:   return False
        
# 演示的喝水动作
DRINK_ACTIONS = [ [[0.30,-0.3,0.24],[1.59,1.41,-0.06]],
                  [[-0.37,-0.3,0.24],[1.7,-0.04,0]],
                  [[-0.37,-0.3,0.24],[1.7,-0.04,0.5]],
                ]


class HLRM(MICO2):
    def __init__(self):
        '''
        :param ev: is used for other thread to check if the task is completed
        '''
        self.complete = Event()
        MICO2.__init__(self)
        self.goHome()
        self.WS = WorkSpace()
        print 'Mico has been initilized!!!!'

    def Kinect2workspace(self,pos):
        return pos/1000. - WORK_HOME_POS_WORLD

    def WorkSpace2MICO(self,pos):
        p = np.array([pos[2],-pos[0],-pos[1]])
        return p+WORK_HOME_POS_MICO

    def MICO2WorkSpace(pos):
        p = pos - WORK_HOME_POS_MICO
        return np.array([-p[1],-p[2],p[0]])

    def MICO2World(self,pos):
        return self.MICO2WorkSpace(pos)+WORK_HOME_POS_WORLD

    def goHome(self):
        self.MOVE(pos=np.array([0.2105,-0.26,-0.00625]),pos_ctr=np.zeros(3),theta=np.array([-2.63,1.558,-2.072]),theta_ctr=np.zeros(3),finger=np.array([1250,1250]),finger_ctr=np.zeros(2),block=1,vel=0.2)
        self.athome = True

    def adgust(self,pos):   #返回建议轮椅再x方向的调整量
        Xl,Xr = [0.001*item for item in self.WS.Xrange]    #坐标尺度统一
        if pos[0] > Xr or pos[0] < Xl:
            return 0.5*(Xl + Xr) - pos[0]   #向中心调整
        else:
            return 0

    def gripbottle(self,pos): #pos 世界坐标系注意单位
        pos[1] += 45   #定位点为瓶盖，适当下降定位位置
        if not self.WS.inws(pos):
            print 'not accessible position'
            return 1

        #mico坐标系
        #机械臂坐标系：
        #朝向轮椅前方：x+,左侧 y+，向上 z+
        self.complete.clear()
        wp = self.Kinect2workspace(pos)
        mp = self.WorkSpace2MICO(wp)

        # 先后退，确保不会撞到桌子边沿
        self.MOVE(pos = np.array([-0.05,0,mp[2]]),pos_ctr = np.array([1,1,0]),vel = 0.2)

        # 水平移动到正对点
        p1 = copy(mp)
        p1[0] = 0
        self.MOVE(pos=p1,pos_ctr=np.array([1,0,0]),vel=0.2)

        p2 = copy(mp)
        p2[0]-=0.06 #快速逼近
        self.MOVE(pos=p2,pos_ctr=np.zeros(3),vel=0.2)
        #缓慢逼近
        self.MOVE(pos=mp,pos_ctr=np.zeros(3))
        self.FINGER(type='grip',level=3)

        self.drink()    #演示喝水动作
        self.MOVE(pos=mp,pos_ctr=np.zeros(3))   #回到原来的位置
        self.FINGER(type='release',level=3)
        self.MOVE(pos=np.array([0,0,0.2]),pos_ctr=np.ones(3),vel=0.2)   #抬高再gohome，避免碰到桌子
        self.goHome()
        self.complete.set() #空闲
        print 'complete grip bottle'
        return 1

    def asy_gripbottle(self,pos):
        thread.start_new_thread(self.gripbottle,(pos,))

    def press_button(self,pos):
        #mico坐标系
        #机械臂坐标系：
        #朝向轮椅前方：x+,左侧 y+，向上 z+
        self.complete.clear()
        wp = self.Kinect2workspace(pos)
        mp = self.WorkSpace2MICO(wp)
        self.FINGER(type='grip',level=3)
        p1 = copy(mp)
        p1[0]=0     #首先运行到正对目标点位置，不前进（x方向不改变）
        self.MOVE(pos=p1,pos_ctr=np.array([1,0,0]),vel=0.2)
        p2 = copy(mp)
        p2[0]-=0.06 #快速逼近
        self.MOVE(pos=p2,pos_ctr=np.zeros(3),vel=0.2)
        #缓慢逼近
        mp[0]-=0.05 #点按示意
        self.MOVE(pos=mp,pos_ctr=np.zeros(3))
        self.goHome()
        self.complete.set()
        print 'complete press button'

    def drink(self):
        dk = copy(DRINK_ACTIONS)    #去喝水
        while len(dk)>0:
            p,t = dk.pop(0)
            self.MOVE(p,np.zeros(3),t,np.zeros(3),vel=0.15)
        
        time.sleep(2)
        
        dk = copy(DRINK_ACTIONS)    #原路返回并放下瓶子
        while len(dk)>0:
            p,t = dk.pop(-1)
            self.MOVE(p,np.zeros(3),t,np.zeros(3),vel=0.15)

if __name__ == '__main__':
    demo1()