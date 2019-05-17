#!user/bin/python
# -*-coding:utf-8-*-

#FileName: MecanumPro.py
#Version: 1.0
#Author: Jingsheng Tang
#Date: 2017/8/25
#Email: mrtang@nudt.edu.cn
#Github: trzp

from __future__ import division
from MecanumBase import MecanumBase
from serial import Serial
import threading
from threading import Event
import numpy as np
import time
from copy import copy
import numpy.linalg as npla

LIMITDIS = 840
CENTERDIS = 200
NOTSELF = 800

class MecanumPro(threading.Thread,MecanumBase):
    def __init__(self, mec_com):
        if type(mec_com) == int or type(mec_com) == float:
            mec_com = 'COM%d'%(mec_com)
        self.ser = Serial(mec_com, 115200)
        self.__close = False
        self.working_queue = []
        self.planning_queue = []
        self.running = False
        self.task_ID = 0
        self.monitor = {'quit':Event()}
        threading.Thread.__init__(self)
        self.start()

    def set_control_channel(self,channel='wireless',wait=False): #wireless or wire
        if not wait:
            self.working_queue = []
            self.planning_queue = []

        self.task_ID+=1
        self.planning_queue.append({'buf':self.set_channel(channel),'duration':0.2,'ID':str(self.task_ID)})
        self.monitor[str(self.task_ID)]=Event()   #为每一个任务对应注册一个事件，便于观察任务执行情况

    def quit(self,wait=False):
        if not wait:
            self.working_queue = []
            self.planning_queue = []

        self.planning_queue.append({'buf':self.stop(),'duration':0.2,'ID':'quit'})

    def Stop(self):
        self.working_queue = []
        self.planning_queue = []

    def Goto(self,pos=np.zeros(2),orient=np.array([0,1]),tvel=30,rvel=1,asy_mode=True,wait=True):
        if not wait:
            self.working_queue = []
            self.planning_queue = []
            
        self.task_ID+=1
        buf,duration = self.translate(pos,tvel)
        if duration==0: buf,duration = self.rotate(orient,rvel)
        self.planning_queue.append({'buf':buf,'duration':duration,'ID':str(self.task_ID)})
        ev = Event()
        self.monitor[str(self.task_ID)]= ev  #为每一个任务对应注册一个事件，便于观察任务执行情况
        if not asy_mode:
            ev.wait()
            del self.monitor[str(self.task_ID)]
            return 0
        else:   return str(self.task_ID)

    def TranslateV(self,orient=np.array([0,1]),tvel=30,wait=True):
        if not wait:
            self.working_queue = []
            self.planning_queue = []
        buf,duration = self.translate(orient,tvel)
        if duration!=0: self.planning_queue.append({'buf':buf,'duration':float('inf')})

    def RotateV(self,orient=np.array([0,1]),rvel=1,wait=True):
        if not wait:
            self.working_queue = []
            self.planning_queue = []
        buf,duration = self.rotate(orient,rvel)
        if duration!=0: self.planning_queue.append({'buf':buf,'duration':float('inf')})

    def update_buf(self):
        if len(self.working_queue)==0:
            try:
                cmd = self.planning_queue.pop(0)
                cmd['endclk']=cmd['duration']+time.clock()
                self.working_queue.append(cmd)
                return cmd['buf']
            except:
                return self.stop()
        else:
            cmd = self.working_queue[0]
            if time.clock()>cmd['endclk']:
                self.monitor[cmd['ID']].set()
                self.working_queue = []
                return self.stop()
            else:
                return cmd['buf']

    def run(self):
        while True:
            self.ser.write(self.update_buf())
            if self.monitor['quit'].isSet():    break
            time.sleep(0.1)
        self.ser.close()
        print 'working thread killed succeed!'

class H_MecanumPro(MecanumPro):
    def __init__(self, mec_com):
        MecanumPro.__init__(self,mec_com)
        self.ENV = EnvMeasure()
        self.high_level_que = []
        self.running = False
        self.current_cmd = None
        self.cmd_ID = 0
        
        self.error_clocks = []  #记录无法探测到深度信息的危险情况

    def gettask(self,cmd):
        if cmd['mode']=='manul':    #不进入任务序列，被立即执行并且立即返回，等待新的任务。
            self.high_level_que = []
            if cmd['op']=='TranslateV': self.TranslateV(orient=cmd['orient'],tvel=35,wait=False)
            elif cmd['op']=='RotateV':  self.RotateV(orient=cmd['orient'],rvel=2.5,wait=False)
            else:                       self.Stop()
        elif cmd['mode']=='auto':
            obj = cmd['op']
            c = obj['center']
            vec0 = copy(c[[0,2]]) #取平面坐标
            vec1 = copy(vec0)
            vec1[1]-= LIMITDIS    #以轮椅前端为参考点
            vec0[1]-= CENTERDIS    #以轮椅回转中心为坐标为参考点

            distance = npla.norm(vec1)
            self.high_level_que = []
            if distance < 1200:     #一米范围内，直接平移到达目标
                self.high_level_que.append({'type':'move','pos':vec1,'closeloop':True})
            else:
                self.high_level_que.append({'type':'rotate','orient':vec0,'closeloop':False})
                self.high_level_que.append({'type':'move','pos':np.array([0,npla.norm(vec1)]),'closeloop':True})
            if obj['class']in['desk','bottle']:
                self.high_level_que.append({'type':'angle_adjust','closeloop':True})
            if obj['class']=='bottle':
                self.high_level_que.append({'type':'grip_adjust','closeloop':True})
                self.high_level_que.append({'type':'close_to','closeloop':True})

    def updatetask(self,obj,pointcloud):
        if not self.running:
            if len(self.high_level_que)==0: 
                self.Stop()
                return True
            else:
                self.current_cmd = self.high_level_que.pop(0)
                self.running = True #还有任务需要完成
                if not self.current_cmd['closeloop']:   #开环的任务
                    if self.current_cmd['type']=='move':    self.cmd_ID = self.Goto(pos=self.current_cmd['pos'],wait=False)
                    elif self.current_cmd['type']=='rotate':self.cmd_ID = self.Goto(orient=self.current_cmd['orient'],wait=False)
        else:
            if self.current_cmd['closeloop']:
                if self.close_control(pointcloud,self.current_cmd,obj):self.running = False
            else:
                if self.monitor[self.cmd_ID].isSet():
                    del self.monitor[self.cmd_ID]
                    self.running = False    #当前命令已经开环完成
        return False
    
    def close_control(self,pointcloud,cmd,obj):
        flg,ang,orient,dis = self.ENV.measure(pointcloud)
        print dis
        if flg:
            if cmd['type']=='move':
                if dis>400:    self.TranslateV(cmd['pos'],tvel=50,wait=False)
                else:
                    self.Stop()
                    return True

            elif cmd['type']=='angle_adjust':
                if abs(ang)>3:  #角度过大，需要调整
                    if dis<100:   self.TranslateV(np.array([0,-1]),wait=False,tvel=5)
                    else:   #前方距离合理
                        if abs(ang)>8:  self.RotateV(np.array([np.sign(-ang),1]),wait=False,rvel=2)
                        elif abs(ang)>3:self.RotateV(np.array([np.sign(-ang),1]),wait=False,rvel=0.5)
                else:
                    self.Stop()
                    return True

            elif cmd['type']=='grip_adjust':
                if obj is None:
                    self.Stop()
                    self.high_level_que = []
                    print 'aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa'
                    return True

                pos = obj['center'][[0,2]]
                temp = pos[0]+70    #水平方向的调整
                if abs(temp)>50:                     
                    if dis<100: #保证安全的调整
                        self.TranslateV(np.array([0,-1]),wait=False,tvel=5)
                    else:   #正在进行水平方向调整
                        if abs(temp)>100:
                            self.TranslateV(np.array([temp,0]),tvel=25,wait=False)
                        else:
                            self.TranslateV(np.array([temp,0]),tvel=5,wait=False)
                else:
                    self.Stop()
                    return True

            elif cmd['type']=='close_to':
                if dis>50:self.TranslateV(np.array([0,1]),tvel=15,wait=False)
                else:
                    self.Stop()
                    return True
        else:
            print 'warning'
        return False
    
    
    
    
    
    def close_control1(self,pointcloud,cmd,obj):
        flg,ang,orient,dis = self.ENV.measure(pointcloud)
        
        if flg:
            self.error_clocks = []  #正常情况下清空
            
            if cmd['type']=='move':
                if dis>400:    self.TranslateV(cmd['pos'],tvel=50,wait=False)
                else:
                    self.Stop()
                    return True

            elif cmd['type']=='angle_adjust':
                print 'angle adjust'
                if abs(ang)>3:  #角度过大，需要调整
                    if dis<100:   self.TranslateV(np.array([0,-1]),wait=False,tvel=5)
                    else:   #前方距离合理
                        if abs(ang)>8:  self.RotateV(np.array([np.sign(-ang),1]),wait=False,rvel=2)
                        elif abs(ang)>3:self.RotateV(np.array([np.sign(-ang),1]),wait=False,rvel=0.5)
                else:
                    self.Stop()
                    return True

            elif cmd['type']=='grip_adjust':
                print 'grip_adjust'
                if obj == None:
                    self.Stop()
                    self.high_level_que = []
                    return True

                pos = obj['center'][[0,2]]
                temp = pos[0]+70    #水平方向的调整
                if abs(temp)>50:                     
                    if dis<100: #保证安全的调整
                        self.TranslateV(np.array([0,-1]),wait=False,tvel=5)
                    else:   #正在进行水平方向调整
                        if abs(temp)>100:
                            self.TranslateV(np.array([temp,0]),tvel=25,wait=False)
                        else:
                            self.TranslateV(np.array([temp,0]),tvel=5,wait=False)
                else:
                    self.Stop()
                    return True

            elif cmd['type']=='close_to':
                print 'close to'
                if dis>100:self.TranslateV(np.array([0,1]),tvel=15,wait=False)
                else:
                    self.Stop()
                    return True
            else:
                pass
        
        else:   #有可能存在某些帧探测不到深度信息等
            self.Stop()     #暂停机器
            self.error_clocks.append(time.clock())  #记录无法探测到深度信息的危险情况
            if len(self.error_clocks)>2 and (self.error_clocks[-1] - self.error_clocks[0] > 5): #连续5秒得不到正常的测量信息
                self.high_level_que = []
                return True                 #终止当前任务

        return False    #还没有完成

class EnvMeasure:
    def measure(self,pointcloud,scale = 25):
        inds = np.where((pointcloud[:,:,0]>-400) & (pointcloud[:,:,0]<400) & (pointcloud[:,:,2]> NOTSELF))
        try:
            flg = 1
            points = pointcloud[inds]   #切割了轮椅正前方区域的点云
            # 滤除噪点
            while True:
                zz = points[:,2]
                onum = points.shape[0]
                minzz = np.min(zz)
                ind = np.where(np.fabs(zz - minzz))[0]
                if ind.size < 10:
                    points = np.delete(points,ind)
                    print '[EnvMeasure] find some noise'
                num = points.shape[0]
                if num == onum: break
                

            distance = np.min(points[:,2]) - LIMITDIS #距离障碍物最短距离
            print distance
            if distance < 0:    flg = 0
            linspace = np.arange(-400,401,scale)
            num = linspace.size-1
            k = [self.__sample(points,linspace,i) for i in range(num)]  #对点云求边缘
            k = np.array(k)
            angs = np.array([self.__measure(k,i) for i in range(num-10)])
            ang = np.mean(angs[np.where(angs<360)[0]])
            return flg,ang,np.array([np.sin(ang*np.pi/180),np.cos(ang*np.pi/180)]),distance
        except:
            return 0,0,0

    def __sample(self,planepoints,linspace,i): #采样
        inds = np.where((planepoints[:,0]>linspace[i]) & (planepoints[:,0]<linspace[i+1]))[0]
        if inds.size > 0:
            tem = np.min(planepoints[inds,2])
        else:
            tem = 0
        return linspace[i],tem

    def __measure(self, edge ,i, scale = 10):
        pp = edge[i:i+scale,:]
        coe,red = np.polyfit(pp[:,0],pp[:,1],1,full=True)[0:2]
        if red < 200:
            ang = np.arctan(coe[0])*180/np.pi
        else:
            ang = 360
        return ang

if __name__ == '__main__':
    m = MecanumPro(3)
    m.set_control_channel('wireless')
    ID1 = m.Move(np.array([-1000,-1000]))
    ID2 = m.Rotate(np.array([0.5,0.5]))
    while True:
        if m.monitor[ID1].isSet():  print 'complete move'
        if m.monitor[ID2].isSet():  print 'complete rote'
        if m.monitor[ID1].isSet() and m.monitor[ID2].isSet():
            m.quit()
            break
