#coding:utf-8
from pykinect import KinectClientV2019
from pykinect import locate_bottle
from dragdraw_pgrect import DragDraw_pgRect
from HighlevelRM import HLRM
import pygame


def demo():
    rm = HLRM()
    pygame.init()
    screen = pygame.display.set_mode((1280,480))
    END = False
    dr = DragDraw_pgRect(screen)
    kk = KinectClientV2019()

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
                    c = locate_bottle(rec,kk.point_cloud)
                    print 'bottle location: ' + str(c)
                    rm.gripbottle(c)
                else:
                    print "you havn't selected any region"
        pygame.display.update()
    kk.release()
    pygame.quit()
    
if __name__ == '__main__':
    demo()