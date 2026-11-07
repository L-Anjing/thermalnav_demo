"""IntentNav-style egocentric BEV renderers from the shared global map."""
import cv2
import numpy as np

class BEVRenderer:
    UNKNOWN=(128,128,128);FREE=(255,255,255);OCCUPIED=(0,0,0)
    TARGET=(0,145,255);FRONTIER=(0,200,0);SELECTED=(0,255,255);ROBOT=(0,0,255);TRAJECTORY=(255,0,0)
    def __init__(self,pixels=448,meters=22.4):
        self.pixels=pixels;self.meters=meters;self.res=meters/pixels;self.center=pixels//2
    def pixel(self,xy,robot):
        d=(np.asarray(xy)-np.asarray(robot))/self.res
        return (int(round(self.center+d[0])),int(round(self.center-d[1])))
    def _crop(self,bev,robot):
        size=self.pixels;center=bev.cell(robot);half=round(self.meters/(2*bev.res))
        occ=bev.occupancy();x0=center[0]-half;y0=center[1]-half
        # Unknown padding is explicit; it is never rendered as free.
        out=np.full((2*half,2*half),-1,np.int8)
        ax0=max(0,x0);ay0=max(0,y0);ax1=min(occ.shape[1],x0+2*half);ay1=min(occ.shape[0],y0+2*half)
        if ax1>ax0 and ay1>ay0:out[ay0-y0:ay1-y0,ax0-x0:ax1-x0]=occ[ay0:ay1,ax0:ax1]
        out=cv2.flip(out,0)
        return cv2.resize(out,(size,size),interpolation=cv2.INTER_NEAREST)
    def render(self,bev,robot,yaw,trajectory=(),candidates=(),selected=None,targets=(),fov_deg=47.,max_range=8.):
        grid=self._crop(bev,robot);rgb=np.empty((self.pixels,self.pixels,3),np.uint8)
        rgb[:]=self.UNKNOWN;rgb[grid==0]=self.FREE;rgb[grid==100]=self.OCCUPIED
        for a,b in zip(trajectory,trajectory[1:]):cv2.line(rgb,self.pixel(a,robot),self.pixel(b,robot),self.TRAJECTORY,2,cv2.LINE_AA)
        for c in candidates:
            if c.kind=='frontier':color=self.FRONTIER
            elif c.kind=='verify_target':color=self.TARGET
            else:color=(0,180,220)
            cv2.circle(rgb,self.pixel(c.xy,robot),5,color,-1,cv2.LINE_AA)
        if selected is not None:cv2.circle(rgb,self.pixel(selected,robot),8,self.SELECTED,2,cv2.LINE_AA)
        for t in targets:cv2.circle(rgb,self.pixel(t['xy'],robot),5,self.TARGET,-1,cv2.LINE_AA)
        c=self.center;length=int(max_range/self.res);a=np.deg2rad(fov_deg/2)
        p1=(int(c+length*np.cos(yaw-a)),int(c-length*np.sin(yaw-a)))
        p2=(int(c+length*np.cos(yaw+a)),int(c-length*np.sin(yaw+a)))
        cv2.line(rgb,(c,c),p1,(0,210,255),1,cv2.LINE_AA);cv2.line(rgb,(c,c),p2,(0,210,255),1,cv2.LINE_AA);cv2.ellipse(rgb,(c,c),(5,5),0,0,360,(0,210,255),1,cv2.LINE_AA)
        cv2.arrowedLine(rgb,(c,c),(int(c+17*np.cos(yaw)),int(c-17*np.sin(yaw))),self.ROBOT,3,cv2.LINE_AA,tipLength=.45)
        return rgb
    def coverage(self,bev,robot):
        cov=bev.coverage();center=bev.cell(robot);half=round(self.meters/(2*bev.res));x0=center[0]-half;y0=center[1]-half
        out=np.full((2*half,2*half),-1,np.int8);ax0=max(0,x0);ay0=max(0,y0);ax1=min(cov.shape[1],x0+2*half);ay1=min(cov.shape[0],y0+2*half)
        if ax1>ax0 and ay1>ay0:out[ay0-y0:ay1-y0,ax0-x0:ax1-x0]=cov[ay0:ay1,ax0:ax1]
        out=cv2.resize(cv2.flip(out,0),(self.pixels,self.pixels),interpolation=cv2.INTER_NEAREST)
        image=np.zeros((self.pixels,self.pixels,3),np.uint8);image[out<0]=self.UNKNOWN;image[out==0]=(40,40,40);image[out==50]=(0,210,210);image[out==75]=self.TARGET;image[out==100]=self.OCCUPIED
        return image
