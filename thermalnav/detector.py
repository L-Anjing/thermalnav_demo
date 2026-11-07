"""Built-in OpenCV people detector; optional local COCO YOLO ONNX model."""
import os
import cv2
import numpy as np
class Detector:
    def __init__(self):
        self.path=os.getenv('THERMALNAV_DETECTOR_ONNX','')
        self.net=cv2.dnn.readNetFromONNX(self.path) if self.path else None
        self.hog=cv2.HOGDescriptor();self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        self.name='COCO YOLO ONNX' if self.net else 'OpenCV HOG person (limited recall)'
    def detect(self,bgr):
        if self.net is None:
            boxes,weights=self.hog.detectMultiScale(bgr,winStride=(8,8),padding=(8,8),scale=1.05)
            return [(tuple(map(float,(x,y,x+w,y+h))),float(s),'person') for (x,y,w,h),s in zip(boxes,weights) if float(s)>.5]
        h,w=bgr.shape[:2];scale=min(640/w,640/h);nw,nh=int(w*scale),int(h*scale)
        image=np.full((640,640,3),114,np.uint8);image[:nh,:nw]=cv2.resize(bgr,(nw,nh))
        self.net.setInput(cv2.dnn.blobFromImage(image,1/255.,(640,640),swapRB=True))
        out=self.net.forward().squeeze()
        if out.shape[0]<out.shape[1]:out=out.T
        boxes=[];scores=[]
        for row in out:
            # YOLOv8/11 COCO export: 4 xywh + 80 class scores.
            if len(row)!=84:raise ValueError('Expected YOLO COCO ONNX output (84,N)')
            if np.argmax(row[4:])!=0 or row[4]<.4:continue
            x,y,bw,bh=row[:4]/scale;boxes.append([float(x-bw/2),float(y-bh/2),float(bw),float(bh)]);scores.append(float(row[4]))
        ids=cv2.dnn.NMSBoxes(boxes,scores,.4,.5)
        return [((boxes[i][0],boxes[i][1],boxes[i][0]+boxes[i][2],boxes[i][1]+boxes[i][3]),scores[i],'person') for i in np.asarray(ids).reshape(-1)]
