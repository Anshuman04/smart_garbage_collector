from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import time
import cv2
import io
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)


fl_en=40
fr_en=38
bl_en=37
br_en=35

fl_cl=31
fl_an=33
fr_cl=16
fr_an=18
bl_cl=13
bl_an=15
br_cl=11
br_an=12

TRIG=29
ECHO=32


GPIO.setup(fl_en,GPIO.OUT)
GPIO.setup(fr_en,GPIO.OUT)
GPIO.setup(bl_en,GPIO.OUT)
GPIO.setup(br_en,GPIO.OUT)
GPIO.setup(fl_cl,GPIO.OUT)
GPIO.setup(fl_an,GPIO.OUT)
GPIO.setup(fr_cl,GPIO.OUT)
GPIO.setup(fr_an,GPIO.OUT)
GPIO.setup(bl_cl,GPIO.OUT)
GPIO.setup(bl_an,GPIO.OUT)
GPIO.setup(br_cl,GPIO.OUT)
GPIO.setup(br_an,GPIO.OUT)
GPIO.setup(TRIG,GPIO.OUT)
GPIO.setup(ECHO,GPIO.IN)

// enable low at start

GPIO.output(fl_en,GPIO.LOW)
GPIO.output(fr_en,GPIO.LOW)
GPIO.output(bl_en,GPIO.LOW)
GPIO.output(br_en,GPIO.LOW)


fin=np.array([[0 for i in xrange(640)]for i in xrange(480)])
camera=PiCamera()
camera.resolution = (640,480)
rawCapture=PiRGBArray(camera)
time.sleep(0.1)
camera.capture(rawCapture, format="bgr")
image=rawCapture .array
img=cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


inp=input('ENTER A NO.')

rawCapture=PiRGBArray(camera)
time.sleep(0.1)
camera.capture(rawCapture, format="bgr")
image=rawCapture .array
img2=cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

fin=cv2.absdiff(img,img2)
(t, fin)=cv2.threshold(fin, 60, 255, cv2.THRESH_BINARY)
k=np.ones((5,5),np.uint8)
hihi1=cv2.morphologyEx(fin, cv2.MORPH_OPEN, k)
hihi=cv2.morphologyEx(hihi1, cv2.MORPH_OPEN, k)
a=0
b=0
f=0
d=0

for i in range (1,480):
    for j in range (1,640):
        if(hihi[i,j]==255):
            a=i;
            b=j;
            break


for k in xrange (479,0,-1):
    for l in xrange (639,0,-1):
        if(hihi[k,l]==255):
            f=k;
            d=l;
            break
        


midx=(a+f)/2;
midy=(b+d)/2;

rot=midy-320;
step=37.46/640;
move=step*rot;
if(move<0):
    
    GPIO.output(fl_cl,GPIO.LOW)
    GPIO.output(fl_an,GPIO.HIGH)
    GPIO.output(bl_cl,GPIO.LOW)
    GPIO.output(bl_an,GPIO.HIGH)
    GPIO.output(fr_cl,GPIO.LOW)
    GPIO.output(fr_an,GPIO.HIGH)
    GPIO.output(br_cl,GPIO.LOW)
    GPIO.output(br_an,GPIO.HIGH)

    move=move*(-1);
else:
    
    GPIO.output(fl_an,GPIO.LOW)
    GPIO.output(fl_cl,GPIO.HIGH)
    GPIO.output(bl_an,GPIO.LOW)
    GPIO.output(bl_cl,GPIO.HIGH)
    GPIO.output(fr_an,GPIO.LOW)
    GPIO.output(fr_cl,GPIO.HIGH)
    GPIO.output(br_an,GPIO.LOW)
    GPIO.output(br_cl,GPIO.HIGH)

cv2.imshow("img",hihi)

GPIO.output(fl_en,GPIO.HIGH)
GPIO.output(fr_en,GPIO.HIGH)
GPIO.output(bl_en,GPIO.HIGH)
GPIO.output(br_en,GPIO.HIGH)
delay=(move+4)/64
time.sleep(delay)
GPIO.output(fl_en,GPIO.LOW)
GPIO.output(fr_en,GPIO.LOW)
GPIO.output(bl_en,GPIO.LOW)
GPIO.output(br_en,GPIO.LOW)


        
/// ultra


GPIO.output(TRIG,0)
time.sleep(0.1)
GPIO.output(fl_cl,GPIO.LOW)
GPIO.output(fl_an,GPIO.HIGH)
GPIO.output(bl_cl,GPIO.LOW)
GPIO.output(bl_an,GPIO.HIGH)
GPIO.output(fr_an,GPIO.LOW)
GPIO.output(fr_cl,GPIO.HIGH)
GPIO.output(br_an,GPIO.LOW)
GPIO.output(br_cl,GPIO.HIGH)


GPIO.output(fl_en,GPIO.HIGH)
GPIO.output(fr_en,GPIO.HIGH)
GPIO.output(bl_en,GPIO.HIGH)
GPIO.output(br_en,GPIO.HIGH)

dist=100
while (dist>30):
    GPIO.output(TRIG,1)
    time.sleep(0.0001)
    GPIO.output(TRIG,0)
    while GPIO.input(ECHO)==0:
        pass
    start=time.time()
    while GPIO.input(ECHO)==1:
        pass
    stop=time.time()

    dist=(stop-start)*17000

// stop all_motors....enable low

GPIO.output(fl_en,GPIO.LOW)
GPIO.output(fr_en,GPIO.LOW)
GPIO.output(bl_en,GPIO.LOW)
GPIO.output(br_en,GPIO.LOW)
