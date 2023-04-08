from picamera import PiCamera
from time import sleep
import cv2
import object_detection as od


camera=PiCamera()
camera.resolution=(1920,1080)
camera.shutter_speed=100

camera.start_preview()
for i in range(5):
	sleep(1)
	camera.capture('/home/raspberrypi/Ingoos/image%s.jpg' %i)
	od.object_detection('Sample_TFLite_model','image%s.jpg' %i)
camera.stop_preview()