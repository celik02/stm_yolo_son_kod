from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import numpy as np
import argparse
import imutils
import pickle
import time
import math
import cv2

# iha objesini olusturur. 
def ihamibagla():


	iha = connect("/dev/serial/by-id/usb-ArduPilot_fmuv3_330037000E51353239333634-if00",wait_ready=True,timeout=100)

	return iha

def big_enough(width,heigth):
	total_screen=480*640
	logo_screen=width*heigth
	rate=float(logo_screen)/(total_screen)
	return 1 if rate>=0.75 else 0

#Iha arm olur ve hedef irtifaya ulasir.
def armolveuc(iha, hedefirtifa):
	while iha.is_armable==False:
		print("Arm icin gerekli sartlar saglanamadi. Lutfen bekleyin")
		time.sleep(1.5)
	print("Ihamiz su anda arm edilebilir")
	
	iha.mode=VehicleMode("GUIDED")
	while iha.mode!='GUIDED':
		print("GUIDED moduna gecis icin bekleniyor")
		time.sleep(1.5)
	print("GUIDED moduna gecis yapildi")
	iha.armed=True
	while iha.armed==False:
		pass
		#print("Arm icin bekleniliyor")
	print("Ihamiz arm olmustur kolay gelsin")

	iha.simple_takeoff(hedefirtifa)
	
	while iha.location.global_relative_frame.alt<=0.90*hedefirtifa:
		print("Su anki yukseklik: %d"%iha.location.global_relative_frame.alt)
		time.sleep(0.5)
	print("Hedef yukseklige ulasildi")

#Hiz komutlarini paketler ve Mavlink protokolu ile ardupilota gonderir.
def send_ned_velocity(iha, velocity_x,velocity_y, velocity_z):

	msg = iha.message_factory.set_position_target_local_ned_encode(
          0,
          0, 0,
          mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, 
          0b0000011111000111,
          0, 0, 0, 
          velocity_x, velocity_y, -velocity_z, #m/s cinsindendir
          0, 0, 0,
          0, 0)

	iha.send_mavlink(msg)

#Pozisyon komutlarini paketler ve Mavlink protokolu ile ardupilota gonderir.
def position(iha, pos_x, pos_y, pos_z, heading):

	msg = iha.message_factory.set_position_target_local_ned_encode(
          0,
          0, 0,
          mavutil.mavlink.MAV_FRAME_LOCAL_NED, 
          0b100111111000,
          pos_x, pos_y, -pos_z, #Pozisyon komutlari (metre cinsindendir)
          0, 0, 0,
          0, 0, 0,
          heading, 0)

	iha.send_mavlink(msg)

def quarter(img, v):
	_dict = {1 : [0, 240, 320, 640],
		2 : [240, 480, 320, 640],
		3 : [240, 480, 0, 320],
		4 : [0, 240, 0, 320]}
	bndrs = _dict.get(v)
	return img[bndrs[0] : bndrs[1], bndrs[2] : bndrs[3]]

def decide(confidences, classIDs):
	class_index_dict = {0:"stm", 1:"odtu", 2:"ort", 3:"helikopter_inis", 4:"turk_bayragi"}

	if len(confidences) == 0:
		return 0, None

	else :
		confidences = np.array(confidences)
		index = np.argmax(confidences)
		confidence = confidences[index] 
		label = classIDs[index]
		label = class_index_dict.get(label)
		if label == "stm" and confidence > 0.1:
			return 1, "stm"
		elif label == "odtu" and confidence > 0.25:
			return 1, "odtu"
		elif label == "ort" and confidence > 0.25:
			return 1, "ort"
		elif label == "helikopter_inis" and confidence > 0.32:
			print("hel_in", confidence)
			return 1, "helikopter_inis"
		elif label == "turk_bayragi" and confidence > 0.25:
			return 1, "turk_bayragi"
		else :
			return 0, None

def gstreamer_pipeline(capture_width=640, capture_height=480,
		display_width=640, display_height=480, framerate=8, flip_method=3) : 
  
	return ('nvarguscamerasrc ! ' 
	'video/x-raw(memory:NVMM), '
	'width=(int)%d, height=(int)%d, '
	'format=(string)NV12, framerate=(fraction)%d/1 ! '
	'nvvidconv flip-method=%d ! '
	'video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! '
	'videoconvert ! '
	'video/x-raw, format=(string)BGR ! appsink'  % (capture_width,capture_height,framerate,flip_method,display_width,display_height))

def interpolation(w, h, logo):
	kp = 0
	ki = 0
	kd = 0
	"""interpolate = pickle.loads(open("interpolate.cpickle", "rb").read())
	print("interpolation : {}".format(logo))
	if logo == "turk_bayragi":
		kp, ki, kd = interpolate.get("turk_bayragi")(w*h)
	else :
		kp, ki, kd = interpolate.get("others")(w*h)"""
		
	return kp, ki, kd

def proper_detection(frame, rects, logo, confidences, classIDs):
	class_index_dict = {"stm":0, "odtu":1, "ort":2, "helikopter_inis":3, "turk_bayragi":4}
	logo = logo.decode("utf-8")
	cond = 0
	confidences = np.array(confidences)
	for i in np.argsort(confidences)[::-1]:
		if classIDs[i] == class_index_dict.get(logo):
			x1 = rects[i][0]
			y1 = rects[i][1]
			x2 = rects[i][2]
			y2 = rects[i][3]
			cond = 1
			break

	if cond == False:
		return frame, cond, (None, None, None, None), None	

	mid_x = int((x1 + x2) / 2)
	mid_y = int((y1 + y2) / 2)

	half_w = int((x2 - x1) * 0.25)
	half_h = int((y2 - y1) * 0.25)

	r_x1 = mid_x - half_w
	r_x2 = mid_x + half_w
	r_y1 = mid_y - half_h
	r_y2 = mid_y + half_h

	h, w = frame.shape[:2]
	centered = False

	if ( ( (h/2) >= r_y1 ) and ( (h/2) <= r_y2 ) and
		( ( (w/2) >= r_x1 ) and  ( (w/2) <= r_x2 ) ) ):
		centered = True
	cv2.line(frame, (320, 240), (mid_x, mid_y), (0,0,255), 2)
	cv2.circle(frame, (mid_x, mid_y), 5, (0,0,255), -1)
	cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
	if centered :
		cv2.rectangle(frame, (r_x1, r_y1), (r_x2, r_y2), (0, 0, 255), 2)

	return frame, cond, (mid_x, mid_y, y1, y2), centered




def is_centered(c):
	false_number = len([i for i in c if i == False])
	r = 1 if false_number <= 3 else 0
	return r

