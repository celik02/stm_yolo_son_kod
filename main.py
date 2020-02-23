from multiprocessing import Manager
from multiprocessing import Process
from ctypes import c_char_p
from utils import gstreamer_pipeline
from utils import send_ned_velocity
from utils import proper_detection
from utils import YOLO_Detector
from utils import is_centered
from utils import ihamibagla
from utils import armolveuc
from utils import position
from utils import quarter
from utils import decide
from utils import PID
from utils import big_enough
import imutils
import time
import cv2
import glob
import signal
import sys


import math
from math import cos
from math import sin
def f_cvt(x, y, angle):
	N = x*cos(angle) - y*sin(angle)
	E = x*sin(angle) + y*cos(angle)
	return N, E


def signal_handler(sig, frame):
	print("[INFO] Exiting...")
	sys.exit()

def flight(pidx, pidy, satisfactory, phase, centered, logo, pos_dict, is_it_big, seen):
	signal.signal(signal.SIGINT, signal_handler)
	iha = ihamibagla()
	armolveuc(iha, 5)
	saf=time.time()

	init_heading = (iha.heading / 180.0) * math.pi
	a = init_heading
	ulp = (f_cvt(4.2,-4.2,a)[0], f_cvt(4.2,-4.2,a)[1], 5,init_heading)
	urp = (f_cvt(4.2,4.2,a)[0],f_cvt(4.2,4.2,a)[1], 5,init_heading)
	dlp = (f_cvt(-4.2,-4.2,a)[0], f_cvt(-4.2,-4.2,a)[1], 5,init_heading)
	drp = (f_cvt(-4.2,4.2,a)[0], f_cvt(-4.2,4.2,a)[1], 5,init_heading)
	cp = (0,0,5,init_heading)

	phase.value = 1
	mapping_points = [(3,3,9), (-3,3,9), (-3,-3,9)]

	for (i, (x, y, z)) in enumerate(mapping_points):
		print(iha.heading)
		position(iha, x, y, z, init_heading)
		prev_time = time.time()
		time.sleep(1)
		satisfactory.value = 0
		while(satisfactory.value != 2):
			if int(time.time() - prev_time) % 10 == 0:
				pass
				#print("Mapping checkpoint {}".format(i))
	phase.value = 2
	corresponding_pos = {1 : ulp, 2 : urp, 3 : dlp, 4 : drp, 5 : cp}

	for i in ["stm", "odtu", "ort", "helikopter_inis", "turk_bayragi"]:
		i = i.encode("utf-8")
		iha.mode = 'GUIDED'
		time.sleep(0.3)
		yaklasim = 1
		inis = 0
		yaklas=0
		logo.value = i
		if iha.mode == 'GUIDED':
			pos = corresponding_pos.get(pos_dict.get(i))
			while True:
				#print("yaklasim:",yaklasim, "yaklas:", yaklas, "inis:",inis, "center:",centered.value, "seen:",seen.value)
				if (seen.value == 1) and not centered.value:
					inis = 1
					yaklasim = 0
					yaklas = 0

				if yaklasim:
					armolveuc(iha, 5)
					position(iha, *pos)
					time.sleep(2)

					if ((iha.location.local_frame.north<=-4.2*0.92 and
						iha.location.local_frame.east>=4.2*0.92) or
						(iha.location.local_frame.north>=4.2*0.92 and
						iha.location.local_frame.east>=4.2*0.92) or
						(iha.location.local_frame.north<=-4.2*0.92 and
						iha.location.local_frame.east<=-4.2*0.92) or
						(iha.location.local_frame.north>=4.2*0.92 and
						iha.location.local_frame.east<=-4.2*0.92) or
						(iha.location.local_frame.north<=-8) or
						(iha.location.local_frame.north>=8) or 
						(iha.location.local_frame.east<=-8) or 
						(iha.location.local_frame.east>=8)) :
						yaklasim = 0
						inis = 1
				elif yaklas:
					position(iha, *pos)
					time.sleep(5)
					yaklas = 0
					inis = 1

				elif centered.value:
					time.sleep(0.1)
					iha.mode="LAND"
					while iha.armed==True:
						print("LANDING")
						if not centered.value:
							#iha.mode = 'GUIDED'
							#time.sleep(0.3)
							#inis = 1
							break
					if not centered.value:
						continue

				elif inis:

					send_ned_velocity(iha, pidx.value, pidy.value, 0)
					if time.time()-saf>=1:
						print(inis, pidx.value,pidy.value)
						
						saf=time.time()
				#print("lost={} ,enough={} ",format(lost.value,is_it_big.value))
				if lost.value==1 and not is_it_big.value:
					inis=0
					yaklas=1
					


				if iha.armed == 0:
					time.sleep(0.5)
					break


			if i == "turk_bayragi" and iha.location.global_relative_frame.alt<=0.1:
				iha.armed = False 
		else :
			break



def cv(up_left_x, up_left_y, bottom_right_x, bottom_right_y,
		logo, centered, phase, pos_dict, satisfactory, lost, is_it_big, seen):

	signal.signal(signal.SIGINT, signal_handler)
	weights = "Tiny-YOLO/logo_final.weights"
	cfg = "Tiny-YOLO/logo.cfg"
	detector = YOLO_Detector(weights, cfg)
	counter_for_mapping = 1
	local_logo = 0
	logo_list = ["stm", "odtu", "ort", "helikopter_inis"]
	pos_dict["turk_bayragi"] = 5
	lost_list = [False for i in range(4)]
	centered_list = [False for i in range(10)]
	order_of_positions = [2, 4, 3, 1]
	camera = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)
	time.sleep(0.2)
	#time.sleep(2)
	while True:
		time.sleep(1)
		if phase.value == 1:
			while True:
				grabbed, frame1 = camera.read()
				rects, confidences, classIDs = detector.detect(frame1)
				print("phase", phase.value)
				print("satisfactory", satisfactory.value)
				print("logo_list", logo_list)
				if satisfactory.value == 0:
					#time.sleep(1)
					#time.sleep(0.13)
					frame = quarter(frame1, counter_for_mapping)
					satisfactory.value, label= decide(confidences, classIDs)
					#cv2.imshow("he", frame1)
					#cv2.waitKey(1)
					#cv2.imshow("hey", frame)
					#cv2.waitKey(1)
					if satisfactory.value == 1:
						if not (label in logo_list):
							satisfactory.value = 0
							continue
						del logo_list[logo_list.index(label)]
						pos_dict[label] = order_of_positions[counter_for_mapping - 1]
						counter_for_mapping = counter_for_mapping + 1
						satisfactory.value = satisfactory.value + 1
						print(logo_list)
						print(len(logo_list))
						if len(logo_list) == 1:
							print(logo_list[0])
							pos_dict[logo_list[0]] = 1
					print(pos_dict)

				if phase.value == 2:
						break

				print(pos_dict)


		if phase.value == 2:
			cv2.destroyAllWindows()
			time.sleep(1)
			while True:
				grabbed, frame = camera.read()
				rects, confidences, classIDs = detector.detect(frame)
				frame, cond, (x1, x2, y1, y2), c = proper_detection(frame, rects, logo.value,
									confidences, classIDs)
				print(rects)
				if cond:
					seen.value = 1
					centered_list[:-1] = centered_list[1:]
					centered_list[-1] = c
					lost_list[:-1] = lost_list[1:]
					lost_list[-1] = True
					up_left_x.value = x1
					up_left_y.value = y1
					bottom_right_x.value = x2
					bottom_right_y.value = y2
					is_it_big.value = big_enough(x2-x1, y2-y1)

				else:
					seen.value = 0
					centered_list[:-1] = centered_list[1:]
					centered_list[-1] = False
					lost_list[:-1] = lost_list[1:]
					lost_list[-1] = False

				lost.value = 0 if any(lost_list) else 1
				centered.value = is_centered(centered_list)

				#cv2.imshow("test", frame)
				#cv2.waitKey(1)


def pid(x1, y1, x2, y2, pidx, pidy, lost, logo):

	signal.signal(signal.SIGINT, signal_handler)
	local_logo = 0
	_pid = PID()
	while True:
		if (local_logo != logo.value) or lost.value == 1:

			_pid = PID()
			local_logo = logo.value
			pidx.value = 0
			pidy.value = 0
		pidx.value, pidy.value = _pid.update(x1.value, y1.value, x2.value, y2.value, local_logo)
		time.sleep(0.015)

if __name__ == "__main__":

	with Manager() as manager:
		# PID <-----> CV
		up_left_x = manager.Value("i", 0)
		up_left_y = manager.Value("i", 0)
		bottom_right_x = manager.Value("i", 0)
		bottom_right_y = manager.Value("i", 0)
		lost = manager.Value("i", 1)

		# AUTOPILOT <-----< PID
		pidx = manager.Value("f", 0.0)
		pidy = manager.Value("f", 0.0)

		# CV <-------> AUTOPILOT
		logo = manager.Value(c_char_p, "stm")
		centered = manager.Value("i", 0)
		phase = manager.Value("i", 1)
		satisfactory = manager.Value("i", 1)
		pos_dict = manager.dict()
		is_it_big = manager.Value("i", 0)
		seen = manager.Value("i", 0)

		# we have 3 independent processes
		# 1. AUTOPILOT  - controls the drone
		# 2. CV         - finds/localizes the object
		# 3. PID        - PID control loop determines x, y velocity

		processPID = Process(target=pid,
			args=(up_left_x, up_left_y, bottom_right_x, bottom_right_y,
				pidx, pidy, lost, logo))

		processCV = Process(target=cv,
			args=(up_left_x, up_left_y, bottom_right_x, bottom_right_y,
				logo, centered, phase, pos_dict, satisfactory, lost, is_it_big, seen))

		processAUTOPILOT = Process(target=flight,
			args=(pidx, pidy, satisfactory, phase, centered, logo, pos_dict, is_it_big, seen))

		# start all 3 processes
		processAUTOPILOT.start()
		processCV.start()
		processPID.start()

		# join all 3 processes
		processAUTOPILOT.join()
		processCV.join()
		processPID.join()
