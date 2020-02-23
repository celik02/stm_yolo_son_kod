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


def signal_handler(sig, frame):
	print("[INFO] Exiting...")
	sys.exit()

def flight(pidx, pidy, satisfactory, phase, centered, logo, pos_dict, is_it_big, seen):
		signal.signal(signal.SIGINT, signal_handler)
		#iha = ihamibagla()
		armolveuc(iha, 6)
		saf=time.time()

		phase.value = 2
		logo.value = "ort"

		inis = 0
		while iha.mode == 'GUIDED':

				#if lost.value == 1:
				#	position(iha, 0, 0, 6, 0)

				if (seen.value == 1) and not centered.value:
					inis = 1

				if centered.value:
					time.sleep(0.1)
					print("Merkezleme basarili")
					"""iha.mode="LAND"
					while iha.armed==True:
						if not centered.value:
							#iha.mode = 'GUIDED'
							#time.sleep(0.3)
							#inis = 1
							break
					if not centered.value:
						continue"""

				elif inis:

					send_ned_velocity(iha, pidx.value, pidy.value, 0)
					if time.time()-saf>=1:
						print(inis, pidx.value,pidy.value)
						
						saf=time.time()

				if iha.armed == 0:
					time.sleep(0.5)
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
	centered_list = [False for i in range(5)]
	order_of_positions = [2, 4, 3, 1]
	#out = cv2.VideoWriter("appsrc ! videoconvert ! video/x-raw ! omxh265enc control-rate=2 bitrate=8000000  ! video/x-h265, stream-format=byte-stream ! rtph265pay mtu=1400 ! udpsink host=192.168.43.186 port=5000 sync=false async=false",cv2.CAP_GSTREAMER,0,30, (320, 240),True)
	camera = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)
	time.sleep(0.2)
	#time.sleep(2)

	while True:
		grabbed, frame = camera.read()
		rects, confidences, classIDs = detector.detect(frame)
		print(rects)
		frame, cond, (x1, x2, y1, y2), c = proper_detection(frame, rects, "ort",
							confidences, classIDs)
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
		#out.write(frame)
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
