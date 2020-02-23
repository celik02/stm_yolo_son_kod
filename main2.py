from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ctypes import c_char_p
from multiprocessing import Manager
from multiprocessing import Process
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
import dlib
import rospy
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
	iha = ihamibagla()
	armolveuc(iha, 9)
	saf=time.time()
	init_heading=0
	ulp = (4.2,-4.2,5,init_heading)
	urp = (4.2,4.2,5,init_heading)
	dlp = (-4.2,-4.2,5,init_heading)
	drp = (-4.2,4.2,5,init_heading)
	cp = (0,0,5,init_heading)

	phase.value = 1
	mapping_points = [(2,2,9), (-2,2,9), (-2,-2,9)]

	for (i, (x, y, z)) in enumerate(mapping_points):
		print(iha.heading)
		position(iha, x, y, z, init_heading)
		prev_time = time.time()
		time.sleep(1.5)
		satisfactory.value = 0
		while(satisfactory.value != 1):
			if int(time.time() - prev_time) % 10 == 0:
				pass
				#print("Mapping checkpoint {}".format(i))
	print("buladayim")
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
					iha.mode="LAND"
					while iha.armed==True:
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
				#if lost.value==1 and not is_it_big.value:
				#	inis=0
				#	yaklas=1
					


				if iha.armed == 0:
					time.sleep(0.5)
					break


			if i == "turk_bayragi" and iha.location.global_relative_frame.alt<=0.1:
				iha.armed = False 
		else :
			break



def cv(up_left_x, up_left_y, bottom_right_x, bottom_right_y,
		logo, centered, phase, pos_dict, satisfactory, lost, image_list, f, is_it_big, seen):

	signal.signal(signal.SIGINT, signal_handler)
	weights = "Tiny-YOLO/logo_final.weights"
	cfg = "Tiny-YOLO/logo.cfg"
	is_permitted = True
	curr_time = 0
	detector = YOLO_Detector(weights, cfg)
	counter_for_mapping = 1
	local_logo = 0
	logo_list = ["stm", "odtu", "ort", "helikopter_inis"]
	pos_dict["turk_bayragi"] = 5
	lost_list = [False for i in range(4)]
	centered_list = [False for i in range(5)]
	order_of_positions = [2, 4, 3, 1]
	time.sleep(2)
	while True:
		time.sleep(1)
		print("phase", phase.value)
		if phase.value == 1:
			print("satisfactory", satisfactory.value)
			while satisfactory.value == 0:
				time.sleep(1)
				print("c", satisfactory.value)
				time.sleep(0.13)
				frame1 = f[-1]
				frame = quarter(frame1, counter_for_mapping)
				rects, confidences, classIDs = detector.detect(frame)
				satisfactory.value, label= decide(confidences, classIDs)
				cv2.imshow("he", frame1)
				cv2.waitKey(1)
				cv2.imshow("hey", frame)
				cv2.waitKey(1)
				if satisfactory.value:
					del logo_list[logo_list.index(label)]
					pos_dict[label] = order_of_positions[counter_for_mapping - 1]
					counter_for_mapping = counter_for_mapping + 1
					print(logo_list)
					print(len(logo_list))
					if len(logo_list) == 1:
						print(logo_list[0])
						pos_dict[logo_list[0]] = 1
				else:
					break

		print(pos_dict)


		if phase.value == 2:
			cv2.destroyAllWindows()
			time.sleep(1)
			while True:
				#grabbed, frame = camera.read()
				#print("frame", frame.shape[:2])
				frame = f[-1]
				rects, confidences, classIDs = detector.detect(frame)
				frame, cond, (x1, x2, y1, y2), c = proper_detection(frame, rects, logo.value,
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

				if (is_centered(centered_list)):
					if is_permitted == True:
						curr_time = time.time()
						is_permitted = False						
					if (time.time() - curr_time > 0.2):
						centered.value = is_centered(centered_list)

				if not (is_centered(centered_list)):
					is_permitted = True
					centered.value = 0				

				cv2.imshow("test", frame)
				cv2.waitKey(1)


def pid(x1, y1, x2, y2, pidx, pidy, lost, logo, image_list, phase, satisfactory, f):

	signal.signal(signal.SIGINT, signal_handler)

	"""while True:
		#frame = cv2.imread("../frame.jpg")
		if satisfactory.value == 0:
			print(f)
			image_list.append(f[0])
		if phase.value == 2:
			break

	camera.release()"""
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


def img_converter(f):
	signal.signal(signal.SIGINT, signal_handler)
	global i
	i = 0
	def callback(data):
		global i
		try:
			cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
		
		f.append(cv_image)
		if i > 3:
			del f[0]
		i = i + 1
		#print(f)
		cv2.imshow("zxc", cv_image)
		cv2.waitKey(1)

		try:
			image_pub.publish(bridge.cv2_to_imgmsg(cv_image, "bgr8"))
		except CvBridgeError as e:
			print(e)
	image_pub = rospy.Publisher("image_topic_2",Image)
	bridge = CvBridge()
	image_sub = rospy.Subscriber("/drone/camera1/image_raw",Image,callback)

	rospy.init_node('image_converter', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

		
				

if __name__ == "__main__":

	with Manager() as manager:
		# PID <-----> CV
		up_left_x = manager.Value("i", 0)
		up_left_y = manager.Value("i", 0)
		bottom_right_x = manager.Value("i", 0)
		bottom_right_y = manager.Value("i", 0)
		lost = manager.Value("i", 1)
		image_list = manager.list()
		f = manager.list()

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
				pidx, pidy, lost, logo, image_list, phase, satisfactory, f))

		processCV = Process(target=cv,
			args=(up_left_x, up_left_y, bottom_right_x, bottom_right_y,
				logo, centered, phase, pos_dict, satisfactory, lost, image_list, f, is_it_big, seen))

		processAUTOPILOT = Process(target=flight,
			args=(pidx, pidy, satisfactory, phase, centered, logo, pos_dict, is_it_big, seen))

		processIMG = Process(target=img_converter,
			args=(f, ))

		# start all 3 processes
		processAUTOPILOT.start()
		processCV.start()
		processPID.start()
		processIMG.start()

		# join all 3 processes
		processAUTOPILOT.join()
		processCV.join()
		processPID.join()
		processIMG.join()
