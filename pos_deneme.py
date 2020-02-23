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
import time
import cv2
import glob
import signal
import sys

############################# Bunları koda ekle #############################################################
import math
from math import cos
from math import sin
def f_cvt(x, y, angle):
	N = x*cos(angle) - y*sin(angle)
	E = x*sin(angle) + y*cos(angle)
	return N, E
#############################################################################################################

iha = ihamibagla()
armolveuc(iha, 1)

################ Burayı böyle değiştir ##################################### 
init_heading = (iha.heading / 180.0) * math.pi
a = init_heading
position(iha, f_cvt(1)[0], f_cvt(1)[1], 1, a)
############################################################################

	
