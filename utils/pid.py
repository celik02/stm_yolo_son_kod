from utils import interpolation
from math import atan2
from math import sin,pi
from math import cos
import matplotlib.pyplot as plt

class PID :

	def __init__(self, coz_x=640, coz_y=480, mid_hiz=0):

		self.cozunurluk_x = coz_x   # Cozunurlugu tanimla
		self.cozunurluk_y = coz_y

		self.mid_hiz = mid_hiz		# Hiz baslangic hiz degerini tanimla
		self.ixerror=0
		self.iyerror=0
		self.pxerrorh = 0		    # Onceki hatayi tanimla
		self.pyerrorh = 0		    # Onceki hatayi taniml
		
		self.error_list = []
		self.j_list = [0]
		self.j = 0
	def update(self, sol_ustx, sol_usty, sag_altx, sag_alty, logo):

		# Dikdortgenlerin diger koselerini tanimla
		sol_altx = sol_ustx
		sol_alty = sag_alty
		sag_ustx = sag_altx
		sag_usty = sol_usty
		
		w = sag_altx - sol_ustx
		h = sag_alty - sol_usty
		#kph, kih, kdh = interpolation(w, h, logo)

		kphx = 0.0014
		kihx = 0.0001
		
		kdhx = 0
		
		kphy = 0.0013
		kihy = 0.0001
		
		kdhy = 0
		
		# Kilitlenme dikdortgeninin merkez koordinatini hesapla
		ort_x = sol_ustx
		ort_y = sag_altx
		
		# Hiz icin PID hesapla
		errorx = ort_x - (self.cozunurluk_x/2.0)
		errory = (self.cozunurluk_y/2.0) - ort_y
		#print("errorx:{} errory:{} merkez:({},{})".format(errorx,errory,self.cozunurluk_x/2.0,self.cozunurluk_y/2.0))
		
		if (errorx + self.ixerror)*kihx < 0.07 and (errorx + self.ixerror)*kihx > -0.07:
			self.ixerror = errorx + self.ixerror
		
		if (errory + self.iyerror)*kihy < 0.07 and (errory + self.iyerror)*kihy > -0.07:
			self.iyerror = errory + self.iyerror
		
		
		errorxdh = errorx - self.pxerrorh
		errorydh = errory - self.pyerrorh

		pidx = kphx*errorx + kihx*self.ixerror + kdhx*errorxdh
		pidy = kphy*errory + kihy*self.iyerror + kdhy*errorydh

		"""if errordh < 0 and pidh > 0 :
			pidh = pidh + kdh*errordh

		elif errordh > 0 and pidh < 0:
			pidh = pidh + kdh*errordh

		elif errordh > 0 and pidh > 0:
			pidh = pidh + kih*self.errorih

		elif errordh < 0 and pidh < 0:
			pidh = pidh + kih*self.errorih 

		#  Hiz degerlerini hesapla
		
		
		bolge_1=(ort_x>(self.cozunurluk_x/2.0))and(ort_y<(self.cozunurluk_y)/2.0)
		bolge_2=(ort_x<(self.cozunurluk_x/2.0))and(ort_y<(self.cozunurluk_y)/2.0)
		bolge_3=(ort_x<(self.cozunurluk_x/2.0))and(ort_y>(self.cozunurluk_y)/2.0)
		bolge_4=(ort_x>(self.cozunurluk_x/2.0))and(ort_y>(self.cozunurluk_y)/2.0)"""
		# Onceki error degerini guncelle
		self.pxerrorh = errorx
		self.pyerrorh = errory

                if pidx > 1:
                	pidx = 1
		elif pidx<-1:
			pidx= -1	
		if pidy > 1:
			pidy=1
		elif pidy<-1:
			pidy=-1
			


		"""rad = atan2(-(ort_y - (self.cozunurluk_y/2.0)),(ort_x-(self.cozunurluk_y/2.0)))
		if rad < 0:
			rad=rad+2*pi"""

		
		vy=pidx    					   # y yonundeki hizi
		vx=pidy 					   # x yonundeki hizi


		return (vx, vy)



# Birim testi
if __name__ == "__main__" :
	pid = PID(10, 20, 30)
	result = pid.update(50, 100, 70, 200)
	print("HizX:{}, HizY:{}, Aci:{}".format(result[0], result[1], result[2]))
