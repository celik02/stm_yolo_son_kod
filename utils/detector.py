import numpy as np
import time
import cv2
import os

class YOLO_Detector :

	def __init__(self, weights, cfg, confidence=0.05):
		self.Labels = ["stm", "odtu", "ort", "helikopter_inis", "turk_bayragi"]
		print("[INFO] loading YOLO from disk...")
		self.net = cv2.dnn.readNetFromDarknet(cfg, weights)
		self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
		self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
		ln = self.net.getLayerNames()
		self.ln = [ln[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]
		self.conf = confidence

	def detect(self, frame):
		(H, W) = frame.shape[:2]
		blob = cv2.dnn.blobFromImage(frame, 1 / 255.0, (416, 416),
			swapRB=True, crop=False)
		self.net.setInput(blob)
		start = time.time()
		layerOutputs = self.net.forward(self.ln)
		end = time.time()

		#print("[INFO] FPS {:.2f}".format(1/(end - start)))

		boxes = []
		confidences = []
		classIDs = []

		for output in layerOutputs:
			for detection in output:
				scores = detection[5:]
				classID = np.argmax(scores)
				confidence = scores[classID]

				if confidence > self.conf:
					box = detection[0:4] * np.array([W, H, W, H])
					(centerX, centerY, width, height) = box.astype("int")

					x1 = int(centerX - (width / 2))
					y1 = int(centerY - (height / 2))

					x2 = int(centerX + (width / 2))
					y2 = int(centerY + (height / 2))

					boxes.append([x1, y1, x2, y2])
					confidences.append(float(confidence))
					classIDs.append(classID)

		idxs = cv2.dnn.NMSBoxes(boxes, confidences, self.conf, 0.3)

		final_boxes = []
		final_confidences = []
		final_classIDs = []

		if len(idxs) > 0:
			for i in idxs.flatten():
				final_boxes.append(boxes[i])
				final_confidences.append(confidences[i])
				final_classIDs.append(classIDs[i])

		return final_boxes, final_confidences, final_classIDs
