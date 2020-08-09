#!/usr/bin/env python3
import numpy as np
import time
import cv2
import os

labelsPath = os.path.sep.join(["/home/guru/catkin_ws/src/grid_working/src/Perception/yolov4", "coco.names"])
LABELS = open(labelsPath).read().strip().split("\n")
np.random.seed(42)
COLORS = np.random.randint(0, 255, size=(len(LABELS), 3),
	dtype="uint8")
weightsPath = os.path.sep.join(["/home/guru/catkin_ws/src/grid_working/src/Perception/yolov4", "yolov4.weights"])
configPath = os.path.sep.join(["/home/guru/catkin_ws/src/grid_working/src/Perception/yolov4", "yolov4.cfg"])
print("[INFO] loading YOLO from disk...")
net = cv2.dnn.readNetFromDarknet(configPath, weightsPath)
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
def detect(image):
    (H, W) = image.shape[:2]
    ln = net.getLayerNames()
    ln = [ln[i[0] - 1] for i in net.getUnconnectedOutLayers()]
    blob = cv2.dnn.blobFromImage(image, 1 / 255.0, (416, 416),
        swapRB=True, crop=False)
    net.setInput(blob)
    start = time.time()
    layerOutputs = net.forward(ln)
    end = time.time()
    
    print("[INFO] YOLO took {:.6f} seconds".format(end - start))
    
    boxes = []
    confidences = []
    classIDs = []
    
    for output in layerOutputs:
        for detection in output:
            scores = detection[5:]
            classID = np.argmax(scores)
            confidence = scores[classID]
            if confidence > 0.9 :
                box = detection[0:4] * np.array([W, H, W, H])
                (centerX, centerY, width, height) = box.astype("int")
                x = int(centerX)
                y = int(centerY)
                boxes.append([x, y, int(width), int(height)])
                confidences.append(float(confidence))
                classIDs.append(classID)
    idxs = cv2.dnn.NMSBoxes(boxes, confidences, 0.9, 0.3)
    
    if len(idxs) > 0:
        box1 = boxes[0]
    else:
	    box1 = []
    return(box1)
