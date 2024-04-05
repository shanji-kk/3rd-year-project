import cv2
import ultralytics
from ultralytics import YOLO
import torch
import argparse
import numpy as np
import time
import serial 
from pathlib import Path

print("Ultralytics version:", ultralytics.__version__)
print(cv2.__version__)

## Setup of flags and parameters
prev_frame_time = 0
grab=False
mode = None
place=False
system_motion=True
device = torch.device('cpu')

# Initialise the serial port
arduino = serial.Serial(port='COM6', baudrate=115200, timeout=.05) 
def write_read(x): 
    arduino.write(bytes(x, 'utf-8')) 
    time.sleep(0.02) 
    data = arduino.readline() 
    return data 

# Get the opencv resource and yolo v8 model
cap = cv2.VideoCapture(1)
model = YOLO("C:/Users/dongg/Desktop/YOLOV8N/runs/detect/train8/weights/best.pt")

ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", type=str,
	help="path to input video file")
ap.add_argument("-t", "--tracker", type=str, default="kcf",
	help="OpenCV object tracker type")
args = vars(ap.parse_args())

# extract the OpenCV version info
(major, minor) = cv2.__version__.split(".")[:2]
# if we are using OpenCV 3.2 OR BEFORE, we can use a special factory
# function to create our object tracker
if int(major) == 3 and int(minor) < 3:
	tracker = cv2.Tracker_create(args["tracker"].upper())
# otherwise, for OpenCV 3.3 OR NEWER, we need to explicity call the
# approrpiate object tracker constructor:
else:  
	# initialize a dictionary that maps strings to their corresponding
	OPENCV_OBJECT_TRACKERS = {
		"csrt": cv2.TrackerCSRT_create,
		"kcf": cv2.TrackerKCF_create,
		"boosting": cv2.TrackerBoosting_create,
		"mil": cv2.TrackerMIL_create,
		"tld": cv2.TrackerTLD_create,
		"medianflow": cv2.TrackerMedianFlow_create,
		"mosse": cv2.TrackerMOSSE_create
	}
	# grab the appropriate object tracker using our dictionary of
	tracker = OPENCV_OBJECT_TRACKERS[args["tracker"]]()



while system_motion == True:
    # Check the mission category
    targetFlag = False
    majdetect = False
    initBB = None
    operation = write_read('')  # read data from the serial port
    print(operation)
    if '100' in operation.decode('utf-8', errors='ignore'):
        grab = True
    elif '200' in operation.decode('utf-8', errors='ignore'):
        place = True
    
    

        
    # Code for grabing       
    while grab == True:
        print('grab yes')
        # Get the frame and fps from resource
        ret, frame = cap.read()
        new_frame_time = time.time()
        fps = 1 / (new_frame_time - prev_frame_time)
        prev_frame_time = new_frame_time
        ## When the system has not catch the grame yet
        if initBB is None:
            # Delay for camera to become stable
            time.sleep(5.0)
            ## While loop that find the 
            while majdetect==False:
                # recall the results from prediction with hyperparameters
                results = model.predict(frame, conf=0.5)
                majnum = -1
                target = -1
                # record the number of each class in a vector and find the class with largest number store in majnum
                for result in results:
                    boxes = result.boxes
                    classnp = np.zeros(len(model.names))
                    for box in range(len(boxes)):
                        detected_class = int(boxes.cls[box])
                        classnp[detected_class] += 1
                    for maj in range(len(classnp)):
                        if (classnp[maj] > majnum) & (classnp[maj]!=0):
                            majnum = maj

                # When there is no object detected end the loop 
                if majnum == -1:
                    grab=False
                    print ("No item detected")
                else:
                    majdetect = True
                    print('object detected')
                    
            while (targetFlag == False) and (majdetect == True):
                for box in range(len(boxes)):
                    print(box)
                    print(int(len(boxes)))
                    if (int(boxes.cls[box]) != majnum):
                        target = int(boxes.cls[box])
                        xywh_tensor = boxes.xywh.clone().detach()
                        (x, y, w, h) = xywh_tensor[box].tolist()
                        initBB = (int(x-w/2), int(y-h/2), w, h)
                        # initBB = (int(x), int(y), w, h)
                        p3 = (int(x),int(y))
                        p4 = (int(x+w), int(y+h))
                        print(initBB)
                        print(x,y,w,h)
                        targetFlag = True
                        tracker.init(frame, initBB)
                    if box == int(len(boxes)-1) and (targetFlag==False):
                        grab=False
                        targetFlag = True
                        print('correct shelf')
            print(target)
        if initBB is not None:
            (success, bbox) = tracker.update(frame)
            if success:
                p1 = (int(bbox[0]), int(bbox[1]))
                p2 = (int(bbox[0] +bbox[2]), int(bbox[1]+ bbox[3]))
                print(p1,p2)   
                # cv2.rectangle(frame, p3, p4, (0, 255, 0), 2)  
                cv2.rectangle(frame, p1, p2, (0, 255, 0), 2) 
                centroid_x = int((bbox[2]+2*bbox[0]) / 2)
                centroid_y  = int((2*bbox[1]+bbox[3]) / 2)
                
                if centroid_x > 325:
                    num =str(-1)
                    value = write_read(num)
                elif centroid_x <315:
                    num = str(1)
                    value = write_read(num)
                else:
                    num = str(-10)
                    value = write_read(num)
                    grab=False
            cv2.putText(frame, f"FPS: {int(fps)}", (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(frame, f"targetclass: {model.names[target]}", (30, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
            cv2.putText(frame, f". target", (centroid_x, centroid_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 1, cv2.LINE_AA)
            cv2.imshow("Frame", frame)
            ## assist
            # width  = cap.get(cv2.CAP_PROP_FRAME_WIDTH)   # float `width`
            # height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)  # float `height`\
            # print('width',width)
            # print('height',height)
            print (centroid_x)
            if cv2.waitKey(1) == 27:  # 按下 'q' 键退出循环
                system_motion=False
                break
        else:
            print("Correct shelf")
            grab=False
            #10 represent the message go to walk but send message to let place = True
            num = str(10)
            value = write_read(num)
    ## Second mode for place the current item onto shelf        
    while place == True:
        time.sleep(5.0)
        ret, frame = cap.read()
        results=model.predict(frame,conf=0.6,show=True)
        majnum = -1
        for result in results:
            boxes = result.boxes
            classnp = np.zeros(len(model.names))
            for box in range(len(boxes)):
                detected_class = int(boxes.cls[box])
                classnp[detected_class] += 1
            for maj in range(len(classnp)):
                if (classnp[maj] > majnum) & (classnp[maj]!=0):
                    majnum = maj
        
        ## sned message whether the holding item belong to current shelf
        if (target == majnum):
            place=False
            #20 means send message to let system place the item
            value = write_read(str(20))
            print(20)
        else:
            place=False
            value = write_read(str(10))
            print(10)
        if cv2.waitKey(1) == 27:
            system_motion=False
            break

    cv2.destroyAllWindows()







# # opencv find central of mass
                # centroidimg = frame[int(y+1):int(y+h-1),int(x+1):int(x+w-1)]
                # gray = cv2.cvtColor(centroidimg,cv2.COLOR_BGR2GRAY)
                # blur = cv2.GaussianBlur(gray, (3,3), 0) 
                # thresh = cv2.threshold(blur, 120, 255, cv2.THRESH_BINARY)[1]
                # sobelx = cv2.Sobel(src=blur, ddepth=cv2.CV_64F, dx=1, dy=0, ksize=5) # Sobel Edge Detection on the X axis
                # sobely = cv2.Sobel(src=blur, ddepth=cv2.CV_64F, dx=0, dy=1, ksize=5) # Sobel Edge Detection on the Y axis
                # sobelxy = cv2.Sobel(src=blur, ddepth=cv2.CV_64F, dx=1, dy=1, ksize=5) # Combined X and Y Sobel Edge Detection
                # edges = cv2.Canny(image=blur, threshold1=100, threshold2=200)
                # # 定义一个椭圆形的内核
                # kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

                # # 对边缘图像应用闭运算以填充内部空洞
                # closed_edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
                # nonzero_points = np.nonzero(closed_edges)
                # nonzero_points_x = nonzero_points[1]
                # nonzero_points_y = nonzero_points[0]

                # # 计算非零像素点的数量
                # num_nonzero_points = len(nonzero_points_x)

                # # 计算非零像素点的质心
                # centroid_x = int(np.sum(nonzero_points_x) / num_nonzero_points)
                # centroid_y = int(np.sum(nonzero_points_y) / num_nonzero_points)
                # cv2.imshow("centroid", thresh)  