import serial as sr
import random as rd
import cv2 
import math as m
import time
import numpy as np
import pyrealsense2 as rs

class DepthCamera:
    def __init__(self):
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        config = rs.config()

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))

        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)

        # Start streaming
        self.pipeline.start(config)

    def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        if not depth_frame or not color_frame:
            return False, None, None
        return True, depth_image, color_image

    def release(self):
        self.pipeline.stop()

ser1 = sr.Serial('COM6', 9600, timeout=5)
# ser2 = sr.Serial('COM7', 9600, timeout=5)

Conf_threshold = 0.5
NMS_threshold = 0.5
COLORS = [(0,255,0)]

# v3.2
class_name = []
with open('C:/Users/ROBOT PC/Documents/Python Save/AI V3.4/obj.names', 'r') as f:
    class_name = [cname.strip() for cname in f.readlines()]
net = cv2.dnn.readNet('C:/Users/ROBOT PC/Documents/Python Save/AI V3.4/WEIGHTS.weights', 'C:/Users/ROBOT PC/Documents/Python Save/AI V3.4/CFG.cfg')

net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
model = cv2.dnn_DetectionModel(net)
model.setInputParams(size=(320, 320), scale=1/255, swapRB=True)

i = 0
StartPoint = []
EndPoint = []

Endprocess = 0

dc = DepthCamera()

while True :
    Stopby = ser1.readline().decode("utf-8").strip()
    if Stopby == 'StandBy' :
        print(Stopby)
        break

while True:
    ret, Rawdepf, Rawcolf = dc.get_frame()
    if ret == False:
        break
    Fsdepf = Rawdepf[100:620, 132:1148]
    Fscolf = cv2.resize(Rawcolf, [1016,520])
    Depf = Fsdepf[20:500, 188:828]
    Colf = Fscolf[20:500, 188:828]
    # Depf = cv2.resize(Rsdepf, [320,320])
    # Colf = cv2.resize(Rscolf, [320,320])
    Def = 1
    ReCheck = 1
    endcommand = 0
    j = 0
    for j in range(i) :
        cv2.rectangle(Colf, StartPoint[j], EndPoint[j], (0, 255, 0), -1)
    time.sleep(1)
    classes, scores, boxes = model.detect(Colf, Conf_threshold, NMS_threshold)
    for (classid, score, box) in zip(classes, scores, boxes):
        Def = 0
        color = COLORS[int(classid) % len(COLORS)]
        label = "%s : %f" % (class_name[classid], score)
        cv2.rectangle(Colf, box, color, 2)
        cv2.putText(Colf, label, (box[0], box[1]-10),cv2.FONT_HERSHEY_COMPLEX, 0.3, color, 1)
        x, y, w, h = box
        Xpos = int(x+(w/2))
        Ypos = int(y+(h/2))
        cor = int((-0.1122*Xpos) - 2.1596)
        Xdpos = Xpos+cor
        Rawdis = Depf[Ypos, Xdpos]
        while Rawdis == 0 and ReCheck < 10 :
            Sd1 = rd.randint(-12,12)
            Sd2 = rd.randint(-12,12)
            Rawdis = Depf[Ypos+Sd1, Xdpos+Sd2]
            print("ReCheck distance...." + str(ReCheck))
            time.sleep(2)
            ReCheck += 1
        if Rawdis == 0 :
            print("Error detected in distance measurement")
            StartPoint.extend([(x, y)])
            EndPoint.extend([(x+w, y+h)])
            i += 1
            break
        DeltaCenter = m.sqrt(((Xpos-320)**2)+((Ypos-240)**2))
        dis = Rawdis + (0.08*DeltaCenter)
        print(str(dis))
        Xdis = (dis*m.cos((m.pi*(11/30))+((m.pi/2400)*(x+(w/2)))))-25     #+((320-Xpos)*0.062)
        Ydis = 160+(dis*m.cos((m.pi*(5/36))+((m.pi/2160)*(y+(h/2)))))
        Zdis = 570-(dis*m.sin((m.pi*(5/36))+((m.pi/2160)*(y+(h/2)))))
        # Adis = int(dis)
        XAdis = int(Xdis)
        YAdis = int(Ydis)
        ZAdis = int(Zdis)
        # # Put Text on screen
        # cv.putText(imgcap,str(Adis),(10,20),cv.FONT_HERSHEY_DUPLEX,0.5,(0,0,0),1)
        # cv.putText(imgcap,str(XAdis),(10,40),cv.FONT_HERSHEY_DUPLEX,0.5,(255,0,0),1)
        # cv.putText(imgcap,str(YAdis),(10,60),cv.FONT_HERSHEY_DUPLEX,0.5,(0,255,0),1)
        # cv.putText(imgcap,str(ZAdis),(10,80),cv.FONT_HERSHEY_DUPLEX,0.5,(0,0,255),1)
        if XAdis >= 0 :
            if XAdis >= 100 :
                charDataX = "+" + str(XAdis) 
            if XAdis < 100 and XAdis >= 10 :
                charDataX = "+0" + str(XAdis) 
            if XAdis < 10 :
                charDataX = "+00" + str(XAdis) 
        if XAdis < 0 :
            XAdis = -1*XAdis
            if XAdis >= 100 :
                charDataX = "-" + str(XAdis) 
            if XAdis < 100 and XAdis >= 10 :
                charDataX = "-0" + str(XAdis) 
            if XAdis < 10 :
                charDataX = "-00" + str(XAdis) 
        if YAdis >= 0 :
            if YAdis >= 100 :
                charDataY = "-" + str(YAdis) 
            if YAdis < 100 and YAdis >= 10 :
                charDataY = "-0" + str(YAdis) 
            if YAdis < 10 :
                charDataY = "-00" + str(YAdis) 
        if YAdis < 0 :
            YAdis = -1*YAdis
            if YAdis >= 100 :
                charDataY = "+" + str(YAdis) 
            if YAdis < 100 and YAdis >= 10 :
                charDataY = "+0" + str(YAdis) 
            if YAdis < 10 :
                charDataY = "+00" + str(YAdis)
        if ZAdis >= 0 :
            if ZAdis >= 100 :
                charDataZ = "+" + str(ZAdis)
            if ZAdis < 100 and ZAdis >= 10 :
                charDataZ = "+0" + str(ZAdis) 
            if ZAdis < 10 :
                charDataZ = "+00" + str(ZAdis) 
        if ZAdis < 0 :
            ZAdis = -1*ZAdis
            if ZAdis >= 100 :
                charDataZ = "-" + str(ZAdis) 
            if ZAdis < 100 and ZAdis >= 10 :
                charDataZ = "-0" + str(ZAdis)
            if ZAdis < 10 :
                charDataZ = "-00" + str(ZAdis)
        charCon = charDataX + charDataY + charDataZ + '\n'
        print(charCon)
        ser1.write(str.encode(charCon))
        while True :
            Stopby = ser1.readline().decode("utf-8").strip()
            if Stopby == 'StandBy' :
                print(Stopby)
                endcommand = 1
                break
            if Stopby == 'OutRange' :
                print(Stopby)
                StartPoint.extend([(x, y)])
                EndPoint.extend([(x+w, y+h)])
                i += 1
                endcommand = 1
                break
        if endcommand == 1 :
            break
    if Def == 1 :
        i = 0
        StartPoint.clear()
        EndPoint.clear()
        print('Move Forward')
        # CommandMove = '+030' + '\n'
        # ser2.write(str.encode(CommandMove))
        # while True :
        #     Stopby = ser2.readline().decode("utf-8").strip()
        #     if Stopby == 'StandBy' :
        #         print(Stopby)
        #         break
        # Endprocess += 1
        time.sleep(1)
    # if Endprocess == 6 :
    #     print('End Process')
    #     break
    # In case want to show the image
    cv2.imshow('Colf', Colf)
    key = cv2.waitKey(1)
    if key == ord('q'):
        break
dc.release()
cv2.destroyAllWindows()