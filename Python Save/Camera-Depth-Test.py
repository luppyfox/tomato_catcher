import cv2
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

Conf_threshold = 0.5
NMS_threshold = 0.5
COLORS = [(0, 255, 0)]

class_name = []
with open('C:/Users/ROBOT PC/Documents/Python Save/AI V3.4/obj.names', 'r') as f:
    class_name = [cname.strip() for cname in f.readlines()]
net = cv2.dnn.readNet('C:/Users/ROBOT PC/Documents/Python Save/AI V3.4/WEIGHTS.weights', 'C:/Users/ROBOT PC/Documents/Python Save/AI V3.4/CFG.cfg')

net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
model = cv2.dnn_DetectionModel(net)
model.setInputParams(size=(320, 320), scale=1/255, swapRB=True)

dc = DepthCamera()

while True :
    ret, Rawdepf, Rawcolf = dc.get_frame()
    if ret == False:
        break
    Fsdepf = Rawdepf[100:620, 132:1148]
    Fscolf = cv2.resize(Rawcolf, [1016,520])
    Rsdepf = Fsdepf[20:500, 188:828]
    Rscolf = Fscolf[20:500, 188:828]
    depf = cv2.resize(Rsdepf, [320,320])
    colf = cv2.resize(Rscolf, [320,320])
    # dis = depf[240, 300]
    # cv2.circle(colf, [320, 240], 4, (0,0,0))
    # print(dis)
    classes, scores, boxes = model.detect(colf, Conf_threshold, NMS_threshold)
    for (classid, score, box) in zip(classes, scores, boxes):
        color = COLORS[int(classid) % len(COLORS)]
        label = "%s : %f" % (class_name[classid], score)
        cv2.rectangle(colf, box, color, 2)
        cv2.putText(colf, label, (box[0], box[1]-20),cv2.FONT_HERSHEY_COMPLEX, 0.3, color, 1)
        x, y, w, h = box
        Xpos = int(x+(w/2))
        Ypos =  int(y+(h/2))
        cor = int((-0.1122*Xpos) - 2.1596)
        Xdpos = Xpos+cor
        cv2.circle(colf, [Xpos, Ypos], 4, (0,0,0))
        cv2.circle(colf, [Xdpos, Ypos], 4, (255,255,255))
        dis = depf[Ypos, Xdpos]
        print(dis, Xpos, Ypos)
        strdis = str(dis) + ' mm'
        cv2.putText(colf, strdis, (box[0], box[1]-10),cv2.FONT_HERSHEY_COMPLEX, 0.3, color, 1)
    cv2.imwrite('C:/Users/ROBOT PC/Documents/Python Save/Pic/colf.jpg', colf)
    cv2.imwrite('C:/Users/ROBOT PC/Documents/Python Save/Pic/depf.jpg', depf)
    cv2.imshow("colf", colf)
    cv2.imshow("depf", depf)
    key = cv2.waitKey(1)
    if key == ord('q'):
        break
dc.release()
cv2.destroyAllWindows()