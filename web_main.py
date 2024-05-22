from yolov5 import Darknet
from camera import LoadStreams, LoadImages
from utils.general import non_max_suppression, scale_boxes, check_imshow
from flask import Response
from flask import Flask
from flask import render_template
from flask_cors import CORS
from time import sleep
import time
import torch
import json
import cv2
import os
import threading
from paho.mqtt import client as mqtt_client
import random
import base64

# initialize a flask object
app = Flask(__name__)
CORS(app)
broker = '192.168.1.37'
port = 1883
device_id =5
topic = f"device/{device_id}/fire"
client_id = f'python-mqtt-{random.randint(0, 1000)}'

# initialize the video stream and allow the camera sensor to warmup
with open('yolov5_config.json', 'r', encoding='utf8') as fp:
    opt = json.load(fp)
    print('[INFO] YOLOv5 Config:', opt)

darknet = Darknet(opt)
if darknet.webcam:
    #cudnn.benchmark = True  # set True to speed up constant image size inference
    dataset = LoadStreams(darknet.source, img_size=opt["imgsz"], stride=darknet.stride)
else:
    dataset = LoadImages(darknet.source, img_size=opt["imgsz"], stride=darknet.stride)
#time.sleep(2.0)

def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)
 
    client = mqtt_client.Client(mqtt_client.CallbackAPIVersion.VERSION1, client_id)
    client.on_connect = on_connect
    client.connect(broker, port,60)
    return client

@app.route("/")
def index():
    # return the rendered template
    return render_template("index.html")

target_flag = 1

def flag(topic,msg):
    global target_flag  
    target_flag += 1
    if target_flag == 2:
        findTarget(topic,msg)
index = 0
@torch.no_grad()
def detect_gen(dataset, feed_type ,device_id):
    view_img = check_imshow()
    detection_interval = 1 / 60
    t0 = time.time()
    global index
    for path, img, img0s, vid_cap in dataset:
        index+=1
        #print(index)
        if index == 2:
          index=0
          #print('drop')
          continue
        img = darknet.preprocess(img)
       # sleep(0.05)
        t1 = time.time()
        pred = darknet.model(img, augment=darknet.opt["augment"])[0]  # 0.22s
       # sleep(0.15)
        pred = pred.float()
        pred = non_max_suppression(pred, darknet.opt["conf_thres"], darknet.opt["iou_thres"])
        t2 = time.time()

        pred_boxes = []
        for i, det in enumerate(pred):
            if device_id ==1:
                img0s[0] = img0s[1]
            elif device_id ==2:
                img0s[0] = img0s[2]
            if i==1:
                break
            if darknet.webcam:  # batch_size >= 1
                feed_type_curr, p, s, im0, frame = "Camera_%s" % str(device_id), path[i], '%g: ' % i, img0s[i].copy(), dataset.count
            else:
                feed_type_curr, p, s, im0, frame = "Camera", path, '', img0s, getattr(dataset, 'frame', 0)

            s += '%gx%g ' % img.shape[2:]  # print string
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            if det is not None and len(det):
                det[:, :4] = scale_boxes(
                    img.shape[2:], det[:, :4], im0.shape).round()

                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    s += f"{n} {darknet.names[int(c)]}{'s' * (n > 1)}, "  # add to string

                for *xyxy, conf, cls_id in det:
                    kx = 90
                    ky = 65
                    lbl = darknet.names[int(cls_id)]
                    xyxy = torch.tensor(xyxy).view(1, 4).view(-1).tolist()
                    score = round(conf.tolist(), 3)
                   # label = "{}: {:.2f}".format(lbl, score)
                    label = "{}: {}".format(lbl, score)
                   # print(label)
                    x1, y1, x2, y2 = int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3])
                    x =((x1 + x2) / 2)
                    y =((y1 + y2) / 2)
                    tx =x / 320
                    ty =y / 240
                    target_pitch = kx *(tx-0.5)
                    target_yaw = ky *(0.5-ty)
                    if lbl == "potted plant":
                        if feed_type == "Camera_0":
                            topic = f"device/{4}/fire"
                            msg = json.dumps({"tp": target_pitch,
                             "ty":target_yaw, 
                             })
                            client.publish(topic, msg)
                        elif feed_type == "Camera_1":
                            topic = f"device/{5}/fire"
                            msg = json.dumps({"tp": target_pitch,
                             "ty":target_yaw,
                             })
                            client.publish(topic, msg)
                        elif feed_type == "Camera_2":
                            topic = f"device/{1}/detected"
                            msg = json.dumps({"object": "tank"})
                            darknet.plot_one_box(xyxy, im0, color=(0, 0, 255), label=label)
                            _, buffer = cv2.imencode('.jpg', im0)
                            image_bytes = base64.b64encode(buffer)
                            image_str = image_bytes.decode('utf-8')
                            msg = json.dumps({"object": "tank",
                                "image":image_str})
                            flag(topic, msg)    
                        #flag(topic,msg)
                    darknet.plot_one_box(xyxy, im0, color=(0, 0, 255), label=label)
                   # pred_boxes.append((x1, y1, x2, y2, lbl, score))
                    if view_img:
                        darknet.plot_one_box(xyxy, im0, color=(0, 0, 255), label=label)
            #print(f'{s}Done. ({t2 - t1:.3f}s)')
            if feed_type_curr == feed_type:
                frames = cv2.imencode('.jpg', im0)[1].tobytes()
                yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + frames + b'\r\n')

def findTarget(topic,msg): 
    #run()
     client.publish(topic, msg)

@app.route('/video_feed/<feed_type>')
def video_feed(feed_type):
    """Video streaming route. Put this in the src attribute of an img tag."""
    if feed_type == 'Camera_0':
        return Response(detect_gen(dataset=dataset, feed_type=feed_type , device_id=0),
                        mimetype='multipart/x-mixed-replace; boundary=frame')
    elif feed_type == 'Camera_1':
        return Response(detect_gen(dataset=dataset, feed_type=feed_type, device_id=1),
                        mimetype='multipart/x-mixed-replace; boundary=frame')
    elif feed_type == 'Camera_2':
        return Response(detect_gen(dataset=dataset, feed_type=feed_type, device_id=2),
                        mimetype='multipart/x-mixed-replace; boundary=frame')    
if __name__ == '__main__':
    client = connect_mqtt()
    client.loop_start()
    app.run(host='0.0.0.0', port="5001", threaded=True)
