# import the necessary packages
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
import os
import numpy as np

app = Flask(__name__)
os.environ['CUDA_LAUNCH_BLOCKING'] = '1' 
CORS(app)
broker = '192.168.6.187'
port = 1883
device_id =5
topic = f"device/{device_id}/fire"
client_id = f'python-mqtt-{random.randint(0, 1000)}'

with open('yolov5_config.json', 'r', encoding='utf8') as fp:
    opt = json.load(fp)
    print('[INFO] YOLOv5 Config:', opt)

darknet = Darknet(opt)
dataset = LoadStreams(darknet.source, img_size=opt["imgsz"], stride=darknet.stride)


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


detect_car1 = bytes()
detect_car2 = bytes()
detect_tello = bytes()
detect_value_mutex = threading.Lock()

# 循环获取最新帧
def loop_detect(feed_type=None):
    print('new request ..................................................', feed_type)
    while(True):
        #detect_value_mutex.acquire()
        sleep(0.1)
        if feed_type == 'Camera_0':
            yield detect_car1
        elif feed_type == 'Camera_1':
            yield detect_car2
        elif feed_type == 'Camera_2':
            yield detect_tello
        else:
            return 

@app.route('/video_feed/<feed_type>')
def video_feed(feed_type):
    print("camera ... ")
    """Video streaming route. Put this in the src attribute of an img tag."""
    if feed_type == 'Camera_0':
        if len(detect_car1) == 0:
            return  "", 500
        return Response(loop_detect(feed_type=feed_type),
                        mimetype='multipart/x-mixed-replace; boundary=frame')
    elif feed_type == 'Camera_1':
        if len(detect_car2) == 0:
            return  "", 500
        return Response(loop_detect(feed_type=feed_type),
                        mimetype='multipart/x-mixed-replace; boundary=frame')
    elif feed_type == 'Camera_2':
        return Response(loop_detect(feed_type=feed_type),
                        mimetype='multipart/x-mixed-replace; boundary=frame')   

# 返回原始视频数据
def ret_origin_data(img0s):
    for i in range(len(img0s)) :
        if(img0s[i] is not None):
            im0 = img0s[i].copy()
            frames = cv2.imencode('.jpg', im0)[1].tobytes()
            dete_bytes = (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + frames + b'\r\n')
            
        # 更新当前帧视频数据
            if i == 0:
                detect_car1 = dete_bytes
            elif i == 1:
                detect_car2 = dete_bytes
            elif i == 2:
                detect_tello = dete_bytes

@torch.no_grad()
def detect_gen():
    print('thread start ...')
    view_img = check_imshow()
    detection_interval = 1 / 60
    t0 = time.time()
    index = 0
    for path, img, img0s, vid_cap in dataset:
        if img0s == None:
            print('sleep 111')
            time.sleep(1)
            continue
        index+=1
        if index >= 2:
            index=0
            ret_origin_data(img0s)
            continue
        try:
            img = darknet.preprocess(img)
        except Exception as e:
            print('preprocess except ... ', e)
            ret_origin_data(img0s)
            time.sleep(1)
            torch.cuda.empty_cache()
            #TODO:处理下异常返回流
            # im0 = img0s[device_id].copy()
            # frames = cv2.imencode('.jpg', im0)[1].tobytes()
            # yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + frames + b'\r\n')
            continue

       # sleep(0.05)
        t1 = time.time()
        try:
            pred = darknet.model(img, augment=darknet.opt["augment"])[0]  # 0.22s
            pred = pred.float()
        except Exception as e:
            print("_______model except ___________", e)
            ret_origin_data(img0s)
            torch.cuda.empty_cache()
            # im0 = img0s[device_id].copy()
            # frames = cv2.imencode('.jpg', im0)[1].tobytes()
            # yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + frames + b'\r\n')
            continue

        pred = non_max_suppression(pred, darknet.opt["conf_thres"], darknet.opt["iou_thres"])
        t2 = time.time()

        pred_boxes = []
        for i, det in enumerate(pred):
            if img0s[i] is None:
                #print("img0s i is null",i)
                continue
            if np.all(img0s[i] == 0):
                print('all zero')
                continue
            if darknet.webcam:   # batch_size >= 1
                feed_type_curr, p, s, im0, frame = "Camera_%s" % str(i), path[i], '%g: ' % i, img0s[i].copy(), dataset.count
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
                    if lbl == "敌军":
                        if i == 0:  #小车1 
                            topic = f"device/{4}/fire"
                            msg = json.dumps({"tp": target_pitch,
                             "ty":target_yaw, 
                             })
                            client.publish(topic, msg)
                        elif i == 1:  #小车2
                            topic = f"device/{5}/fire"
                            msg = json.dumps({"tp": target_pitch,
                             "ty":target_yaw,
                             })
                            client.publish(topic, msg)
                        elif i == 2:  # 飞机
                            topic = f"device/{1}/detected"
                            msg = json.dumps({"object": "tank"})
                            im0 = darknet.plot_one_box(xyxy, im0, label=label)
                            _, buffer = cv2.imencode('.jpg', im0)
                            image_bytes = base64.b64encode(buffer)
                            image_str = image_bytes.decode('utf-8')
                            msg = json.dumps({"object": "tank",
                                "image":image_str})

                    im0 = darknet.plot_one_box(xyxy, im0, label=label)
                    #print(label)
                   # pred_boxes.append((x1, y1, x2, y2, lbl, score))
                    if view_img:
                        im0 = darknet.plot_one_box(xyxy, im0, color=(0, 0, 255), label=label)

            #print(f'{s}Done. ({t2 - t1:.3f}s)')
            im0 = cv2.resize(im0, (320,240))
            frames = cv2.imencode('.jpg', im0)[1].tobytes()
            dete_bytes = (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + frames + b'\r\n')
            
            # 更新当前帧视频数据
            global detect_car1
            global detect_car2
            detect_value_mutex.acquire()
            if i == 0:
                detect_car1 = dete_bytes
            elif i == 1:
                detect_car2 = dete_bytes
            elif i == 2:
                detect_tello = dete_bytes
            detect_value_mutex.release()

        torch.cuda.empty_cache()



if __name__ == '__main__':
    client = connect_mqtt()
    client.loop_start()

    # 启动模型推理线程
    thread = threading.Thread(target=detect_gen)
    thread.start()

    app.run(host='0.0.0.0', port="5000", threaded=True)
