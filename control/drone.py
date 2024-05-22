# -*- coding: utf-8 -*-
import robomaster
from robomaster import robot
import cv2
from flask import Flask, Response
import random
import time
import json
from paho.mqtt import client as mqtt_client
from threading import Thread
import threading

app = Flask(__name__)

#tl_drone = robot.Drone()
robomaster.config.LOCAL_IP_STR = "192.168.6.136"
robomaster.config.ROBOT_IP_STR = "192.168.6.116"
# robomaster.config.ROBOT_IP_STR = "192.168.6.12"
# robomaster.config.ROBOT_IP_STR = "192.168.5.203"
robomaster.config.DEFAULT_PROTO_TYPE = "udp"
tl_drone = None
tl_flight = None
tl_camera = None
tl_running = None

broker = '192.168.6.187'
port = 1883
device_id =1 
topic = f"device/{device_id}/report"
dispatch = f"device/{device_id}/task-dispatch"
location=f"device/{device_id}/location"
update=f"device/{device_id}/task-update"
client_id = f'python-mqtt-{random.randint(0, 1000)}'
FRAME_WIDTH = 160
FRAME_HEIGHT = 120

def drone_init():
    global tl_drone
    global tl_flight
    global tl_camera
    tl_drone = robot.Drone()
    tl_drone.initialize("sta")
    tl_flight = tl_drone.flight
    tl_camera = tl_drone.camera
    tl_camera.set_down_vision(1)

    tl_camera.start_video_stream(display=False)
    tl_camera.set_fps("low")
    tl_camera.set_resolution("low")
    tl_camera.set_bitrate(6)

class Tello:
    def __init__(self):
        self.speed = {}
        self.pitch = 0
        self.roll = 0
        self.yaw = 0
        self.tof = 0
        self.height = 0
        self.batter = 0
        self.baro = 0
        self.time = 0

t1 = Tello()

class location:
    def __init__(self, x, y):
        self.x = x
        self.y = y

l1 = location(207,14)

def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(mqtt_client.CallbackAPIVersion.VERSION1, client_id)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client

def publish(client):
    location=f"device/{device_id}/location"
    msg = json.dumps(l1.__dict__)
    result = client.publish(location, msg)
    sub_drone()
    while True:
        time.sleep(1)
        try:
            msg = json.dumps({"battery": t1.batter, 
                              "speed": t1.speed, 
                              "baro": t1.baro, 
                              "height": t1.height,
                              "time": t1.time,
                              "pitch": t1.pitch,
                              "roll": t1.roll,
                              "yaw": t1.yaw,
                              "tof": t1.tof,
                              })
            result = client.publish(topic, msg)
            status = result[0]
            if status == 0:
                # print(msg)
                msg=msg
            else:
                print(f"Failed to send message to topic {topic}")
        except Exception as e:
            print(f"MQTT publish error: {e}")
        time.sleep(2)

def mqtt():
    client = connect_mqtt()
    client.loop_start()
    publish(client) 

should_stop = False

def subscribe(client: mqtt_client):
    global should_stop
    def on_message(client, userdata, msg):
        global should_stop
        msg = msg.payload.decode()
        data = json.loads(msg)
        print(data)
        route = data['route']
        id =data['id']
        print(id)
        deviceID = data['deviceID']
        print(deviceID)
        status = data["status"]
        if status =="aborted":
            should_stop = True 
        if deviceID == "1" and status!="aborted":
            location=f"device/{device_id}/location"
            update=f"device/{device_id}/task-update"
            run_thread = threading.Thread(target=run, args=(location, update, id))
            run_thread.start()
 
    client.subscribe(dispatch)
    client.on_message = on_message

def sub_battery(battery_info):
    t1.batter = battery_info

def sub_tof(tof_info):
    t1.tof = tof_info

def sub_drone_info(drone_info):
    high, baro, motor_time = drone_info
    t1.height = high
    t1.baro = baro
    t1.time = motor_time

def sub_atti(attitute_info):
    yaw, pitch, roll = attitute_info
    t1.yaw = yaw
    t1.pitch = pitch
    t1.roll = roll

def sub_imu(imu_info):
    vgx, vgy, vgz, agx, agy, agz = imu_info
    t1.speed = {
                "x_speed": vgx,
                "y_speed": vgy,
                "z_speed": vgz,
                "x_accelerated": agx,
                "y_accelerated": agy,
                "z_accelerated": agz,
            }  

def sub_drone():
    tl_drone.battery.sub_battery_info(freq=5, callback=sub_battery)
    tl_drone.sub_tof(freq=5, callback=sub_tof)
    tl_drone.sub_drone_info(freq=5, callback=sub_drone_info)
    tl_drone.flight.sub_imu(5, sub_imu)
    tl_drone.flight.sub_attitude(5, sub_atti)

def crop_frame(frame):
    height, width, _ = frame.shape
    # print(height) # 240 320   720 320
    # print(width)
    if height == 720 and width ==320:
        cropped_frame = frame[:height//2 - 120, :, :]
        return cropped_frame
    elif height ==240 and width ==320:
        return frame
    elif height ==720 and width ==960:
        cropped_frame = frame[:height//2 - 120, :, :]
        return cropped_frame
    # return frame 

def gen_frames():
    try:
        while True:
            frame = tl_camera.read_cv2_image()
            cropped_frame = crop_frame(frame)
            frames = cv2.resize(cropped_frame, (FRAME_WIDTH, FRAME_HEIGHT))
            ret, buffer = cv2.imencode('.jpg', frames)
            if not ret:
                break
            frames = buffer.tobytes()
            yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frames + b'\r\n')
            time.sleep(0.003)
    except:
        print('get frame exception ... ')

@app.route('/video_feed')
def video_feed():
    if not tl_running:
        return Response("")
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

def run_flask():
    app.run(host='0.0.0.0', port=5002)

def execute_command(command_func, *args, **kwargs):
    while True:
        result = command_func(*args, **kwargs).wait_for_completed(timeout=7)
        print(result)
        if result:
            break 
        else:
            tl_flight.land().wait_for_completed()
            print("try again")
            # tl_battery = tl_drone.battery
            # battery_info = tl_battery.get_battery()
            # print("Drone battery soc: {0}".format(battery_info))
            time.sleep(1) 

def checkCommand(result,id):
    if result:
        print("Flight successful")
    else:
        print("Flight failed")
        mqtt_topic_failure = f"device/{device_id}/task-update"
        msg = json.dumps({"id": id, "status": "failed"})
        client.publish(mqtt_topic_failure, msg)

def run(location, update, id):
    global should_stop
    global tl_running
    tl_running = True
    drone_init()
    msg = json.dumps(l1.__dict__)
    client.publish(location, msg)
    tl_flight.set_speed(speed=50)
    # execute_command(tl_flight.set_speed(speed=50))
    # execute_command(tl_flight.takeoff)
    result = tl_flight.takeoff().wait_for_completed(timeout=10)
    msg = json.dumps({"id": id, "status": "executing"})
    client.publish(update, msg)
    checkCommand(result,id)
    if should_stop:
        tl_flight.land().wait_for_completed(timeout=10)
        return

    result = tl_flight.forward(distance=235).wait_for_completed(timeout=10)
    checkCommand(result,id)
    if should_stop:
        tl_flight.land().wait_for_completed(timeout=10)
        return

    result = tl_flight.left(distance=180).wait_for_completed(timeout=10)
    checkCommand(result,id)
    if should_stop:
        tl_flight.land().wait_for_completed(timeout=10)
        return

    result = tl_flight.backward(distance=105).wait_for_completed(timeout=10)
    checkCommand(result,id)
    if should_stop:
        tl_flight.land().wait_for_completed(timeout=10)
        return

    result = tl_flight.right(distance=125).wait_for_completed(timeout=10)
    checkCommand(result,id)
    if should_stop:
        tl_flight.land().wait_for_completed(timeout=10)
        return

    result = tl_flight.forward(distance=75).wait_for_completed(timeout=10)
    checkCommand(result,id)
    if should_stop:
        tl_flight.land().wait_for_completed(timeout=10)
        return

    time.sleep(5)
    obj = json.dumps({"object": "tank"})
    detected = f"device/{device_id}/detected"
    client.publish(detected, obj)
    # execute_command(tl_flight.backward, distance=53)
    result = tl_flight.backward(distance=75).wait_for_completed(timeout=10)
    checkCommand(result,id)
    if should_stop:
        tl_flight.land().wait_for_completed(timeout=10)
        return

    result = tl_flight.right(distance=55).wait_for_completed(timeout=10)
    checkCommand(result,id)
    if should_stop:
        tl_flight.land().wait_for_completed(timeout=10)
        return

    result = tl_flight.backward(distance=130).wait_for_completed(timeout=10)
    checkCommand(result,id)
    if should_stop:
        tl_flight.land().wait_for_completed(timeout=10)
        return

    result = tl_flight.up(distance=50).wait_for_completed(timeout=10)
    checkCommand(result,id)
    if should_stop:
        tl_flight.land().wait_for_completed(timeout=10)
        return

    tl_flight.mission_pad_on()
    tl_flight.go(x=0, y=0, z=100, speed=20, mid="m1").wait_for_completed(timeout=10)
    threshold = 1
    x=tl_drone.get_status("x")
    y=tl_drone.get_status("y")
    while abs(x) > threshold or abs(y) > threshold:
        if abs(x) == 100 and abs(y) == 100:
            # execute_command(tl_flight.land)
            result = tl_flight.land().wait_for_completed(timeout=10)
            checkCommand(result,id)
            return
        else:       
            # execute_command(tl_flight.go, x=-x, y=-y, z=100, speed=10, mid="m1")
            result = tl_flight.go(x=-x, y=-y, z=70, speed=10, mid="m1").wait_for_completed(timeout=10)
            checkCommand(result,id)
            if not result:
                return
            x = tl_drone.get_status("x")
            y = tl_drone.get_status("y")
            print(f"x={x}, y={y}")

    execute_command(tl_flight.land)
    ljs = json.dumps({"id": id, "status": "finished"})
    client.publish(update, ljs)

    tl_flight.mission_pad_off()
    tl_running = False
    tl_camera.stop_video_stream()
    tl_drone.close()
    subscribe(client)


@app.route('/fly')
def execute_run():
    location=f"device/{device_id}/location"
    run(location,location,location)
    return "Run function executed"

if __name__ == '__main__':
    #robomaster.enable_logging_to_file()
    flask_thread = Thread(target=run_flask)
    flask_thread.start()
    # mqtt()
    client = connect_mqtt()
    subscribe(client)
    client.loop_start()
    publish_thread = threading.Thread(target=publish,args=(client,))
    publish_thread.start()