import cv2
from flask import Flask, Response
from threading import Thread
import robomaster
from robomaster import robot
import time
from paho.mqtt import client as mqtt_client
from robomaster import blaster
import random
import json
import threading

app = Flask(__name__)

robomaster.config.LOCAL_IP_STR = "192.168.6.136"
robomaster.config.ROBOT_IP_STR = "192.168.6.146"
robomaster.config.DEFAULT_PROTO_TYPE ="tcp"

ep_robot = None
ep_chassis = None
ep_gimbal = None
ep_blaster = None
ep_vision = None
ep_battery = None
ep_sensor = None
ep_camera = None
ep_running = False  # 小车是否运行中，调了运行接口为启动中

broker = '192.168.6.187'
port = 1883
device_id =4 
topic = f"device/{4}/report"
dispatch = f"device/{device_id}/task-dispatch"
update=f"device/{device_id}/task-update"
location=f"device/{device_id}/location"
fires = f"device/{device_id}/fire"
client_id = f'python-mqtt-{random.randint(0, 1000)}'
FRAME_WIDTH = 320
FRAME_HEIGHT = 240

def robomaster_init():
    global ep_robot
    global ep_chassis
    global ep_gimbal
    global ep_blaster
    global ep_vision
    global ep_battery
    global ep_sensor 
    global ep_camera
    # robomaster.enable_logging_to_file()
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="sta", sn="3JKCK6U0030AVL")
    ep_chassis = ep_robot.chassis
    ep_gimbal = ep_robot.gimbal
    ep_blaster = ep_robot.blaster
    ep_vision = ep_robot.vision
    ep_battery = ep_robot.battery
    ep_sensor = ep_robot.sensor
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False)

    ep_robot.set_robot_mode(mode=robot.CHASSIS_LEAD)
    ep_gimbal.recenter().wait_for_completed()
    ep_gimbal.move(pitch=-30, yaw=0).wait_for_completed
    ep_robot.set_robot_mode(mode=robot.GIMBAL_LEAD)

class robomaster:
    def __init__(self, batter, status, speed, position, attitude, distance_sensor,chassis_attitude,rpm ):
        self.batter = batter  
        self.status = status                                      
        self.speed = speed
        self.position = position
        self.attitude  = attitude 
        self.distance_sensor = distance_sensor
        self.chassis_attitude = chassis_attitude
        self.rpm = rpm

rm1 = robomaster("", "", "", "","","","","")

class location:
    def __init__(self, x, y):
        self.x = x
        self.y = y

l1 = location(19,264)


def sub_battery(batter_info, ep_robot):
    rm1.batter = batter_info
 
def sub_gimbal(angle_info):
    pitch_angle, yaw_angle, pitch_ground_angle, yaw_ground_angle = angle_info
    rm1.attitude = {
        "x_attitude":yaw_angle,
        "y_attitude":pitch_angle
    }

def sub_position(position_info):
    x, y, z = position_info 
    rm1.position={
        # "x_distance":x,
        # "y_distance":y,
        "angle":z
    }

def sub_sensor(sub_info):
    rm1.distance_sensor = sub_info[0]

def sub_chassis_attitude(attitude_info):
    yaw, pitch, roll = attitude_info
    rm1.chassis_attitude = {
        "yaw":yaw,
        "pitch":pitch,
        "roll":roll
    }

def sub_speed(speed_info):
    vgx, vgy, vgz, vbx, vby, vbz = speed_info
    rm1.speed
    rm1.speed={
        "x_speed":vbx,
        "y_speed":vby,
        "z_speed":vbz,
    }

def sub_rpm(rpm_info):
    speed, angle, timestamp, state = rpm_info
    rm1.rpm={
        "r1_rev":speed[0],
        "l1_rev":speed[1],
        "r2_rev":speed[2],
        "l2_rev":speed[3],
    }

def sub_status(status_info):
    static_flag, up_hill, down_hill, on_slope, pick_up, slip_flag, impact_x, impact_y, impact_z, roll_over, hill_static = status_info
    if static_flag ==1:
        rm1.status = "static_flag"
    elif up_hill ==1:
        rm1.status = "static_flag"
    elif down_hill ==1:
        rm1.status = "down_hill"
    elif on_slope ==1:
        rm1.status = "on_slope"
    elif pick_up ==1:
        rm1.status = "pick_up"
    elif slip_flag ==1:
        rm1.status = "slip_flag"
    elif impact_x ==1:
        rm1.status = "impact_x"
    elif impact_y ==1:
        rm1.status = "impact_y"
    elif impact_z ==1:
        rm1.status = "impact_z"
    elif roll_over ==1:
        rm1.status = "roll_over" 
    elif hill_static ==1:
        rm1.status = "hill_static"                                      

def sub_robomaster():
    ep_battery.sub_battery_info(5, sub_battery, ep_robot)
    ep_chassis.sub_status(freq=5, callback=sub_status)
    ep_chassis.sub_position(freq=5, callback=sub_position)
    ep_sensor.sub_distance(freq=5, callback=sub_sensor)
    ep_gimbal.sub_angle(freq=5, callback=sub_gimbal)
    ep_chassis.sub_attitude(freq=5, callback=sub_chassis_attitude)
    ep_chassis.sub_velocity(freq=5, callback=sub_speed)
    ep_chassis.sub_esc(freq=5, callback=sub_rpm)

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

def publish(client):
    location=f"device/{device_id}/location"
    msg = json.dumps(l1.__dict__)
    result = client.publish(location, msg)
    sub_robomaster()
    while True:
        time.sleep(5)
        msg = json.dumps({"battery": rm1.batter,
                          "status":rm1.status, 
                          "speed": rm1.speed, 
                          "position":rm1.position, 
                          "attitude":rm1.attitude,
                          "distance_sensor":rm1.distance_sensor,
                          "chassis_attitude":rm1.chassis_attitude,
                          "rpm":rm1.rpm
                          })
        result = client.publish(topic, msg)
        status = result[0]
        if status == 0:
            # print(msg)   
            msg = msg
        else:
            print(f"Failed to send message to topic {topic}")

def task_update(client: mqtt_client,message):
    result = client.publish(update, message)
    status = result[0]
    if status == 0:
        print(message)
    else:
        print(f"Failed to send message to topic {update}")      

should_stop = False

def subscribe(client: mqtt_client):
    global should_stop
    def on_message(client, userdata, msg):
        global should_stop
        msg = msg.payload.decode()
        data = json.loads(msg)
        print("onmessage data: ",data)

        # deviceID = data['deviceID']
        id=''
        deviceID='4'
        try:
            id =data['id']
        except:
            return
        print(id)
        status = data["status"]
        if status =="aborted":
            should_stop = True 
        if deviceID == '4' and status!="aborted":
            should_stop = False 
            location=f"device/{device_id}/location"
            update=f"device/{device_id}/task-update"
            # run(location,update,id)
            run_thread = threading.Thread(target=run, args=(location, update, id))
            # run_thread = threading.Thread(target=fire, args=())
            run_thread.start()
            # run_thread.join()
 
    client.subscribe(dispatch)
    client.on_message = on_message

last_move_time = 0

def subFire(client: mqtt_client):
    global last_move_time
    start_time = time.time()
    def on_message(client, userdata, msg):
        global last_move_time
        if time.time() - start_time >= 4:
            # client.unsubscribe(fires)
            subscribe(client)
            return
        msg = msg.payload.decode()
        data = json.loads(msg)
        tp = data['tp']
        ty = data['ty']
        current_time = time.time()
        if current_time - last_move_time >= 1:
            ep_gimbal.move(pitch=ty, yaw=tp, pitch_speed=540, yaw_speed=540)
            # ep_blaster.fire(times=1)
            ep_blaster.fire(fire_type=blaster.INFRARED_FIRE)
            last_move_time = current_time

    client.subscribe(fires)
    client.on_message = on_message
    # subscribe(client)

def mqtt():
    client = connect_mqtt()
    subscribe(client)
    client.loop_start()
    publish(client) 

def on_detect_line_1(line_info):
    line_type = line_info[0]
    if line_type >=1:
        line = line_info[2]
        x = line[0]
        # print(x)
        output = 200 * (x-0.5)
        print(output)
        if x>0.6 :
            return
        if output>60 or output<-50:
            return
        ep_chassis.drive_speed(x=0.2, y=0.0, z=output, timeout=None)
    else:  
        ep_gimbal.move(pitch=-30, yaw=0).wait_for_completed()
        print(output)
        print("none line")
        ep_chassis.drive_speed(x=0, y=0.0, z=0.0, timeout=None)

def stop(id):
    ep_vision.unsub_detect_info(name="line")
    ep_chassis.drive_speed(x=0.0, y=0.0, z=0.0, timeout=None)
    ep_robot.set_robot_mode(mode=robot.GIMBAL_LEAD)
    print("stop success")

def fire(id):
    ep_robot.set_robot_mode(mode=robot.FREE)
    result = ep_gimbal.moveto(pitch=0, yaw=0,pitch_speed=540, yaw_speed=540).wait_for_completed(timeout=8)
    checkCommand(result,id)
    result = ep_gimbal.move(pitch=-5, yaw=90,pitch_speed=10, yaw_speed=100).wait_for_completed(timeout=8)
    checkCommand(result,id)
    time.sleep(3)
    subFire(client)
    time.sleep(5)        
    # ep_gimbal.recenter().wait_for_completed(timeout=8)
    result = ep_gimbal.moveto(pitch=0, yaw=0,pitch_speed=540, yaw_speed=540).wait_for_completed(timeout=8)
    checkCommand(result,id)
    ep_robot.set_robot_mode(mode=robot.CHASSIS_LEAD)
    result = ep_gimbal.move(pitch=-30, yaw=0,pitch_speed=540, yaw_speed=0).wait_for_completed(timeout=8)
    checkCommand(result,id)
    print("fire success")

def generate_frames():
    try:
        while True:
            frame = ep_camera.read_cv2_image()
            frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))
            ret, buffer = cv2.imencode('.jpg', frame)
            if not ret:
                break
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            time.sleep(0.001)
    except:
        return
        
def checkCommand(result,id):
    if result:
        print("Executed successful")
    else:
        print("Executed failed")
        mqtt_topic_failure = f"device/{device_id}/task-update"
        msg = json.dumps({"id": id, "status": "failed"})
        client.publish(mqtt_topic_failure, msg)

def homing(id):
    ep_robot.set_robot_mode(mode=robot.CHASSIS_LEAD)
    # ep_chassis.move(x=85, y=0, z=0, xy_speed=0.4).wait_for_completed() 

    #ep_chassis.drive_speed(x=0.2, y=0.0, z=0, timeout=None)
    #time.sleep(3.15)

    ep_chassis.move(x=0.7, y=0, z=0, xy_speed=0.6).wait_for_completed()
    result = ep_chassis.move(x=0, y=0, z=180, z_speed=45).wait_for_completed(timeout=8)
    checkCommand(result,id)
    print("is home")

def turnRight(id):
    ep_robot.set_robot_mode(mode=robot.CHASSIS_LEAD)
    result = ep_chassis.move(x=0, y=0, z=175, z_speed=45).wait_for_completed(timeout=8)
    checkCommand(result,id)
    result = ep_gimbal.move(pitch=-30, yaw=0).wait_for_completed(timeout=8)
    checkCommand(result,id)

def straight(id):
    result = ep_chassis.move(x=0.55, y=0, z=0, xy_speed=0.2).wait_for_completed(timeout=8)
    checkCommand(result,id)   

def run(location,update,id):
    #每次启动
    robomaster_init()
    global ep_running
    ep_running = True
    js = json.dumps({"id": id,
                          "status":"executing", 
                          })
    client.publish(update,js)

    time.sleep(0.5)

    global should_stop
    ep_robot.set_robot_mode(mode=robot.CHASSIS_LEAD)
    msg = json.dumps(l1.__dict__)
    client.publish(location, msg)

    #js = json.dumps({"id": id,
     #                     "status":"executing", 
      #                    })
    #client.publish(update,js)


    #ep_chassis.drive_speed(x=0.2, y=0.0, z=0, timeout=None)
    #time.sleep(2)
    ep_chassis.move(x=0.4, y=0.02, z=0, xy_speed=0.6).wait_for_completed()

    ep_vision.sub_detect_info(name="line", color="blue", callback=on_detect_line_1)
    time.sleep(3.5)
    stop(id)
    stop(id)
    # print(should_stop)
    if should_stop:
        return
    time.sleep(1.5)
    fire(id)
    print(should_stop)
    if should_stop:
        return

    turnRight(id)
    print(should_stop)
    if should_stop:
        return
    ep_vision.sub_detect_info(name="line", color="blue", callback=on_detect_line_1)
    time.sleep(2)
    print(should_stop)
    if should_stop:
        return
    print('homing... ')
    stop(id)
    stop(id)
    # straight()
    # turnRight()
    homing(id)
    # time.sleep(4)
    ljs = json.dumps({"id": id,
                          "status":"finished", 
                          })
    client.publish(update, ljs)
    ep_robot.set_robot_mode(mode=robot.CHASSIS_LEAD)
    result = ep_gimbal.moveto(pitch=0, yaw=0,pitch_speed=540, yaw_speed=540).wait_for_completed(timeout=8)
    checkCommand(result,id)
    result = ep_gimbal.move(pitch=-30, yaw=0).wait_for_completed(timeout=8)
    checkCommand(result,id)
    ep_robot.set_robot_mode(mode=robot.GIMBAL_LEAD)
    
    ep_running = False
    ep_robot.close()
    subscribe(client)
    
@app.route('/video_feed')
def video_feed():
    if not ep_running:
        return Response('')
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/line')
def execute_run():
    location=f"device/{device_id}/location"
    update=f"device/{device_id}/task-update"
    run(location,update)
    return "Run function executed"

def run_flask():
    app.run(host='0.0.0.0', port=5000)

if __name__ == '__main__':
    # robomaster.enable_logging_to_file()
    flask_thread = Thread(target=run_flask)
    flask_thread.start()
    # mqtt()
    client = connect_mqtt()
    subscribe(client)
    client.loop_start()
    publish_thread = threading.Thread(target=publish,args=(client,))
    publish_thread.start()
