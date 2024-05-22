### Robomaster控制程序

***
python版本 3.8.18 <br>
robomaster sdk文档 <https://robomaster-dev.readthedocs.io/zh-cn/latest/python_sdk/modules.html>

1. 无人车/无人机配置
```bash
robomaster.config.LOCAL_IP_STR="local ip"
robomaster.config.ROBOT_IP_STR="rofrobomaster ip"
##无人车设置为tcp 无人机设置为udp
robomaster.config.DEFAULT_PROTO_TYPE="tcp/udp"
##conn_type默认为sta组网模式，sn为无人车序列号(无人机可不指定sn)
ep_robot.initialize(conn_type="sta", sn="xxxxxxxxxx")
```

2. mqtt配置
```bash
broker='mqtt ip'
port=1883
##无人机ID为1，无人车为4，5
device_id=''
##上传设备信息
report="device/{device_id}/report"
##任务派发
dispatch="device/{device_id}/task-dispatch"
##更新设备位置信息
location="device/{device_id}/location"
##更新任务状态/设备状态
update="device/{device_id}/task-update"
##打击位置坐标(仅无人车)
fires="device/{device_id}/fire"
##上报检测结果(仅无人机)
detected="device/{device_id}/detected"
```

3. 无人机/无人车原始流地址
```bash
##无人车1(device_id=4)
http://localIP:5000/video_feed 
##无人车2(device_id=5)
http://localIP:5001/video_feed
##无人机(device_id=1)
http://localIP:5002/video_feed
##推理服务在stream.txt中写入3个设备原始流地址
```

4. 启动服务
```bash
##无人车1(device_id=4)
python rm1.py
##无人车2(device_id=5)
python rm.py
##无人机(device_id=1)
python drone.py
```

### 推理服务
```bash
python fire.py

##推理流
##无人车1(device_id=4)
http://inferenceServiceIP:5000/video_feed/Camera_0
##无人车2(device_id=5)
http://inferenceServiceIP:5000/video_feed/Camera_1
##无人机1(device_id=1)
http://inferenceServiceIP:5000/video_feed/Camera_2
```
