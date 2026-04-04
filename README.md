# Jetson-Nano-STM32F407
采用Jetson Nano作为上位机，STM32F407开发板作为下位机，二者之间使用串口进行通信，上位机具备摄像头、雷达、IMU以及GPS传感器，下位机使用编码电机以及伺服电机进行控制。

<div align="center">

# 基于室外场景的智能车

</div>


## GPS节点
```
实现GPS数据的发布与订阅；
目前需要在室外进行验证；
```

## IMU节点
```
实现读取IMU传感器的数据，同时进行发布与订阅；
尚未开发客户端界面战术数据；
```

## 雷达节点
```
实现雷达使用;
目前启用翻转。实际激光雷安装位置可能需要进行优化；
```

## 摄像头节点
```
需要提高帧率较低的问题，能够实现检测，下一步需要针对实际情景训练模型；
ros2 launch vision_node vision.launch.py
ros2 run rqt_image_view rqt_image_view
```

## USART通信节点
```
下一步构建通信功能包，实现上位机与下位机之间的通信，通过上位机对下位机发布指令控制编码电机的转速，下位机实时返回编码电机转速，上位机推断当前速度；上位机也需要发布对转向舵机的控制，也需要发布对云台的控制（后期添加）；
/dev/ttyACM0
/dev/ttyTSH1
```

## C/C++配置
```
${workspaceFolder}/**
/opt/ros/${ROS_DISTRO}/include**
/opt/ros/humble/include/rosidl_runtime_cpp
/opt/ros/humble/include/rclcpp
/opt/ros/humble/include/std_msgs
```

---
