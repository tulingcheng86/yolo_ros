# YOLO-ROS

**YOLO-ROS for PyTorch**

**YOLO-ROS for TensorRT, please checkout for branch `tensorrt`**

**YOLO-ROS for HUAWEI ATLAS200, please checkout for branch `atlas200`**

Modified from [ros-yolov5](https://github.com/OuyangJunyuan/ros-yolov5), originated from [yolov5](https://github.com/ultralytics/yolov5)

1. clone `YOLO-ROS`
   ```
   cd {YOUR_WORKSPACE}/src
   git clone https://github.com/jianhengLiu/yolo_ros.git
   ```

2. install python dependencies
```
sudo apt install ros-melodic-ros-numpy
sudo apt install python3-pip -y 
pip3 install --upgrade pip

pip3 install torch torchvision --extra-index-url https://download.pytorch.org/whl/cu113

pip3 install -r yolo_ros/requirements.txt
//这一步安装到opencv-python会卡住 可以手动安装
pip cache purge
pip install opencv-python==4.5.3.56

catkin build
```
3. `catkin_make`

4. `roslaunch yolo_ros yolo_service.launch`

5. play your rosbag.





# 启动

`roslaunch yolo_ros yolo_service.launch`

然后播放自己的bag或者启动摄像头

话题是`/camera/color/image_raw`



**如果要显示检测框的话**

将[`service_client.cpp`](/src/service_client.cpp)的注释取消掉
