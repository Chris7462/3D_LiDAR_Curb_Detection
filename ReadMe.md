# 3D Lidar curb detection
## 1. Moriyama dataset
Download the Moriyama rosbag from Autoware website: [Moriyama 150324](https://www.autoware.ai/sample/sample_moriyama_150324.tar.gz).
```bash
$ wget https://www.autoware.ai/sample/sample_moriyama_150324.tar.gz
$ tar -zxvf sample_moriyama_150324.tar.gz
```

## 2. Clone the repository
Clone the repo into your workspace
```bash
$ cd ~/catkin_ws/src/
$ git clone https://github.com/Chris7462/3D_LiDAR_Curb_Detection
```

## 3. Build
Build the code with catkin
```bash
$ cd ~/catkin_ws/
$ catkin build
```

## 4. Launch
Launch the curb detection launch file
```bash
$ roslaunch curb_detection curb_detection.launch
```
