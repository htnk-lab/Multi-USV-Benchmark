# Multi-USV-Benchmark
A benchmark on multi-USV control for aquatic environmental monitoring

## Requirements
- Ubuntu22.04
- ROS 2 Humble
- Python3.10

## Installation
TBD
```sh
mkdir ~/ros2ws/src
cd ~/ros2_ws/src
git clone https://github.com/htnk-lab/Multi-USV-Benchmark.git
cd ~/ros2_ws/src/Multi-USV-Benchmark
```
```sh
python3 -m pip install -r requirements.txt
```
```sh
sudo apt-get install xterm
```
```sh
sudo apt-get install ros-humble-xacro
```

```sh
git submodule add git@github.com:toshi67026/ros2bag2csv.git ros2bag2csv
```

## License
Apache License 2.0

## Usage

### 1. Build the workspace and source the environment

Run the following commands to build the packages and set up the environment:

```sh
cd ~/ros2_ws
colcon build
source ~/ros2_ws/install/local_setup.bash
```

### 2. Launch the program
Run the following command to start the program:
```sh
ros2 launch benchmark_main main.launch.py num:=2 name:=Data0
```
- ### Parameters
| Name  | Description               | Type        | Default | Min | Max | Note                |
|-------|---------------------------|-------------|---------|-----|-----|---------------------|
| `num` | Number of agents          | Natural int | 2       | 1   | 4   | Must be a natural number (≥1) |
| `name` | Name of the output rosbag          | String | Data       | -   | -   | Must not be empty |


## Maintainer
- [Toshiyuki Oshima](https://github.com/toshi67026)
