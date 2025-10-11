# Multi-USV-Benchmark
A benchmark on multi-USV control for aquatic environmental monitoring

## Requirements
- Ubuntu22.04
- ROS 2 Humble
- Python3.10

## Installation
```sh
mkdir ~/ros2ws/src
cd ~/ros2_ws/src
git clone https://github.com/htnk-lab/Multi-USV-Benchmark.git
```
```sh
cd ~/ros2_ws/src/Multi-USV-Benchmark
python3 -m pip install -r requirements.txt
```
```sh
sudo apt-get install ros-humble-xacro
```
```sh
sudo apt-get install ros-humble-tf-transformations
```

```sh
git submodule update --init --recursive
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
| `num` | Number of agents          | Natural int | 2       | 1   | 4   | Must be a natural number (≥1).|
| `name` | Name of the output rosbag          | String | Data       | -   | -   | Must not be empty. Must not match the name of any existing file in the save path. |


To stop the simulation, press `Ctrl + C`.

### 3.  Extract data from the recorded rosbag
After the simulation is finished, you can extract the required　data from the generated bag file(.db3) and convert it into CSV format using `ros2bag2csv.py`. To use it, first navigate to the `ros2bag2csv` directory:

```sh
cd src/Multi-USV-Benchmark/ros2bag2csv
```
Instructions for how to run ros2bagcsv.py are provided in the README of the following repository:
https://github.com/toshi67026/ros2bag2csv/tree/757a36f1401ddc039c3a5b88b6476155cf678b85
## Maintainer
- [Toshiyuki Oshima](https://github.com/toshi67026)
