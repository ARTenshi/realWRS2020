# WRS Challenge

Please, follow the indications [here](https://github.com/hsr-project/tmc_wrs_docker) to setup the correct working environment.

## 1. Get the repository

In a new terminal in your virtual IDE at [http://localhost:3001](http://localhost:3001) first, create a workspace:

```
cd /workspace
mkdir -p catkin_ws/src
```

Then, clone this repository into the src folder:

```
cd /workspace/catkin_ws/src
git clone https://github.com/ARTenshi/wrs2020.git
```

Then, install the necessary libraties and packages by entering:

```
cd /workspace/catkin_ws/src/wrs2020
sudo ./install.sh
```

Additionally, in case you haven't installed OpenCV, please enter

```
cd /workspace/catkin_ws/src/wrs2020
sudo ./install-opencv.sh
```

Finally, compile your project:

```
cd /workspace/catkin_ws
catkin_make
```

### 1.1 Configuration

In a text editor, add the following lines to your ~/.bashrc file

> rosparam set /hsrb/omni_base_controller/base_coordinates
"[odom_x,odom_y,odom_r]"

> rosparam set /hsrb/impedance_control/joint_names "[]"

> source /workspace/carkin_ws/devel/setup.bash


Finally, kill the default localisation node

```
rosnode kill /iris_lama_loc2d
```

## 2. Usage

In a terminal, enter

```
roslaunch navigation_tamagoya navigation_module.launch
```

Then, in a second terminal, enter

```
roslaunch wrs_challenge erasersvision.launch
```

Finally, in a different terminal enter

```
roslaunch wrs_challenge tidyup.launch
```

# Authors

Please, if you use this material, don't forget to add the following reference:

```
@misc{contreras2020,
    title={WRS Challenge},
    author={Luis Contreras and Edgar Vazquez and Hugo Sanchez and Hiroyuki Okada},
    url={https://github.com/ARTenshi/wrs2020},
    year={2020}
}
```

* **Luis Contreras** - [AIBot Research Center](http://aibot.jp/)
* **Edgar Vazquez** - [Bio-Robotics Lab](https://biorobotics.fi-p.unam.mx/)
* **Hugo Sanchez** - [Bio-Robotics Lab](https://biorobotics.fi-p.unam.mx/)
* **Hiroyuki Okada** - [AIBot Research Center](http://aibot.jp/)
