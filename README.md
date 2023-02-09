# LPMS-IG1/BE/NAV3/CURS3/INC1 Series OpenSource Lib


## Usage
### 1.Compiling Library programs
```bash
    $ cd ~
    $ git clone https://bitbucket.org/lpresearch/lpmsig1opensourcelib
    $ cd lpmsig1opensourcelib
    $ mkdir build
    $ cd build
    $ cmake ..
    $ make
    $ make package
    $ sudo dpkg -i libLpmsIG1_OpenSource-x.y.z-Linux.deb
```

### 2.Compiling Sample programs
```bash
    $ cd linux_example
    $ mkdir build
    $ cd build
    $ cmake ..
    $ make
    $ ./LpmsIG1SimpleExample
```
You might require running command using sudo if current user is not in dialout group. See [Troubleshooting](#troubleshooting) 

### 3.Compiling ROS Example programs
```bash
# Create ROS workspace.
    $ mkdir -p ~/catkin_ws/src
    $ cd ~/catkin_ws/src

# We suggest to create a symbolic link to ros_example folder instead of copying
    $ ln -s ~/lpmsig1opensourcelib/ros_example lpms_ig1_ros_example

# Compiling ROS example programs
    $ cd ~/catkin_ws
    $ catkin_make
    $ source ./devel/setup.bash
```

Open a new terminal window and run roscore
```bash
    $ roscore
```
Connect `LPMS-IG1/BE/NAV3/CURS3` sensor to PC.
Now you can run lpms_ig1 node on your other terminal windows.
The following are some example commands to launch ros lpms_ig1 node. Please change the parameters appropriately according to your sensor settings:

*IG1*:
```bash
    $ rosrun lpms_ig1 lpms_ig1_node _port:=LPMSIG1000001 _baudrate:=921600
```

*IG1-RS485*:
```bash
    $ rosrun lpms_ig1 lpms_ig1_rs485_node _port:=/dev/ttyTHS2  _baudrate:=115200 _rs485ControlPin:=388 _rs485ControlPinToggleWaitMs:=2

```

*BE*:
```bash
    $ rosrun lpms_ig1 lpms_be1_node _port:=/dev/ttyUSB0 _baudrate:=115200
```

*NAV3*:
```bash
    $ rosrun lpms_ig1 lpms_nav3_node _port:=/dev/ttyUSB0 _baudrate:=115200
```

*CU3/CURS3*:
```bash
    $ rosrun lpms_ig1 lpms_curs3_node _port:=/dev/ttyUSB0 _baudrate:=921600
```

Please refer to [Troubleshooting](#troubleshooting) section is error occurs.

You can print out the imu data by subscribing to `/imu/data` topic
```bash
#Show imu message.
    $ rostopic echo /imu/data
#Plot imu message.
    $ rosrun rqt_plot rqt_plot
```

Alternatively, you can use the sample launch file (lpmsig1.launch) start data acquisition and data plotting:

*IG1*:
```
roslaunch lpms_ig1 lpmsig1.launch
```

*IG1-RS485*:
```
roslaunch lpms_ig1 lpmsig1_rs485.launch
```

*BE1*:
```
roslaunch lpms_ig1 lpmsbe1.launch
```

*NAV3*:
```
roslaunch lpms_ig1 lpmsnav3.launch
```

*CU3/CURS3*:
```
roslaunch lpms_ig1 lpmscurs3.launch
```

## Troubleshooting

### Serial connection error
If prompted with the following information：
```bash
    $ rosrun lpms_ig1 lpms_ig1_node
    #Error opening [IG1] COM:/dev/ttyUSB0 disconnected
```
this error might be due to the fact that the current user does not have sufficient permission to access the device. 
To allow access to sensors connected via USB, you need to ensure that the user running the ROS sensor node has access to the /dev/ttyUSB devices. You can do this by adding the user to the dialout group. After this call, you should logout and login with this user to ensure the changed permissions are in effect.

```bash
    $ sudo adduser <username> dialout
```



## ROS Package Summary

### 1. Supported Hardware
This driver interfaces with LPMS-IG1 IMU sensor from LP-Research Inc.


### 2.1 lpms_ig1_node / lpms_ig1_rs485_node  / lpms_be1_node / lpms_nav3_node / lpms_curs3_node
lpms_ig1_node is a driver for the LPMS-IG1 Inertial Measurement Unit. It publishes orientation, angular velocity, linear acceleration and magnetometer data (covariances are not yet supported), and complies with the [Sensor message](https://wiki.ros.org/sensor_msgs) for [IMU API](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html) and [MagneticField](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/MagneticField.html) API.

Similarly lpms_ig1_rs485_node is a driver for the LPMS-IG1-RS485 Inertial Measurement Unit.

lpms_be1_node is a driver for the LPMS-BE1/BE2 Inertial Measurement Unit.

lpms_nav3_node is a driver for the LPMS-NAV3 Inertial Measurement Unit.

lpms_curs3_node is a driver for the LPMS-CU3/CURS3 Inertial Measurement Unit.

#### 2.1.1 Published Topics
/imu/data ([sensor_msgs/Imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html)) 
:   Inertial data from the IMU. Includes calibrated acceleration, calibrated angular rates and orientation. The orientation is always unit quaternion. 

/imu/mag ([sensor_msgs/MagneticField](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/MagneticField.html)) *IG1 Series only*
:   Magnetometer reading from the sensor.

/imu/is_autocalibration_active ([std_msgs/Bool](http://docs.ros.org/api/std_msgs/html/msg/Bool.html), default: True)
:   Latched topic indicating if the gyro autocalibration feature is active

#### 2.1.2 Services
/imu/calibrate_gyroscope ([std_srvs/Empty](http://docs.ros.org/api/std_srvs/html/srv/Empty.html)) 
:   This service activates the IMU internal gyro bias estimation function. Please make sure the IMU sensor is placed on a stable platform with minimal vibrations before calling the service. Please make sure the sensor is stationary for at least 4 seconds. The service call returns a success response once the calibration procedure is completed.

/imu/reset_heading ([std_srvs/Empty](http://docs.ros.org/api/std_srvs/html/srv/Empty.html)) 
:   This service will reset the heading (yaw) angle of the sensor to zero. 

/imu/enable_gyro_autocalibration ([std_srvs/SetBool](http://docs.ros.org/melodic/api/std_srvs/html/srv/SetBool.html))
:   Turn on/off autocalibration function in the IMU. The status of autocalibration can be obtained by subscribing to the /imu/is_autocalibration_active topic. A message will published to /imu/is_autocalibration_active for each call to /imu/autocalibrate. 

/imu/enable_auto_reconnect ([std_srvs/SetBool](http://docs.ros.org/melodic/api/std_srvs/html/srv/SetBool.html))
:   Turn on/off auto reconnect function in the library. When auto reconnect is enabled, the library will reestablish connection to the sensor if it detects no data from the sensor for more than 5 seconds. With auto reconnect enabled, the library will automatically put the sensor into streaming mode. We recommend to disable auto reconnect if user plans to interact with sensor via polling method.

/imu/get_imu_data ([std_srvs/Empty](http://docs.ros.org/api/std_srvs/html/srv/Empty.html)) *IG1-RS485 only*
:   Poll imu sensor for latest sensor data. Latest sensor data will be publish to /imu/data topic once the driver received a new data from the sensor.

/imu/set_streaming_mode ([std_srvs/Empty](http://docs.ros.org/api/std_srvs/html/srv/Empty.html)) *IG1-RS485 only*
:   Put sensor into streaming mode

/imu/set_command_mode ([std_srvs/Empty](http://docs.ros.org/api/std_srvs/html/srv/Empty.html)) *IG1-RS485 only*
:   Put sensor into command mode

#### 2.1.3 Parameters

~port (string, default: /dev/ttyUSB0) 
:   The port the IMU is connected to.

~baudrate (int, default IG1: 921600, IG1-RS485: 115200, BE1: 115200)
:   Baudrate for the IMU sensor.

~autoreconnect (bool, default True)
:   Enable/disable autoreconnect in library

~frame_id (string, default: imu) 
:   The frame in which imu readings will be returned.

~rate (int, default: 200) 
:   Data processing rate of the internal loop. This rate has to be equal or larger than the data streaming frequency of the sensor to prevent internal data queue overflow.

~rs485ControlPin (int, default: -1 (disabled)) *IG1-RS485 only* 
:   RS485 send/receive control pin. The driver will automatically toggle this pin to HIGH when sending message and set the pin to LOW once data is sent.

~rs485ControlPinToggleWaitMs (int, default(ms): 1 ) *IG1-RS485 only* 
:   Delay (in millisecond) to wait before rs485ControlPin is toggled before/after sending data. 

For further information on `LP-Research`, please visit our website:

* http://www.lp-research.com

* http://www.alubi.cn
