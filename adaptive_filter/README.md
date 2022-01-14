# Adaptive Filter Package


Adaptive Filter Package is a filter that provides sensor fusion for the best accuracy in LiDAR SLAM odometry input, fusing wheel odometry with inertial sensor data and LiDAR odometry. This filter is part of the EKF-LOAM package, that is an update of the LeGO-LOAM, and has the adaptive covariances for wheel and LiDAR odometry. Depending on the angular velocity of the robot, two matrices are considered for wheel odometry. The LiDAR odometry covariance matrix depends on the number of edge and planar features identified in the environment.

## The System:

LiDAR SLAM techniques accumulate significant errors when estimating the device's position in environments with few geometric features, such as tunnels, galleries, or even extensive and homogeneous corridors. These environments characterize several places in which the EspeleoRobô is supposed to operate. Aiming to correct eventual errors in the odometry and map computation, we use a filter to incorporate the measurement of wheel speed with the IMU data in the LiDAR SLAM. This information is used between the stages of the Back-End and Front-End of the SLAM. Fig.1 depicts the proposed structure.


<p align='center'>
    <img src="ReadMe/ekf_loam.png" alt="center" width="550"/>
</p>


In environments with few geometric features, LiDAR odometry presents errors in estimating the pose propagated to the LiDAR mapping, causing misalignment between the set of points **L**[k] and the map **M**[k-1], and generating map deformations and incorrect pose estimations **x_M**[k]. Therefore, our strategy uses an EKF to correct the pose estimate **x_L**[k] by combining LiDAR odometry with wheel odometry and IMU data. Thus, we propose an adaptive covariance matrix for LiDAR odometry according to the number of features identified in the environment. The new corrected pose is used in the mapping module to align the set **L**[k] with the map. Likewise, the corrected information is also used to integrate the transformations that generate the estimated output pose. The EKF integrated with the LiDAR SLAM allows to correct the pose estimation and the map in low feature environments.


## Parameters

This package has some configurable parameters, which can be found in the folder:


```
/adaptive_filter/Config/adaptive_filter_parameters.yaml
```

The parameters of this package are:


- Boolean Variables:

> - `enableFilter`: Boolean variable to enable or disable the filter;
> - `enableImu`: Boolean variable to enable or disable the IMU data;
> - `enableWheel`: Boolean variable to enable or disable the wheel odometry;
> - `enableLidar`: Boolean variable to enable or disable the LiDAR odometry;

- Set Frequency:

> - `enableFreq`: Char variable to set the frequency of the output, where "l" represent the same frenquency of the LiDAR odmoetry, "w" the same frequency of the wheel odometry and "i" the same frequency of the IMU data.

## Input and Output:

This package has three inputs and one output in the form of a ROS topic. Input topic names are defined below in which:

- `/odom`: is the odomtry topic of the wheel odometry;
- `/imu/data`: is the IMU sensor message topics of the inertial sensor with the orientation filtered;
- `/ekf_loam/laser_odom_to_init`: is the odometry topic of the LiDAR odometry.

The output topic name is defined as:

- `/ekf_loam/ad\ptiveFilter`: is the odomtry topic of the wheel odometry;

