//================================================ADAPTIVE FILTER=========================================================
//Project: EspeleoRobô2
//Institution: Universidade Federal de Minas Gerais (UFMG) and Instituto Tecnológico Vale (ITV);
//Description: This file uploads the setting constant parameters and libraries used in the other files of the package adap-
//             tive_filter. 
//Modification: 
//             Date: December 29, 2021
//             member: Gilmar Pereira da Cruz Júnior
//             e-mail: gilmarpcruzjunior@gmail.com
//========================================================================================================================
#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_


#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Time.h>
#include <std_msgs/Header.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include <time.h>
#include <stdint.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#define PI 3.14159265

using namespace std;

const double inf = std::numeric_limits<double>::infinity();

struct bias{ 
    double x;
    double y;
    double z;
};

#endif
