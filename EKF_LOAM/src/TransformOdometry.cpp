
//=====================================================EKF-LOAM=============================================================
//Project: EspeleoRobô2
//Institution: Universidade Federal de Minas Gerais and ITV (Instito Tecnológico Vale)
//Description: This file transform the poses generation by the lidar-odometry to merge on the filter.
//Modification: 
//             Date: November 27, 2021
//             member: Gilmar Pereira da Cruz Júnior
//             e-mail: gilmarpcruzjunior@gmail.com
//=========================================================================================================================

#include "settings_ekf_loam.h"

using namespace std;

//variables
std::string direction;

Eigen::Matrix3d Ro;
Eigen::MatrixXd Ho(4,4), H1(4,4), H2(4,4), H(4,4);

class TransformOdometry{

private:
    // NodeHandler
    ros::NodeHandle nh;

    // time
    ros::Time Time_tf;

    // Subscribe and Publisher
    ros::Subscriber subOdometry;
    ros::Publisher pubOdometryOut;

    // Odometry
    nav_msgs::Odometry odometryOut;

    // String
    std::string childFrameId;
    std::string frameId;
    
public:

    TransformOdometry():
        nh("~")
        {

        if (direction=="foward"){
            //Subscriber topics
            subOdometry = nh.subscribe<nav_msgs::Odometry>("/ekf_loam/laser_odom_to_init", 5, &TransformOdometry::odometryHandler, this);
            //publisher topics  
            pubOdometryOut = nh.advertise<nav_msgs::Odometry> ("/ekf_loam/laser_odom_to_initOut", 5);
        }else if (direction=="back"){
            //Subscriber topics
            subOdometry = nh.subscribe<nav_msgs::Odometry>("/ekf_loam/filter_odom_to_init", 5, &TransformOdometry::odometryHandler, this);
            //publisher topics  
            pubOdometryOut = nh.advertise<nav_msgs::Odometry> ("/ekf_loam/filter_odom_to_initOut", 5);
        }else{
            cout << "Direction for odometry transformation not identified!";

        }

        initialization();
        
    }

    ~TransformOdometry(){}

    void initialization(){
        // initialization
        H1 << 0, 0, 1, 0, // x
              1, 0, 0, 0, // y
              0, 1, 0, 0, // z 
              0, 0, 0, 1; // (x) foward (z) up
        H2 << 0, 1, 0, 0, 
              0, 0, 1, 0, 
              1, 0, 0, 0, 
              0, 0, 0, 1; // (z) foward (y) up
    }

    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odometryMsg){
        Time_tf = odometryMsg->header.stamp;

        Eigen::Quaterniond q;
        q.x() = odometryMsg->pose.pose.orientation.x;
        q.y() = odometryMsg->pose.pose.orientation.y;
        q.z() = odometryMsg->pose.pose.orientation.z;
        q.w() = odometryMsg->pose.pose.orientation.w;
        Ro = q.toRotationMatrix();

        Ho.block(0,0,3,3) = Ro;
        Ho.block(0,3,3,1) << odometryMsg->pose.pose.position.x, odometryMsg->pose.pose.position.y, odometryMsg->pose.pose.position.z;
        Ho.block(3,0,1,4) << 0,0,0,1;

        if (direction=="foward"){
            H = H1*Ho*H1.inverse();
            childFrameId = "/lidarFilterIn";
            frameId = "/os1_initial";
            odometryOut.header.stamp = odometryMsg->header.stamp;

            // covariance
            odometryOut.pose.covariance[0] = odometryMsg->pose.covariance[14];
            odometryOut.pose.covariance[7] = odometryMsg->pose.covariance[0];
            odometryOut.pose.covariance[14] = odometryMsg->pose.covariance[7]; 
            odometryOut.pose.covariance[21] = odometryMsg->pose.covariance[35]; 
            odometryOut.pose.covariance[28] = odometryMsg->pose.covariance[21]; 
            odometryOut.pose.covariance[35] = odometryMsg->pose.covariance[28];

        }else if (direction=="back"){
            H = H2*Ho*H2.inverse();
            childFrameId = "/lidarFilterOut";
            frameId = "/chassis_init";

            // covariance
            odometryOut.header.stamp = ros::Time::now();
            odometryOut.pose.covariance = odometryMsg->pose.covariance; 
        }

        Eigen::Matrix3d R = H.block(0,0,3,3);
        Eigen::Quaterniond q_out(R);

        odometryOut.header.frame_id = frameId;
        odometryOut.child_frame_id = childFrameId;
        
        odometryOut.pose.pose.orientation.x = q_out.x();
        odometryOut.pose.pose.orientation.y = q_out.y();
        odometryOut.pose.pose.orientation.z = q_out.z();
        odometryOut.pose.pose.orientation.w = q_out.w();
        odometryOut.pose.pose.position.x = H(0,3); 
        odometryOut.pose.pose.position.y = H(1,3); 
        odometryOut.pose.pose.position.z = H(2,3); 

        // velocities:
        odometryOut.twist.twist.linear = odometryMsg->twist.twist.linear;
        odometryOut.twist.twist.angular = odometryMsg->twist.twist.angular;
        
        // publish
        pubOdometryOut.publish(odometryOut);

    }

};

int main (int argc, char **argv)
{
    // node init
    ros::init(argc, argv, "TransformOdometry");
    
    ROS_INFO("\033[1;32m---->\033[0m Odometry Transform Started.");

    try
    {
        direction = argv[1];
    }
    catch (int e)
    {
        ROS_INFO("\033[1;31m---->\033[0m Exception occurred when importing parameters in Transform Odometry Node. Exception Nr. %d", e);
    }

    TransformOdometry SC3D;

    ros::spin();
    return 0;
}