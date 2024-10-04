
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

class TransformProjected{

private:
    // NodeHandler
    ros::NodeHandle nh;

    // Subscribe and Publisher
    ros::Subscriber subOdometry;
    ros::Publisher pubOdometryOut;

    // Odometry
    nav_msgs::Odometry odometryOut;
    
public:

    TransformProjected():
        nh("~")
        {
            //Subscriber topics
            subOdometry = nh.subscribe<nav_msgs::Odometry>("/ekf_loam/integrated_to_init", 5, &TransformProjected::odometryHandler, this);
            //publisher topics  
            pubOdometryOut = nh.advertise<nav_msgs::Odometry> ("/ekf_loam/integrated_projected", 5);

        initialization();
        
    }

    ~TransformProjected(){}

    void initialization(){
        double a = 0;
    }

    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odometryMsg){        
        odometryOut.header = odometryMsg->header;
        odometryOut.child_frame_id = odometryMsg->child_frame_id;
        odometryOut.pose = odometryMsg->pose;
        odometryOut.twist = odometryMsg->twist;

        double x, y, z, d, alpha;

        x = odometryMsg->pose.pose.position.x;
        y = odometryMsg->pose.pose.position.y;
        z = odometryMsg->pose.pose.position.z;

        d = sqrt(pow(x,2) + pow(y,2) + pow(z,2));
        alpha = atan2(y,x);
        
        odometryOut.pose.pose.position.x = d*cos(alpha); 
        odometryOut.pose.pose.position.y = d*sin(alpha); 
        odometryOut.pose.pose.position.z = 0.0;
        
        // publish
        pubOdometryOut.publish(odometryOut);

    }

};

int main (int argc, char **argv)
{
    // node init
    ros::init(argc, argv, "TransformProjected");
    
    ROS_INFO("\033[1;32m---->\033[0m Odometry Projected Started.");

    TransformProjected TP;

    ros::spin();
    return 0;
}