//=====================================================EKF-LOAM=============================================================
//Project: EspeleoRobô2
//Institution: Universidade Federal de Minas Gerais and ITV (Instito Tecnológico Vale)
//Description: This file merges the transformation of the poses generation by the lidar-odometry and lidar-mapping algorithm.
//             This is a modification of original ROS package LeGO-LOAM, where you find in the link below.
//             https://github.com/RobustFieldAutonomyLab/LeGO-LOAM.
//Modification: 
//             Date: November 25, 2021
//             member: Gilmar Pereira da Cruz Júnior
//             e-mail: gilmarpcruzjunior@gmail.com
//=========================================================================================================================

#include "settings_ekf_loam.h"

tf::StampedTransform pose;

bool enableFilter;

std::string init_frame;
std::string lidar_integrated_frame;
std::string inertial_frame;
std::string base_frame;
std::string integrated_frame;

class IntegrationTransform{

private:
    // NodeHandle
    ros::NodeHandle nh;

    // Subscribers and publisher
    ros::Publisher pubLaserOdometry2;
    ros::Subscriber subLaserOdometry;
    ros::Subscriber subOdomAftMapped;  

    // Odometry and tf
    nav_msgs::Odometry laserOdometry2;
    tf::StampedTransform laserOdometryTrans2;
    tf::TransformBroadcaster tfBroadcaster2;
    tf::StampedTransform laserOdometryTrans3;
    tf::TransformBroadcaster tfBroadcaster3;

    tf::StampedTransform map_2_camera_init_Trans;
    tf::TransformBroadcaster tfBroadcasterMap2CameraInit;

    tf::StampedTransform camera_2_base_link_Trans;
    tf::TransformBroadcaster tfBroadcasterCamera2Baselink;

    // poses
    float transformSum[6];
    float transformIncre[6];
    float transformMapped[6];
    float transformBefMapped[6];
    float transformAftMapped[6];

    // header
    std_msgs::Header currentHeader;

public:

    IntegrationTransform(){

        // Publisher
        pubLaserOdometry2 = nh.advertise<nav_msgs::Odometry> ("/ekf_loam/integrated_to_init", 5);

        // Subscribers
        subOdomAftMapped = nh.subscribe<nav_msgs::Odometry>("/ekf_loam/aft_mapped_to_init", 5, &IntegrationTransform::odomAftMappedHandler, this);
        // lidar odometry or filtered odometry
        if (enableFilter){
            subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/ekf_loam/filter_odom_to_initOut", 5, &IntegrationTransform::laserOdometryHandler, this);
        }else{
            subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/ekf_loam/laser_odom_to_init", 5, &IntegrationTransform::laserOdometryHandler, this);
        } 
        
        // initialization
        initialization();
        
    }

    void initialization(){
        laserOdometry2.header.frame_id = init_frame;     
        laserOdometry2.child_frame_id = lidar_integrated_frame; 

        laserOdometryTrans2.frame_id_ = inertial_frame; 
        laserOdometryTrans2.child_frame_id_ = integrated_frame;

        laserOdometryTrans3.frame_id_ = init_frame; 
        laserOdometryTrans3.child_frame_id_ = lidar_integrated_frame;

        map_2_camera_init_Trans.frame_id_ = init_frame;
        map_2_camera_init_Trans.child_frame_id_ = inertial_frame;

        camera_2_base_link_Trans.frame_id_ = integrated_frame; 
        camera_2_base_link_Trans.child_frame_id_ = base_frame;

        for (int i = 0; i < 6; ++i)
        {
            transformSum[i] = 0;
            transformIncre[i] = 0;
            transformMapped[i] = 0;
            transformBefMapped[i] = 0;
            transformAftMapped[i] = 0;
        }
    }

    void transformAssociateToMap(){
        float x1 = cos(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) 
                 - sin(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);
        float y1 = transformBefMapped[4] - transformSum[4];
        float z1 = sin(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) 
                 + cos(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);

        float x2 = x1;
        float y2 = cos(transformSum[0]) * y1 + sin(transformSum[0]) * z1;
        float z2 = -sin(transformSum[0]) * y1 + cos(transformSum[0]) * z1;

        transformIncre[3] = cos(transformSum[2]) * x2 + sin(transformSum[2]) * y2;
        transformIncre[4] = -sin(transformSum[2]) * x2 + cos(transformSum[2]) * y2;
        transformIncre[5] = z2;

        float sbcx = sin(transformSum[0]);
        float cbcx = cos(transformSum[0]);
        float sbcy = sin(transformSum[1]);
        float cbcy = cos(transformSum[1]);
        float sbcz = sin(transformSum[2]);
        float cbcz = cos(transformSum[2]);

        float sblx = sin(transformBefMapped[0]);
        float cblx = cos(transformBefMapped[0]);
        float sbly = sin(transformBefMapped[1]);
        float cbly = cos(transformBefMapped[1]);
        float sblz = sin(transformBefMapped[2]);
        float cblz = cos(transformBefMapped[2]);

        float salx = sin(transformAftMapped[0]);
        float calx = cos(transformAftMapped[0]);
        float saly = sin(transformAftMapped[1]);
        float caly = cos(transformAftMapped[1]);
        float salz = sin(transformAftMapped[2]);
        float calz = cos(transformAftMapped[2]);

        float srx = -sbcx*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz)
                  - cbcx*sbcy*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                  - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
                  - cbcx*cbcy*(calx*salz*(cblz*sbly - cbly*sblx*sblz) 
                  - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx);
        transformMapped[0] = -asin(srx);

        float srycrx = sbcx*(cblx*cblz*(caly*salz - calz*salx*saly)
                     - cblx*sblz*(caly*calz + salx*saly*salz) + calx*saly*sblx)
                     - cbcx*cbcy*((caly*calz + salx*saly*salz)*(cblz*sbly - cbly*sblx*sblz)
                     + (caly*salz - calz*salx*saly)*(sbly*sblz + cbly*cblz*sblx) - calx*cblx*cbly*saly)
                     + cbcx*sbcy*((caly*calz + salx*saly*salz)*(cbly*cblz + sblx*sbly*sblz)
                     + (caly*salz - calz*salx*saly)*(cbly*sblz - cblz*sblx*sbly) + calx*cblx*saly*sbly);
        float crycrx = sbcx*(cblx*sblz*(calz*saly - caly*salx*salz)
                     - cblx*cblz*(saly*salz + caly*calz*salx) + calx*caly*sblx)
                     + cbcx*cbcy*((saly*salz + caly*calz*salx)*(sbly*sblz + cbly*cblz*sblx)
                     + (calz*saly - caly*salx*salz)*(cblz*sbly - cbly*sblx*sblz) + calx*caly*cblx*cbly)
                     - cbcx*sbcy*((saly*salz + caly*calz*salx)*(cbly*sblz - cblz*sblx*sbly)
                     + (calz*saly - caly*salx*salz)*(cbly*cblz + sblx*sbly*sblz) - calx*caly*cblx*sbly);
        transformMapped[1] = atan2(srycrx / cos(transformMapped[0]), 
                                   crycrx / cos(transformMapped[0]));
        
        float srzcrx = (cbcz*sbcy - cbcy*sbcx*sbcz)*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
                     - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx)
                     - (cbcy*cbcz + sbcx*sbcy*sbcz)*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                     - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
                     + cbcx*sbcz*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz);
        float crzcrx = (cbcy*sbcz - cbcz*sbcx*sbcy)*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                     - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
                     - (sbcy*sbcz + cbcy*cbcz*sbcx)*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
                     - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx)
                     + cbcx*cbcz*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz);
        transformMapped[2] = atan2(srzcrx / cos(transformMapped[0]), 
                                   crzcrx / cos(transformMapped[0]));

        x1 = cos(transformMapped[2]) * transformIncre[3] - sin(transformMapped[2]) * transformIncre[4];
        y1 = sin(transformMapped[2]) * transformIncre[3] + cos(transformMapped[2]) * transformIncre[4];
        z1 = transformIncre[5];

        x2 = x1;
        y2 = cos(transformMapped[0]) * y1 - sin(transformMapped[0]) * z1;
        z2 = sin(transformMapped[0]) * y1 + cos(transformMapped[0]) * z1;

        transformMapped[3] = transformAftMapped[3] 
                           - (cos(transformMapped[1]) * x2 + sin(transformMapped[1]) * z2);
        transformMapped[4] = transformAftMapped[4] - y2;
        transformMapped[5] = transformAftMapped[5] 
                           - (-sin(transformMapped[1]) * x2 + cos(transformMapped[1]) * z2);
    }

    void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry){
        currentHeader = laserOdometry->header;

        double roll, pitch, yaw;
        geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
        tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

        transformSum[0] = -pitch;
        transformSum[1] = -yaw;
        transformSum[2] = roll;

        transformSum[3] = laserOdometry->pose.pose.position.x;
        transformSum[4] = laserOdometry->pose.pose.position.y;
        transformSum[5] = laserOdometry->pose.pose.position.z;

        transformAssociateToMap();

        geoQuat = tf::createQuaternionMsgFromRollPitchYaw
                  (transformMapped[2], -transformMapped[0], -transformMapped[1]);

        Eigen::Matrix3d R_r_r0, R_l_l0, R_l_r;
        Eigen::MatrixXd H_r_r0(4,4), H_l_l0(4,4), H_l_r(4,4);

        //Transformation matriz of the frame_map to frame_init
        Eigen::Quaterniond q1;
        q1.x() = pose.getRotation().x();
        q1.y() = pose.getRotation().y();
        q1.z() = pose.getRotation().z();
        q1.w() = pose.getRotation().w();
        R_l_r = q1.toRotationMatrix();

        H_l_r.block(0,0,3,3) = R_l_r;
        H_l_r.block(0,3,3,1) << pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z();
        H_l_r.block(3,0,1,4) << 0,0,0,1;

        //Transformation matriz of odometry
        Eigen::Quaterniond q;
        q.x() = -geoQuat.y;
        q.y() = -geoQuat.z;
        q.z() = geoQuat.x;
        q.w() = geoQuat.w;
        R_l_l0 = q.toRotationMatrix();

        H_l_l0.block(0,0,3,3) = R_l_l0;
        H_l_l0.block(0,3,3,1) << transformMapped[3], transformMapped[4], transformMapped[5];
        H_l_l0.block(3,0,1,4) << 0,0,0,1; 

        //Similarity transformation
        H_r_r0 = H_l_r*H_l_l0*H_l_r.inverse();

        R_r_r0 = H_r_r0.block(0,0,3,3);

        Eigen::Quaterniond q_r_r0(R_r_r0);

        laserOdometry2.header.stamp = laserOdometry->header.stamp;
        laserOdometry2.pose.pose.orientation.x = q_r_r0.x();
        laserOdometry2.pose.pose.orientation.y = q_r_r0.y();
        laserOdometry2.pose.pose.orientation.z = q_r_r0.z();
        laserOdometry2.pose.pose.orientation.w = q_r_r0.w();
        laserOdometry2.pose.pose.position.x = H_r_r0(0,3); //5
        laserOdometry2.pose.pose.position.y = H_r_r0(1,3); //3
        laserOdometry2.pose.pose.position.z = H_r_r0(2,3); //4

        // linear velocity:
        laserOdometry2.twist.twist.linear = laserOdometry->twist.twist.linear;
        // angular velocity
        laserOdometry2.twist.twist.angular = laserOdometry->twist.twist.angular;
                
        pubLaserOdometry2.publish(laserOdometry2);

        laserOdometryTrans2.stamp_ = laserOdometry->header.stamp;
        laserOdometryTrans2.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
        laserOdometryTrans2.setOrigin(tf::Vector3(transformMapped[3], transformMapped[4], transformMapped[5]));
        tfBroadcaster2.sendTransform(laserOdometryTrans2);

        laserOdometryTrans3.stamp_ = laserOdometry->header.stamp;
        laserOdometryTrans3.setRotation(tf::Quaternion(q_r_r0.x(), q_r_r0.y(), q_r_r0.z(), q_r_r0.w()));
        laserOdometryTrans3.setOrigin(tf::Vector3(H_r_r0(0,3), H_r_r0(1,3), H_r_r0(2,3)));
        tfBroadcaster3.sendTransform(laserOdometryTrans3);
    }

    void odomAftMappedHandler(const nav_msgs::Odometry::ConstPtr& odomAftMapped){
        double roll, pitch, yaw;
        geometry_msgs::Quaternion geoQuat = odomAftMapped->pose.pose.orientation;
        tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

        transformAftMapped[0] = -pitch;
        transformAftMapped[1] = -yaw;
        transformAftMapped[2] = roll;

        transformAftMapped[3] = odomAftMapped->pose.pose.position.x;
        transformAftMapped[4] = odomAftMapped->pose.pose.position.y;
        transformAftMapped[5] = odomAftMapped->pose.pose.position.z;

        transformBefMapped[0] = odomAftMapped->twist.twist.angular.x;
        transformBefMapped[1] = odomAftMapped->twist.twist.angular.y;
        transformBefMapped[2] = odomAftMapped->twist.twist.angular.z;

        transformBefMapped[3] = odomAftMapped->twist.twist.linear.x;
        transformBefMapped[4] = odomAftMapped->twist.twist.linear.y;
        transformBefMapped[5] = odomAftMapped->twist.twist.linear.z;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ekf_loam");

    ROS_INFO("\033[1;32m---->\033[0m Integration Transform Started.");

    //Parameters init:    
    ros::NodeHandle nh_;
    try
    {
        nh_.param("/ekf_loam/enableFilter", enableFilter, false);

        nh_.param("/ekf_loam/init_frame", init_frame, std::string("os1_init"));
        nh_.param("/ekf_loam/lidar_integrated_frame", lidar_integrated_frame, std::string("lidar_integrate_odom"));
        nh_.param("/ekf_loam/integrated_frame", integrated_frame, std::string("os1_integrate_odom"));
        nh_.param("/ekf_loam/inertial_frame", inertial_frame, std::string("os1_initial"));
        nh_.param("/ekf_loam/base_frame", base_frame, std::string("base_link"));

        }
    catch (int e)
    {
        ROS_INFO("\033[1;31m---->\033[0m Exception occurred when importing parameters in Integration Transform Node. Exception Nr. %d", e);
    }

    tf::TransformListener listener;

    bool tf_read = false;

    while (!tf_read){
        try{
            listener.lookupTransform(init_frame, inertial_frame, ros::Time(0), pose);
            tf_read = true;
        }
        catch (tf::TransformException ex){
            ros::Duration(1.0).sleep();
        }
    }    
    
    IntegrationTransform IT;    

    ros::spin();

    return 0;
}