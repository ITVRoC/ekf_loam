//=====================================================EKF-LOAM=========================================================
//Project: EspeleoRobô2
//Institution: Universidade Federal de Minas Gerais and ITV (Instito Tecnológico Vale)
//Description: This file projects the point cloud in a segmented image. This is a modification of original ROS package LeGO-
//             LOAM, where you find in the link below.
//             https://github.com/RobustFieldAutonomyLab/LeGO-LOAM.
//Modification: 
//             Date: November 27, 2021
//             member: Gilmar Pereira da Cruz Júnior
//             e-mail: gilmarpcruzjunior@gmail.com
//========================================================================================================================

#include "settings_ekf_loam.h"

using namespace std;

//-----------------------------
// Global variables
//-----------------------------
float segmentThetanum;
float segmentTheta; 
float segmentAlphaX;
float segmentAlphaY;
bool enableFlatGround;

float opening_vertical_angle;
float opening_horizon_angle;
float start_angle_v;
float start_angle_h;
float ang_res_h;
float ang_res_v;

float sensorMountAngle;

int groundScanInd;
int H_SCAN;
int V_SCAN;
int segmentValidPointNum;
int segmentValidLineNum;

std::string lidar_frame;

//-----------------------------
// Pre-Treatment class
//-----------------------------
class PreTreatment{
private:
    // NodeHandle
    ros::NodeHandle nh;

    // Subscribers and Publishers
    ros::Subscriber subLaserCloud;
    
    ros::Publisher pubFullCloud;
    ros::Publisher pubFullInfoCloud;

    ros::Publisher pubGroundCloud;
    ros::Publisher pubSegmentedCloud;
    ros::Publisher pubSegmentedCloudPure;
    ros::Publisher pubSegmentedCloudInfo;
    ros::Publisher pubOutlierCloud;

    ros::Publisher pubTime;

    // Varible to manipulating point clouds
    pcl::PointCloud<PointType>::Ptr laserCloudIn;
    pcl::PointCloud<PointType>::Ptr cloudIn;

    pcl::PointCloud<PointType>::Ptr fullCloud; // projected velodyne raw cloud, but saved in the form of 1-D matrix
    pcl::PointCloud<PointType>::Ptr fullInfoCloud; // same as fullCloud, but with intensity - range

    pcl::PointCloud<PointType>::Ptr groundCloud;
    pcl::PointCloud<PointType>::Ptr segmentedCloud;
    pcl::PointCloud<PointType>::Ptr segmentedCloudPure;
    pcl::PointCloud<PointType>::Ptr outlierCloud;

    PointType nanPoint; // fill in fullCloud at each iteration

    // For matrices
    cv::Mat rangeMat; // range matrix for range image
    cv::Mat labelMat; // label matrix for segmentaiton marking
    cv::Mat groundMat; // ground matrix for ground cloud marking
    int labelCount;

    // Info of segmented cloud
    cloud_msgs::cloud_info segMsg; // info of segmented cloud
    std_msgs::Header cloudHeader;

    // Neighbor iterator for segmentaiton process
    std::vector<std::pair<int8_t, int8_t> > neighborIterator; 

    // array for tracking points of a segmented object
    uint16_t *allPushedIndX; 
    uint16_t *allPushedIndY;

    // array for breadth-first search process of segmentation
    uint16_t *queueIndX; 
    uint16_t *queueIndY;

    // TFs
    bool tf_read = false;
    tf::TransformListener listener;
    tf::StampedTransform pose;

    // start and end points
    bool start_angle = false;

    // Times:
    double timeInit;  
    double timeEnd;

public:
    PreTreatment():
        nh("~"){

        // Subscribers
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/os1_cloud_node/points", 1, &PreTreatment::cloudHandler, this);

        // Publishers
        pubFullCloud = nh.advertise<sensor_msgs::PointCloud2> ("/ekf_loam/full_cloud_projected", 1);
        pubFullInfoCloud = nh.advertise<sensor_msgs::PointCloud2> ("/ekf_loam/full_cloud_info", 1);  
        pubGroundCloud = nh.advertise<sensor_msgs::PointCloud2> ("/ekf_loam/ground_cloud", 1);
        pubSegmentedCloud = nh.advertise<sensor_msgs::PointCloud2> ("/ekf_loam/segmented_cloud", 1); 
        pubSegmentedCloudPure = nh.advertise<sensor_msgs::PointCloud2> ("/ekf_loam/segmented_cloud_pure", 1);
        pubSegmentedCloudInfo = nh.advertise<cloud_msgs::cloud_info> ("/ekf_loam/segmented_cloud_info", 1);  
        pubOutlierCloud = nh.advertise<sensor_msgs::PointCloud2> ("/ekf_loam/outlier_cloud", 1);         
        pubTime = nh.advertise<std_msgs::Time> ("/ekf_loam/PreTreatmentTime", 1); 

        allocateMemory();
        resetParameters();
    }

    void allocateMemory(){
        nanPoint.x = std::numeric_limits<float>::quiet_NaN();
        nanPoint.y = std::numeric_limits<float>::quiet_NaN();
        nanPoint.z = std::numeric_limits<float>::quiet_NaN();
        nanPoint.intensity = -1;

        laserCloudIn.reset(new pcl::PointCloud<PointType>());
        cloudIn.reset(new pcl::PointCloud<PointType>());

        fullCloud.reset(new pcl::PointCloud<PointType>());
        fullInfoCloud.reset(new pcl::PointCloud<PointType>());

        groundCloud.reset(new pcl::PointCloud<PointType>());
        segmentedCloud.reset(new pcl::PointCloud<PointType>());
        segmentedCloudPure.reset(new pcl::PointCloud<PointType>());
        outlierCloud.reset(new pcl::PointCloud<PointType>());

        fullCloud->points.resize(V_SCAN*H_SCAN);
        fullInfoCloud->points.resize(V_SCAN*H_SCAN);

        segMsg.startRingIndex.assign(V_SCAN, 0);
        segMsg.endRingIndex.assign(V_SCAN, 0);

        segMsg.segmentedCloudGroundFlag.assign(V_SCAN*H_SCAN, false);
        segMsg.segmentedCloudColInd.assign(V_SCAN*H_SCAN, 0);
        segMsg.segmentedCloudRange.assign(V_SCAN*H_SCAN, 0);

        std::pair<int8_t, int8_t> neighbor;
        neighbor.first = -1; neighbor.second =  0; neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second =  1; neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second = -1; neighborIterator.push_back(neighbor);
        neighbor.first =  1; neighbor.second =  0; neighborIterator.push_back(neighbor);

        allPushedIndX = new uint16_t[V_SCAN*H_SCAN];
        allPushedIndY = new uint16_t[V_SCAN*H_SCAN];

        queueIndX = new uint16_t[V_SCAN*H_SCAN];
        queueIndY = new uint16_t[V_SCAN*H_SCAN];
    }

    void resetParameters(){
        laserCloudIn->clear();
        cloudIn->clear();
        groundCloud->clear();
        segmentedCloud->clear();
        segmentedCloudPure->clear();
        outlierCloud->clear();

        rangeMat = cv::Mat(V_SCAN, H_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
        groundMat = cv::Mat(V_SCAN, H_SCAN, CV_8S, cv::Scalar::all(0));
        labelMat = cv::Mat(V_SCAN, H_SCAN, CV_32S, cv::Scalar::all(0));
        labelCount = 1;

        std::fill(fullCloud->points.begin(), fullCloud->points.end(), nanPoint);
        std::fill(fullInfoCloud->points.begin(), fullInfoCloud->points.end(), nanPoint);
    }

    ~PreTreatment(){}

    void copyPointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){        
        if (!tf_read){
            try{
                std::string frame_cloud = laserCloudMsg->header.frame_id;
                listener.lookupTransform(frame_cloud, lidar_frame, ros::Time(0), pose);
                tf_read = true;
            }
            catch (tf::TransformException ex){
                ROS_INFO("Waiting for Transformation");
            }
        }

        // If the Tf was read
        if (tf_read){
            cloudHeader = laserCloudMsg->header;
            cloudHeader.stamp = ros::Time::now(); // Ouster lidar users may need to uncomment this line
            pcl::fromROSMsg(*laserCloudMsg, *cloudIn);
            
            pcl_ros::transformPointCloud(*cloudIn, *laserCloudIn, pose);

            // Remove Nan points
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);
        }
    }
    
    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
        // Initial time processing pointcloud - Gilmar
        timeInit = ros::Time::now().toSec(); 

        // 1. Convert ros message to pcl point cloud
        copyPointCloud(laserCloudMsg);
        // 2. Start and end angle of a scan
        findStartEndAngle();
        // 3. Range image projection
        projectPointCloud();
        // 4. Mark ground points
        groundRemoval();
        // 5. Point cloud segmentation
        cloudSegmentation();
        // 6. Publish all clouds
        publishCloud();
        // 7. Reset parameters for next iteration
        resetParameters();
    }

    void findStartEndAngle(){
        // start and end orientation of this cloud
        segMsg.startOrientation = -atan2(laserCloudIn->points[0].y, laserCloudIn->points[0].x);
        segMsg.endOrientation   = -atan2(laserCloudIn->points[laserCloudIn->points.size() - 1].y,
                                                     laserCloudIn->points[laserCloudIn->points.size() - 1].x) + 2 * M_PI;
        if (segMsg.endOrientation - segMsg.startOrientation > 3 * M_PI) {
            segMsg.endOrientation -= 2 * M_PI;
        } else if (segMsg.endOrientation - segMsg.startOrientation < M_PI)
            segMsg.endOrientation += 2 * M_PI;
        segMsg.orientationDiff = segMsg.endOrientation - segMsg.startOrientation;
    }

    void projectPointCloud(){
        // range image projection
        float verticalAngle, horizonAngle, range;
        size_t rowIdn, columnIdn, index, cloudSize; 
        PointType thisPoint;

        cloudSize = laserCloudIn->points.size();

        for (size_t i = 0; i < cloudSize; ++i){

            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;

            if (enableFlatGround && thisPoint.z < -0.41){
                continue;
            }

            // find the row and column index in the iamge for this point
            verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
            rowIdn = (verticalAngle + start_angle_v) / ang_res_v;
            if (rowIdn < 0 || rowIdn >= V_SCAN)
                continue;

            horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

            columnIdn = -round((horizonAngle-90.0)/ang_res_h) + H_SCAN/2;
            if (columnIdn >= H_SCAN)
                columnIdn -= H_SCAN;

            if (columnIdn < 0 || columnIdn >= H_SCAN)
                continue;

            range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
            if (range < 0.1)
                continue;
            
            rangeMat.at<float>(rowIdn, columnIdn) = range;

            thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;

            index = columnIdn  + rowIdn * H_SCAN;
            fullCloud->points[index] = thisPoint;
            fullInfoCloud->points[index] = thisPoint;
            fullInfoCloud->points[index].intensity = range; // the corresponding range of a point is saved as "intensity"
        }
    }

    void groundRemoval(){
        size_t lowerInd, upperInd;
        float diffX, diffY, diffZ, angle;
        // groundMat
        // -1, no valid info to check if ground of not
        //  0, initial value, after validation, means not ground
        //  1, ground
        for (size_t j = 0; j < H_SCAN; ++j){
            for (size_t i = 0; i < groundScanInd; ++i){

                lowerInd = j + ( i )*H_SCAN;
                upperInd = j + (i+1)*H_SCAN;

                if (fullCloud->points[lowerInd].intensity == -1 ||
                    fullCloud->points[upperInd].intensity == -1){
                    // no info to check, invalid points
                    groundMat.at<int8_t>(i,j) = -1;
                    continue;
                }
                    
                diffX = fullCloud->points[upperInd].x - fullCloud->points[lowerInd].x;
                diffY = fullCloud->points[upperInd].y - fullCloud->points[lowerInd].y;
                diffZ = fullCloud->points[upperInd].z - fullCloud->points[lowerInd].z;

                angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY) ) * 180 / M_PI;

                if (abs(angle - sensorMountAngle) <= 10){
                    groundMat.at<int8_t>(i,j) = 1;
                    groundMat.at<int8_t>(i+1,j) = 1;
                }
            }
        }
        // extract ground cloud (groundMat == 1)
        // mark entry that doesn't need to label (ground and invalid point) for segmentation
        // note that ground remove is from 0~V_SCAN-1, need rangeMat for mark label matrix for the 16th scan
        for (size_t i = 0; i < V_SCAN; ++i){
            for (size_t j = 0; j < H_SCAN; ++j){
                if (groundMat.at<int8_t>(i,j) == 1 || rangeMat.at<float>(i,j) == FLT_MAX){
                    labelMat.at<int>(i,j) = -1;
                }
            }
        }
        if (pubGroundCloud.getNumSubscribers() != 0){
            for (size_t i = 0; i <= groundScanInd; ++i){
                for (size_t j = 0; j < H_SCAN; ++j){
                    if (groundMat.at<int8_t>(i,j) == 1)
                        groundCloud->push_back(fullCloud->points[j + i*H_SCAN]);
                }
            }
        }
    }

    void cloudSegmentation(){
        // segmentation process
        for (size_t i = 0; i < V_SCAN; ++i)
            for (size_t j = 0; j < H_SCAN; ++j)
                if (labelMat.at<int>(i,j) == 0)
                    labelComponents(i, j);

        int sizeOfSegCloud = 0;
        // extract segmented cloud for lidar odometry
        for (size_t i = 0; i < V_SCAN; ++i) {

            segMsg.startRingIndex[i] = sizeOfSegCloud-1 + 5;

            for (size_t j = 0; j < H_SCAN; ++j) {
                if (labelMat.at<int>(i,j) > 0 || groundMat.at<int8_t>(i,j) == 1){
                    // outliers that will not be used for optimization (always continue)
                    if (labelMat.at<int>(i,j) == 999999){
                        if (i > groundScanInd && j % 5 == 0){
                            outlierCloud->push_back(fullCloud->points[j + i*H_SCAN]);
                            continue;
                        }else{
                            continue;
                        }
                    }
                    // majority of ground points are skipped
                    if (groundMat.at<int8_t>(i,j) == 1){
                        if (j%5!=0 && j>5 && j<H_SCAN-5)
                            continue;
                    }
                    // mark ground points so they will not be considered as edge features later
                    segMsg.segmentedCloudGroundFlag[sizeOfSegCloud] = (groundMat.at<int8_t>(i,j) == 1);
                    // mark the points' column index for marking occlusion later
                    segMsg.segmentedCloudColInd[sizeOfSegCloud] = j;
                    // save range info
                    segMsg.segmentedCloudRange[sizeOfSegCloud]  = rangeMat.at<float>(i,j);
                    // save seg cloud
                    segmentedCloud->push_back(fullCloud->points[j + i*H_SCAN]);
                    // size of seg cloud
                    ++sizeOfSegCloud;
                }
            }

            segMsg.endRingIndex[i] = sizeOfSegCloud-1 - 5;
        }
        
        // extract segmented cloud for visualization
        if (pubSegmentedCloudPure.getNumSubscribers() != 0){
            for (size_t i = 0; i < V_SCAN; ++i){
                for (size_t j = 0; j < H_SCAN; ++j){
                    if (labelMat.at<int>(i,j) > 0 && labelMat.at<int>(i,j) != 999999){
                        segmentedCloudPure->push_back(fullCloud->points[j + i*H_SCAN]);
                        segmentedCloudPure->points.back().intensity = labelMat.at<int>(i,j);
                    }
                }
            }
        }
    }

    void labelComponents(int row, int col){
        // use std::queue std::vector std::deque will slow the program down greatly
        float d1, d2, alpha, angle;
        int fromIndX, fromIndY, thisIndX, thisIndY; 
        bool lineCountFlag[V_SCAN] = {false};

        queueIndX[0] = row;
        queueIndY[0] = col;
        int queueSize = 1;
        int queueStartInd = 0;
        int queueEndInd = 1;

        allPushedIndX[0] = row;
        allPushedIndY[0] = col;
        int allPushedIndSize = 1;
        
        while(queueSize > 0){
            // Pop point
            fromIndX = queueIndX[queueStartInd];
            fromIndY = queueIndY[queueStartInd];
            --queueSize;
            ++queueStartInd;
            // Mark popped point
            labelMat.at<int>(fromIndX, fromIndY) = labelCount;
            // Loop through all the neighboring grids of popped grid
            for (auto iter = neighborIterator.begin(); iter != neighborIterator.end(); ++iter){
                // new index
                thisIndX = fromIndX + (*iter).first;
                thisIndY = fromIndY + (*iter).second;
                // index should be within the boundary
                if (thisIndX < 0 || thisIndX >= V_SCAN)
                    continue;
                // at range image margin (left or right side)
                if (thisIndY < 0)
                    thisIndY = H_SCAN - 1;
                if (thisIndY >= H_SCAN)
                    thisIndY = 0;
                // prevent infinite loop (caused by put already examined point back)
                if (labelMat.at<int>(thisIndX, thisIndY) != 0)
                    continue;

                d1 = std::max(rangeMat.at<float>(fromIndX, fromIndY), 
                              rangeMat.at<float>(thisIndX, thisIndY));
                d2 = std::min(rangeMat.at<float>(fromIndX, fromIndY), 
                              rangeMat.at<float>(thisIndX, thisIndY));

                if ((*iter).first == 0)
                    alpha = segmentAlphaX;
                else
                    alpha = segmentAlphaY;

                angle = atan2(d2*sin(alpha), (d1 -d2*cos(alpha)));

                if (angle > segmentTheta){

                    queueIndX[queueEndInd] = thisIndX;
                    queueIndY[queueEndInd] = thisIndY;
                    ++queueSize;
                    ++queueEndInd;

                    labelMat.at<int>(thisIndX, thisIndY) = labelCount;
                    lineCountFlag[thisIndX] = true;

                    allPushedIndX[allPushedIndSize] = thisIndX;
                    allPushedIndY[allPushedIndSize] = thisIndY;
                    ++allPushedIndSize;
                }
            }
        }

        // check if this segment is valid
        bool feasibleSegment = false;
        if (allPushedIndSize >= 30)
            feasibleSegment = true;
        else if (allPushedIndSize >= segmentValidPointNum){
            int lineCount = 0;
            for (size_t i = 0; i < V_SCAN; ++i)
                if (lineCountFlag[i] == true)
                    ++lineCount;
            if (lineCount >= segmentValidLineNum)
                feasibleSegment = true;            
        }
        // segment is valid, mark these points
        if (feasibleSegment == true){
            ++labelCount;
        }else{ // segment is invalid, mark these points
            for (size_t i = 0; i < allPushedIndSize; ++i){
                labelMat.at<int>(allPushedIndX[i], allPushedIndY[i]) = 999999;
            }
        }
    }
   
    void publishCloud(){
        // 1. Publish Seg Cloud Info - Gilmar featuresAssociated subscriber
        segMsg.header = cloudHeader;
        pubSegmentedCloudInfo.publish(segMsg);
        // 2. Publish clouds
        sensor_msgs::PointCloud2 laserCloudTemp;
        pcl::toROSMsg(*outlierCloud, laserCloudTemp); // Gilmar featuresAssociated subscriber
        // cout << laserCloudTemp; //imprimindo no terminal
        laserCloudTemp.header.stamp = cloudHeader.stamp;
        laserCloudTemp.header.frame_id = "base_link";
        pubOutlierCloud.publish(laserCloudTemp);
        // segmented cloud with ground
        pcl::toROSMsg(*segmentedCloud, laserCloudTemp); // Gilmar featuresAssociated subscriber
        laserCloudTemp.header.stamp = cloudHeader.stamp;
        laserCloudTemp.header.frame_id = "base_link";
        pubSegmentedCloud.publish(laserCloudTemp);

        // End time processing pointcloud - Gilmar
        timeEnd = ros::Time::now().toSec();
        double timeDiff = timeEnd - timeInit;
        std_msgs::Time msgTimeDiff;
        msgTimeDiff.data = ros::Time().fromSec(timeDiff);
        if (pubTime.getNumSubscribers() != 0){
            pubTime.publish(msgTimeDiff);
        }

        // projected full cloud
        if (pubFullCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*fullCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubFullCloud.publish(laserCloudTemp);
        }
        // original dense ground cloud
        if (pubGroundCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*groundCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubGroundCloud.publish(laserCloudTemp);
        }
        // segmented cloud without ground
        if (pubSegmentedCloudPure.getNumSubscribers() != 0){
            pcl::toROSMsg(*segmentedCloudPure, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubSegmentedCloudPure.publish(laserCloudTemp);
        }
        // projected full cloud info
        if (pubFullInfoCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*fullInfoCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubFullInfoCloud.publish(laserCloudTemp);
        }
    }
};


int main(int argc, char** argv){

    ros::init(argc, argv, "ekf_loam");
    
    //Parameters init:    
    ros::NodeHandle nh_;    
    try
    {
	    nh_.param("/ekf_loam/lidar_frame", lidar_frame, std::string("os1_sensor"));
        nh_.param("/ekf_loam/segmentThetaValue", segmentThetanum, float(10.0));
        nh_.param("/ekf_loam/sensorMountAngle", sensorMountAngle, float(0.0));
        nh_.param("/ekf_loam/segmentValidPointNum", segmentValidPointNum, int(5));
        nh_.param("/ekf_loam/segmentValidLineNum", segmentValidLineNum, int(3));
        nh_.param("/ekf_loam/enableFlatGround", enableFlatGround, bool(false));

        nh_.param("/sensor_parameters/H_SCAN", H_SCAN, int(1024));
        nh_.param("/sensor_parameters/V_SCAN", V_SCAN, int(16));
        nh_.param("/sensor_parameters/groundScanInd", groundScanInd, int(7));
        nh_.param("/sensor_parameters/opening_vertical_angle", opening_vertical_angle, float(33.2));
        nh_.param("/sensor_parameters/opening_horizon_angle", opening_horizon_angle, float(360.0));
        nh_.param("/sensor_parameters/start_angle_v", start_angle_v, float(16.7));
        nh_.param("/sensor_parameters/start_angle_h", start_angle_h, float(-180));
    }
    catch (int e)
    {
        ROS_INFO("\033[1;31m---->\033[0m Exception occurred when importing parameters in Pre-Treatment Node. Exception Nr. %d", e);
    }

    // Angle resolution
    ang_res_h = opening_horizon_angle/float(H_SCAN);
    ang_res_v = opening_vertical_angle/float(V_SCAN-1);

    // Segmentation angle
    segmentTheta = segmentThetanum/180.0*M_PI; 
    segmentAlphaX = ang_res_h/180.0*M_PI; 
    segmentAlphaY = ang_res_v/180.0*M_PI; 
    
    PreTreatment PT;

    ROS_INFO("\033[1;32m---->\033[0m Pre-Treatment Started.");

    ros::spin();
    return 0;
}
