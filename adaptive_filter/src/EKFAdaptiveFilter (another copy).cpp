//=====================================================EKF-LOAM=========================================================
//Project: EspeleoRobô2
//Institution: Universidade Federal de Minas Gerais (UFMG) and Instituto Tecnológico Vale (ITV)
//Description: This file is responsible for merging the wheel odometry with the IMU data and the LiDAR odometry.
//Modification: 
//             Date: November 27, 2021
//             member: Gilmar Pereira da Cruz Júnior e Adriano Resende
//             e-mail: gilmarpcruzjunior@gmail.com, adrianomcr18@gmail.com
//=======================================================================================================================

#include "settings_adaptive_filter.h"

using namespace Eigen;
using namespace std;

//-----------------------------
// Global variables
//-----------------------------
bool enableFilter;
bool enableImu;
bool enableWheel;
bool enableLidar;

std::string filterFreq;

std::mutex mtx; 

std::vector<double> g1wx1, g1wx2, g1xmin, g1xmax;
std::vector<double> g1wy1, g1wy2, g1ymin, g1ymax;
std::vector<double> g1wz1, g1wz2, g1zmin, g1zmax;
std::vector<double> g1wr1, g1wr2, g1rmin, g1rmax;
std::vector<double> g1wp1, g1wp2, g1pmin, g1pmax;
std::vector<double> g1wya1, g1wya2, g1yamin, g1yamax;

std::vector<double> g2wx1, g2wx2, g2xmin, g2xmax;
std::vector<double> g2wy1, g2wy2, g2ymin, g2ymax;
std::vector<double> g2wz1, g2wz2, g2zmin, g2zmax;
std::vector<double> g2wr1, g2wr2, g2rmin, g2rmax;
std::vector<double> g2wp1, g2wp2, g2pmin, g2pmax;
std::vector<double> g2wya1, g2wya2, g2yamin, g2yamax;

std::vector<double> g3wx1, g3wx2, g3xmin, g3xmax;
std::vector<double> g3wy1, g3wy2, g3ymin, g3ymax;
std::vector<double> g3wz1, g3wz2, g3zmin, g3zmax;
std::vector<double> g3wr1, g3wr2, g3rmin, g3rmax;
std::vector<double> g3wp1, g3wp2, g3pmin, g3pmax;
std::vector<double> g3wya1, g3wya2, g3yamin, g3yamax;

std::vector<double> g4wx1, g4wx2, g4xmin, g4xmax;
std::vector<double> g4wy1, g4wy2, g4ymin, g4ymax;
std::vector<double> g4wz1, g4wz2, g4zmin, g4zmax;
std::vector<double> g4wr1, g4wr2, g4rmin, g4rmax;
std::vector<double> g4wp1, g4wp2, g4pmin, g4pmax;
std::vector<double> g4wya1, g4wya2, g4yamin, g4yamax;

std::vector<double> g5wx1, g5wx2, g5xmin, g5xmax;
std::vector<double> g5wy1, g5wy2, g5ymin, g5ymax;
std::vector<double> g5wz1, g5wz2, g5zmin, g5zmax;
std::vector<double> g5wr1, g5wr2, g5rmin, g5rmax;
std::vector<double> g5wp1, g5wp2, g5pmin, g5pmax;
std::vector<double> g5wya1, g5wya2, g5yamin, g5yamax;

std::vector<double> g6wx1, g6wx2, g6xmin, g6xmax;
std::vector<double> g6wy1, g6wy2, g6ymin, g6ymax;
std::vector<double> g6wz1, g6wz2, g6zmin, g6zmax;
std::vector<double> g6wr1, g6wr2, g6rmin, g6rmax;
std::vector<double> g6wp1, g6wp2, g6pmin, g6pmax;
std::vector<double> g6wya1, g6wya2, g6yamin, g6yamax;

std::vector<double> g7wx1, g7wx2, g7xmin, g7xmax;
std::vector<double> g7wy1, g7wy2, g7ymin, g7ymax;
std::vector<double> g7wz1, g7wz2, g7zmin, g7zmax;
std::vector<double> g7wr1, g7wr2, g7rmin, g7rmax;
std::vector<double> g7wp1, g7wp2, g7pmin, g7pmax;
std::vector<double> g7wya1, g7wya2, g7yamin, g7yamax;

std::vector<double> g8wx1, g8wx2, g8xmin, g8xmax;
std::vector<double> g8wy1, g8wy2, g8ymin, g8ymax;
std::vector<double> g8wz1, g8wz2, g8zmin, g8zmax;
std::vector<double> g8wr1, g8wr2, g8rmin, g8rmax;
std::vector<double> g8wp1, g8wp2, g8pmin, g8pmax;
std::vector<double> g8wya1, g8wya2, g8yamin, g8yamax;

std::vector<double> g9wx1, g9wx2, g9xmin, g9xmax;
std::vector<double> g9wy1, g9wy2, g9ymin, g9ymax;
std::vector<double> g9wz1, g9wz2, g9zmin, g9zmax;
std::vector<double> g9wr1, g9wr2, g9rmin, g9rmax;
std::vector<double> g9wp1, g9wp2, g9pmin, g9pmax;
std::vector<double> g9wya1, g9wya2, g9yamin, g9yamax;

std::vector<double> g10wx1, g10wx2, g10xmin, g10xmax;
std::vector<double> g10wy1, g10wy2, g10ymin, g10ymax;
std::vector<double> g10wz1, g10wz2, g10zmin, g10zmax;
std::vector<double> g10wp1, g10wp2, g10pmin, g10pmax;

//-----------------------------
// LiDAR Odometry class
//-----------------------------
class AdaptiveFilter{

private:
    // ros node
    ros::NodeHandle nh;

    // Subscriber
    ros::Subscriber subImu;
    ros::Subscriber subWheelOdometry;
    ros::Subscriber subLaserOdometry;

    // Publisher
    ros::Publisher pubFilteredOdometry;
    ros::Publisher pubIndLiDARMeasurement;

    // header
    std_msgs::Header headerI;
    std_msgs::Header headerW;
    std_msgs::Header headerL;

    // TF 
    tf::StampedTransform filteredOdometryTrans;
    tf::TransformBroadcaster tfBroadcasterfiltered;

    // filtered odom
    nav_msgs::Odometry filteredOdometry;
    nav_msgs::Odometry indLiDAROdometry;

    // Measure
    Eigen::VectorXd imuMeasure, wheelMeasure, lidarMeasure, lidarMeasureL;

    // Measure Covariance
    Eigen::MatrixXd E_imu, E_wheel, E_lidar, E_lidarL, E_pred;

    // States and covariances
    Eigen::VectorXd X, V;
    Eigen::MatrixXd P, PV;

    // pose and velocities
    Eigen::VectorXd pose, velocities;

    // Times
    double imuTimeLast;
    double wheelTimeLast;
    double lidarTimeLast;

    double imuTimeCurrent;
    double wheelTimeCurrent;
    double lidarTimeCurrent;

    double imu_dt;
    double wheel_dt;
    double lidar_dt;

    // imu varibles
    struct bias bias_linear_acceleration;
    struct bias bias_angular_velocity;

    // number of state or measure vectors
    int N_STATES = 12;
    int N_IMU = 9; 
    int N_WHEEL = 2; 
    int N_LIDAR = 6;
    
    // boolean
    bool imuActivated;
    bool wheelActivated;
    bool lidarActivated;
    bool imuNew;
    bool wheelNew;
    bool lidarNew;
    bool velComp;

    // adaptive covariance
    double nCorner, nSurf; 
    double Gx, Gy, Gz, Gphi, Gtheta, Gpsi;
    float l_min;

    // nfn
    double wx[10][2][10]; //, wx2[2][10], wx3[2][10], wx4[2][10], wx5[2][10], wx6[2][10], wx7[2][10], wx8[2][10], wx9[2][10], wx10[2][10];
    double wy[10][2][10]; //, wy2[2][10], wy3[2][10], wy4[2][10], wy5[2][10], wy6[2][10], wy7[2][10], wy8[2][10], wy9[2][10], wy10[2][10];
    double wz[10][2][10]; //, wz2[2][10], wz3[2][10], wz4[2][10], wz5[2][10], wz6[2][10], wz7[2][10], wz8[2][10], wz9[2][10], wz10[2][10];
    double wr[10][2][10]; //, wr2[2][10], wr3[2][10], wr4[2][10], wr5[2][10], wr6[2][10], wr7[2][10], wr8[2][10], wr9[2][10], wr10[2][10];
    double wp[10][2][10]; //, wp2[2][10], wp3[2][10], wp4[2][10], wp5[2][10], wp6[2][10], wp7[2][10], wp8[2][10], wp9[2][10], wp10[2][10];
    double wya[10][2][10]; //, wya2[2][10], wya3[2][10], wya4[2][10], wya5[2][10], wya6[2][10], wya7[2][10], wya8[2][10], wya9[2][10], wya10[2][10];

    double xming[10][2], yming[10][2], zming[10][2], rming[10][2], pming[10][2], yaming[10][2]; //, xmin2[2], xmin3[2], xmin4[2], xmin5[2], xmin6[2], xmin7[2], xmin8[2], xmin9[2], xmin10[2];
    double xmaxg[10][2], ymaxg[10][2], zmaxg[10][2], rmaxg[10][2], pmaxg[10][2], yamaxg[10][2]; //, xmax2[2], xmax3[2], xmax4[2], xmax5[2], xmax6[2], xmax7[2], xmax8[2], xmax9[2], xmax10[2];

    int nE, mFP;
    double Cx[10][2], Cy[10][2], Cz[10][2], Cr[10][2], Cp[10][2], Cya[10][2];
    double Sx[2], Sy[2], Sz[2], Sr[2], Sp[2], Sya[2];  

public:
    AdaptiveFilter():
        nh("~")
    {
        // Subscriber
        subImu = nh.subscribe<sensor_msgs::Imu>("/imu/data", 50, &AdaptiveFilter::imuHandler, this);
        subWheelOdometry = nh.subscribe<nav_msgs::Odometry>("/odom", 5, &AdaptiveFilter::wheelOdometryHandler, this);
        subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/ekf_loam/laser_odom_to_initOut", 5, &AdaptiveFilter::laserOdometryHandler, this);
        
        // Publisher
        pubFilteredOdometry = nh.advertise<nav_msgs::Odometry> ("/ekf_loam/filter_odom_to_init", 5);
        pubIndLiDARMeasurement = nh.advertise<nav_msgs::Odometry> ("/indirect_lidar_measurement", 5); // only publish indirect lidar measuremet 

        // Initialization
        allocateMemory();
        std::cout << "antes do initialization\n";
        initialization();
        std::cout << "depois do ginitialization\n";
    }

    //------------------
    // Auxliar functions
    //------------------
    void allocateMemory(){
        imuMeasure.resize(N_IMU);
        wheelMeasure.resize(N_WHEEL);
        lidarMeasure.resize(N_LIDAR);
        lidarMeasureL.resize(N_LIDAR);

        E_imu.resize(N_IMU,N_IMU);
        E_wheel.resize(N_WHEEL,N_WHEEL);
        E_lidar.resize(N_LIDAR,N_LIDAR);
        E_lidarL.resize(N_LIDAR,N_LIDAR);
        E_pred.resize(N_STATES,N_STATES);

        X.resize(N_STATES);
        P.resize(N_STATES,N_STATES);

        V.resize(N_STATES);
        PV.resize(N_STATES,N_STATES);
    }

    void initialization(){
        // times
        imuTimeLast = 0;
        lidarTimeLast = 0;
        wheelTimeLast = 0;

        imuTimeCurrent = 0;
        lidarTimeCurrent = 0;
        wheelTimeCurrent = 0;

        // auxliar 
        bias_linear_acceleration.x = 0.0001;
        bias_linear_acceleration.y = 0.0001;
        bias_linear_acceleration.z = 0.0001;

        bias_angular_velocity.x = 0.00000001;
        bias_angular_velocity.y = 0.00000001;
        bias_angular_velocity.z = 0.00000001;

        wheel_dt = 0.05;
        lidar_dt = 0.1;

        // boolean
        imuActivated = false;
        lidarActivated = false;
        wheelActivated = false;

        imuNew = false;
        wheelNew = false;
        lidarNew = false;

        velComp = false;

        // matrices and vectors
        imuMeasure = Eigen::VectorXd::Zero(N_IMU);
        wheelMeasure = Eigen::VectorXd::Zero(N_WHEEL);
        lidarMeasure = Eigen::VectorXd::Zero(N_LIDAR);
        lidarMeasureL = Eigen::VectorXd::Zero(N_LIDAR);
        
        E_imu = Eigen::MatrixXd::Zero(N_IMU,N_IMU);
        E_lidar = Eigen::MatrixXd::Zero(N_LIDAR,N_LIDAR);
        E_lidarL = Eigen::MatrixXd::Zero(N_LIDAR,N_LIDAR);
        E_wheel = Eigen::MatrixXd::Zero(N_WHEEL,N_WHEEL);
        E_pred = Eigen::MatrixXd::Zero(N_STATES,N_STATES);

        // state initial
        X = Eigen::VectorXd::Zero(N_STATES);
        P = Eigen::MatrixXd::Zero(N_STATES,N_STATES);
        V = Eigen::VectorXd::Zero(N_STATES);

        // covariance initial
        P(0,0) = 0.1;   // x
        P(1,1) = 0.1;   // y
        P(2,2) = 0.1;   // z
        P(3,3) = 0.1;   // roll
        P(4,4) = 0.1;   // pitch
        P(5,5) = 0.1;   // yaw
        P(6,6) = 0.1;   // vx
        P(7,7) = 0.1;   // vy
        P(8,8) = 0.1;   // vz
        P(9,9) = 0.1;   // wx
        P(10,10) = 0.1;   // wy
        P(11,11) = 0.1;   // wz

        // Fixed prediction covariance
        E_pred.block(6,6,6,6) = 0.01*P.block(6,6,6,6);

        // adptive covariance constants
        nCorner = 500.0; // 7000
        nSurf = 5000;    // 5400
        
        Gz = 0.0048;    // x [m]
        Gx = 0.0022;    // y [m]
        Gy = 0.0016;    // z [m]
        Gpsi = 0.0044;  // phi [rad]
        Gphi = 0.0052;  // theta [rad]
        Gtheta = 0.005; // psi [rad]

        l_min = 0.005;

        // nfn
        mFP = 10;
        nE = 2;
        std::cout << "antes do for dos ws\n";
        for (int j = 0; j < mFP; ++j){
            wx[0][0][j] = g1wx1[j];
            wx[0][1][j] = g1wx2[j];
            wy[0][0][j] = g1wy1[j];
            wy[0][1][j] = g1wy2[j];
            wz[0][0][j] = g1wz1[j];
            wz[0][1][j] = g1wz2[j];
            wr[0][0][j] = g1wr1[j];
            wr[0][1][j] = g1wr2[j];
            wp[0][0][j] = g1wp1[j];
            wp[0][1][j] = g1wp2[j];
            wya[0][0][j] = g1wya1[j];
            wya[0][1][j] = g1wya2[j];
            
            wx[1][0][j] = g1wx1[j];
            wx[1][1][j] = g1wx2[j];
            wy[1][0][j] = g1wy1[j];
            wy[1][1][j] = g1wy2[j];
            wz[1][0][j] = g1wz1[j];
            wz[1][1][j] = g1wz2[j];
            wr[1][0][j] = g1wr1[j];
            wr[1][1][j] = g1wr2[j];
            wp[1][0][j] = g1wp1[j];
            wp[1][1][j] = g1wp2[j];
            wya[1][0][j] = g1wya1[j];
            wya[1][1][j] = g1wya2[j];

        
            wx[2][0][j] = g1wx1[j];
            wx[2][1][j] = g1wx2[j];
            wy[2][0][j] = g1wy1[j];
            wy[2][1][j] = g1wy2[j];
            wz[2][0][j] = g1wz1[j];
            wz[2][1][j] = g1wz2[j];
            wr[2][0][j] = g1wr1[j];
            wr[2][1][j] = g1wr2[j];
            wp[2][0][j] = g1wp1[j];
            wp[2][1][j] = g1wp2[j];
            wya[2][0][j] = g1wya1[j];
            wya[2][1][j] = g1wya2[j];

            wx[3][0][j] = g1wx1[j];
            wx[3][1][j] = g1wx2[j];
            wy[3][0][j] = g1wy1[j];
            wy[3][1][j] = g1wy2[j];
            wz[3][0][j] = g1wz1[j];
            wz[3][1][j] = g1wz2[j];
            wr[3][0][j] = g1wr1[j];
            wr[3][1][j] = g1wr2[j];
            wp[3][0][j] = g1wp1[j];
            wp[3][1][j] = g1wp2[j];
            wya[3][0][j] = g1wya1[j];
            wya[3][1][j] = g1wya2[j];

            
            wx[4][0][j] = g1wx1[j];
            wx[4][1][j] = g1wx2[j];
            wy[4][0][j] = g1wy1[j];
            wy[4][1][j] = g1wy2[j];
            wz[4][0][j] = g1wz1[j];
            wz[4][1][j] = g1wz2[j];
            wr[4][0][j] = g1wr1[j];
            wr[4][1][j] = g1wr2[j];
            wp[4][0][j] = g1wp1[j];
            wp[4][1][j] = g1wp2[j];
            wya[4][0][j] = g1wya1[j];
            wya[4][1][j] = g1wya2[j];  

            wx[5][0][j] = g1wx1[j];
            wx[5][1][j] = g1wx2[j];
            wy[5][0][j] = g1wy1[j];
            wy[5][1][j] = g1wy2[j];
            wz[5][0][j] = g1wz1[j];
            wz[5][1][j] = g1wz2[j];
            wr[5][0][j] = g1wr1[j];
            wr[5][1][j] = g1wr2[j];
            wp[5][0][j] = g1wp1[j];
            wp[5][1][j] = g1wp2[j];
            wya[5][0][j] = g1wya1[j];
            wya[5][1][j] = g1wya2[j];  

            wx[6][0][j] = g1wx1[j];
            wx[6][1][j] = g1wx2[j];
            wy[6][0][j] = g1wy1[j];
            wy[6][1][j] = g1wy2[j];
            wz[6][0][j] = g1wz1[j];
            wz[6][1][j] = g1wz2[j];
            wr[6][0][j] = g1wr1[j];
            wr[6][1][j] = g1wr2[j];
            wp[6][0][j] = g1wp1[j];
            wp[6][1][j] = g1wp2[j];
            wya[6][0][j] = g1wya1[j];
            wya[6][1][j] = g1wya2[j];

            wx[7][0][j] = g1wx1[j];
            wx[7][1][j] = g1wx2[j];
            wy[7][0][j] = g1wy1[j];
            wy[7][1][j] = g1wy2[j];
            wz[7][0][j] = g1wz1[j];
            wz[7][1][j] = g1wz2[j];
            wr[7][0][j] = g1wr1[j];
            wr[7][1][j] = g1wr2[j];
            wp[7][0][j] = g1wp1[j];
            wp[7][1][j] = g1wp2[j];
            wya[7][0][j] = g1wya1[j];
            wya[7][1][j] = g1wya2[j];

            wx[8][0][j] = g1wx1[j];
            wx[8][1][j] = g1wx2[j];
            wy[8][0][j] = g1wy1[j];
            wy[8][1][j] = g1wy2[j];
            wz[8][0][j] = g1wz1[j];
            wz[8][1][j] = g1wz2[j];
            wr[8][0][j] = g1wr1[j];
            wr[8][1][j] = g1wr2[j];
            wp[8][0][j] = g1wp1[j];
            wp[8][1][j] = g1wp2[j];
            wya[8][0][j] = g1wya1[j];
            wya[8][1][j] = g1wya2[j];
            

            wx[9][0][j] = g1wx1[j];
            wx[9][1][j] = g1wx2[j];
            wy[9][0][j] = g1wy1[j];
            wy[9][1][j] = g1wy2[j];
            wz[9][0][j] = g1wz1[j];
            wz[9][1][j] = g1wz2[j];
            wp[9][0][j] = g1wp1[j];
            wp[9][1][j] = g1wp2[j];

        }

        // min e max
        xming[0][0] = g1xmin[0];
        xmaxg[0][1] = g1xmax[j]; 
        yming[0][0] = g1ymin[j];
        ymaxg[0][1] = g1ymax[j];
        zming[0][0] = g1zmin[j];
        zmaxg[0][1] = g1zmax[j];
        rming[0][0] = g1rmin[j];
        rmaxg[0][1] = g1rmax[j];
        pming[0][0] = g1pmin[j];
        pmaxg[0][1] = g1pmax[j];
        yaming[0][0] = g1yamin[j];
        yamaxg[0][1] = g1yamax[j];  
        xming[1][0] = g1xmin[j];
        xmaxg[1][1] = g1xmax[j]; 
        yming[1][0] = g1ymin[j];
        ymaxg[1][1] = g1ymax[j];
        zming[1][0] = g1zmin[j];
        zmaxg[1][1] = g1zmax[j];
        rming[1][0] = g1rmin[j];
        rmaxg[1][1] = g1rmax[j];
        pming[1][0] = g1pmin[j];
        pmaxg[1][1] = g1pmax[j];
        yaming[1][0] = g1yamin[j];
        yamaxg[1][1] = g1yamax[j];
        xming[2][0] = g1xmin[j];
        xmaxg[2][1] = g1xmax[j]; 
        yming[2][0] = g1ymin[j];
        ymaxg[2][1] = g1ymax[j];
        zming[2][0] = g1zmin[j];
        zmaxg[2][1] = g1zmax[j];
        rming[2][0] = g1rmin[j];
        rmaxg[2][1] = g1rmax[j];
        pming[2][0] = g1pmin[j];
        pmaxg[2][1] = g1pmax[j];
        yaming[2][0] = g1yamin[j];
        yamaxg[2][1] = g1yamax[j];
        xming[3][0] = g1xmin[j];
        xmaxg[3][1] = g1xmax[j]; 
        yming[3][0] = g1ymin[j];
        ymaxg[3][1] = g1ymax[j];
        zming[3][0] = g1zmin[j];
        zmaxg[3][1] = g1zmax[j];
        rming[3][0] = g1rmin[j];
        rmaxg[3][1] = g1rmax[j];
        pming[3][0] = g1pmin[j];
        pmaxg[3][1] = g1pmax[j];
        yaming[3][0] = g1yamin[j];
        yamaxg[3][1] = g1yamax[j]; 
        xming[4][0] = g1xmin[j];
        xmaxg[4][1] = g1xmax[j]; 
        yming[4][0] = g1ymin[j];
        ymaxg[4][1] = g1ymax[j];
        zming[4][0] = g1zmin[j];
        zmaxg[4][1] = g1zmax[j];
        rming[4][0] = g1rmin[j];
        rmaxg[4][1] = g1rmax[j];
        pming[4][0] = g1pmin[j];
        pmaxg[4][1] = g1pmax[j];
        yaming[4][0] = g1yamin[j];
        yamaxg[4][1] = g1yamax[j];
        xming[5][0] = g1xmin[j];
        xmaxg[5][1] = g1xmax[j]; 
        yming[5][0] = g1ymin[j];
        ymaxg[5][1] = g1ymax[j];
        zming[5][0] = g1zmin[j];
        zmaxg[5][1] = g1zmax[j];
        rming[5][0] = g1rmin[j];
        rmaxg[5][1] = g1rmax[j];
        pming[5][0] = g1pmin[j];
        pmaxg[5][1] = g1pmax[j];
        yaming[5][0] = g1yamin[j];
        yamaxg[5][1] = g1yamax[j];
        xming[6][0] = g1xmin[j];
        xmaxg[6][1] = g1xmax[j]; 
        yming[6][0] = g1ymin[j];
        ymaxg[6][1] = g1ymax[j];
        zming[6][0] = g1zmin[j];
        zmaxg[6][1] = g1zmax[j];
        rming[6][0] = g1rmin[j];
        rmaxg[6][1] = g1rmax[j];
        pming[6][0] = g1pmin[j];
        pmaxg[6][1] = g1pmax[j];
        yaming[6][0] = g1yamin[j];
        yamaxg[6][1] = g1yamax[j];  
        xming[7][0] = g1xmin[j];
        xmaxg[7][1] = g1xmax[j]; 
        yming[7][0] = g1ymin[j];
        ymaxg[7][1] = g1ymax[j];
        zming[7][0] = g1zmin[j];
        zmaxg[7][1] = g1zmax[j];
        rming[7][0] = g1rmin[j];
        rmaxg[7][1] = g1rmax[j];
        pming[7][0] = g1pmin[j];
        pmaxg[7][1] = g1pmax[j];
        yaming[7][0] = g1yamin[j];
        yamaxg[7][1] = g1yamax[j];  
        xming[8][0] = g1xmin[j];
        xmaxg[8][1] = g1xmax[j]; 
        yming[8][0] = g1ymin[j];
        ymaxg[8][1] = g1ymax[j];
        zming[8][0] = g1zmin[j];
        zmaxg[8][1] = g1zmax[j];
        rming[8][0] = g1rmin[j];
        rmaxg[8][1] = g1rmax[j];
        pming[8][0] = g1pmin[j];
        pmaxg[8][1] = g1pmax[j];
        yaming[8][0] = g1yamin[j];
        yamaxg[8][1] = g1yamax[j]; 
        xming[9][0] = g1xmin[j];
        xmaxg[9][1] = g1xmax[j]; 
        yming[9][0] = g1ymin[j];
        ymaxg[9][1] = g1ymax[j];
        zming[9][0] = g1zmin[j];
        zmaxg[9][1] = g1zmax[j];
        pming[9][0] = g1pmin[j];
        pmaxg[9][1] = g1pmax[j];

        std::cout << "depois do for dos ws\n";
        // C and S
        Cx[0][0] = 69;
        Cx[0][1] = 516;
        Cx[1][0] = 320;	
        Cx[1][1] = 404;
        Cx[2][0] = 56;
        Cx[2][1] = 2348;
        Cx[3][0] = 156; 
        Cx[3][1] = 383;
        Cx[4][0] = 53;
        Cx[4][1] = 1168;
        Cx[5][0] = 42;
        Cx[5][1] = 335;
        Cx[6][0] = 161;
        Cx[6][1] = 2310;
        Cx[7][0] = 163;
        Cx[7][1] = 1168;
        Cx[8][0] = 318;
        Cx[8][1] = 1185;
        Cx[9][0] = 317;
        Cx[9][1] = 2332;

        Sx[0] = 40.1990205104552;
        Sx[1] = 284.044794002636;

        Cy[0][0] = 66; 
        Cy[0][1] = 515;
        Cy[1][0] = 320;	
        Cy[1][1] = 406;
        Cy[2][0] = 56;
        Cy[2][1] = 2346;
        Cy[3][0] = 157; 
        Cy[3][1] = 379;
        Cy[4][0] = 54;
        Cy[4][1] = 1182;
        Cy[5][0] = 40;
        Cy[5][1] = 335;
        Cy[6][0] = 161;
        Cy[6][1] = 2310;
        Cy[7][0] = 163;
        Cy[7][1] = 1168;
        Cy[8][0] = 318;
        Cy[8][1] = 1185;
        Cy[9][0] = 317;
        Cy[9][1] = 2332;

        Sy[0] = 40.1990205104552;
        Sy[1] = 284.044794002636;

        Cz[0][0] = 80;
        Cz[0][1] = 466;
        Cz[1][0] = 320;	
        Cz[1][1] = 403;
        Cz[2][0] = 56;
        Cz[2][1] = 2348;
        Cz[3][0] = 57; 
        Cz[3][1] = 1138;
        Cz[4][0] = 57;
        Cz[4][1] = 392;
        Cz[5][0] = 39;
        Cz[5][1] = 387;
        Cz[6][0] = 161;
        Cz[6][1] = 2310;
        Cz[7][0] = 163;
        Cz[7][1] = 1168;
        Cz[8][0] = 318;
        Cz[8][1] = 1185;
        Cz[9][0] = 317;
        Cz[9][1] = 2332;

        Sz[0] = 40.1990205104552;
        Sz[1] = 284.044794002636;
        
        Cr[0][0] = 65;
        Cr[0][1] = 459;
        Cr[1][0] = 321;	
        Cr[1][1] = 407;
        Cr[2][0] = 163;
        Cr[2][1] = 403;
        Cr[3][0] = 62; 
        Cr[3][1] = 2342;
        Cr[4][0] = 62;
        Cr[4][1] = 1172;
        Cr[5][0] = 161;
        Cr[5][1] = 2310;
        Cr[6][0] = 163;
        Cr[6][1] = 1168;
        Cr[7][0] = 318;
        Cr[7][1] = 1185;
        Cr[8][0] = 317;
        Cr[8][1] = 2332;
        Cr[9][0] = 0;
        Cr[9][1] = 0;

        Sr[0] = 40.3050865276332;
        Sr[1] = 284.150860019814; 

        Cp[0][0] = 82;
        Cp[0][1] = 462;
        Cp[1][0] = 321;	
        Cp[1][1] = 405;
        Cp[2][0] = 58;
        Cp[2][1] = 2346;
        Cp[3][0] = 60; 
        Cp[3][1] = 1142;
        Cp[4][0] = 167;
        Cp[4][1] = 399;
        Cp[5][0] = 39;
        Cp[5][1] = 453;
        Cp[6][0] = 161;
        Cp[6][1] = 2310;
        Cp[7][0] = 163;
        Cp[7][1] = 1168;
        Cp[8][0] = 318;
        Cp[8][1] = 1185;
        Cp[9][0] = 317;
        Cp[9][1] = 2332;

        Sp[0] = 40.3050865276332;
        Sp[1] = 284.150860019814; 

        Cya[0][0] = 69;
        Cya[0][1] = 459;
        Cya[1][0] = 321;	
        Cya[1][1] = 407;
        Cya[2][0] = 162;
        Cya[2][1] = 404;
        Cya[3][0] = 62; 
        Cya[3][1] = 2342;
        Cya[4][0] = 60;
        Cya[4][1] = 1176;
        Cya[5][0] = 161;
        Cya[5][1] = 2310;
        Cya[6][0] = 163;
        Cya[6][1] = 1168;
        Cya[7][0] = 318;
        Cya[7][1] = 1185;
        Cya[8][0] = 317;
        Cya[8][1] = 2332;
        Cya[9][0] = 0;
        Cya[9][1] = 0;

        Sya[0] = 40.3050865276332;
        Sya[1] = 284.150860019814; 
        
    }

    MatrixXd adaptive_covariance(double fCorner, double fSurf){
        Eigen::MatrixXd Q(6,6);
        double cov_x, cov_y, cov_z, cov_phi, cov_psi, cov_theta;
        
        // heuristic
        cov_x     = (nCorner - min(fCorner,nCorner))/nCorner + l_min;
        cov_y     = (nCorner - min(fCorner,nCorner))/nCorner + l_min;
        cov_psi = (nCorner - min(fCorner,nCorner))/nCorner + l_min;
        cov_z     = (nSurf - min(fSurf,nSurf))/nSurf + l_min;
        cov_phi   = (nSurf - min(fSurf,nSurf))/nSurf + l_min;
        cov_theta   = (nSurf - min(fSurf,nSurf))/nSurf + l_min;
        
        Q = MatrixXd::Zero(6,6);
        float b = 2000.0/1.0;
        float c = 2000.0/1.0;
        Q(0,0) = b*Gx*cov_x;
        Q(1,1) = c*Gy*cov_y;
        Q(2,2) = b*Gz*cov_z;
        Q(3,3) = c*Gphi*cov_phi;
        Q(4,4) = b*Gtheta*cov_theta;
        Q(5,5) = c*Gpsi*cov_psi;

        return Q;
    }

    // x, delta, bs, xmin, xmax, w[j], C[j], S
    double nfn(double x[], double delta[], double bs[][2], double xmin[], double xmax[], double w[2][10], double C[], double S[]){
        int ki[nE], kii[nE];
        double miki[nE], mikii[nE], yi[nE];
        double ys, aux_int;

        for (size_t k = 0; k < nE; k++){
            if (x[k] <= xmin[k]){
                ki[k] = 1;
                kii[k] = ki[k] + 1;
                miki[k] = 1;
                mikii[k] = 0;
            }else if (x[k] >= xmax[k]){
                ki[k] = mFP-1;
                kii[k] = ki[k] + 1;
                miki[k] = 0;
                mikii[k] = 1;
            }else{
                double aux = modf ((x[k]-xmin[k])/delta[k], &aux_int);
                ki[k] = int (aux_int + 1);
                kii[k] = ki[k] + 1;
                miki[k] = (-1/delta[k])*x[k] + bs[k][int(ki[k])];
                mikii[k] = 1 - miki[k];
            }
            
            yi[k] = miki[k]*w[k][int(ki[k])] + mikii[k]*w[k][int(kii[k])];
        }  
    
        // output
        ys = 0;
        for (size_t i = 0; i < nE; i++){
            ys = std::max(ys + yi[i],0.0) + 1e-7;
        }

        return ys;        
    }

    double sugeno(double x[], double w[10][2][10], double xmin[][2], double xmax[][2], double C[][2], double S[]){
        double nMF = 10;
        double max, min, ys;
        double delta[2], bs[10][2], zs[10];
        std::vector<double> wsg(10);

        // loop - sugeno
        for (int i = 0; i < nMF; i++){
            wsg[i] = 1.0;
        }

        for (int j = 0; j < nMF; j++){
            // w's
            for (int k = 0; k < nE; k++){
                double waux = exp(-0.5*pow((x[k]- C[j][k])/S[k],2)); // gaussmf  
                // product
                wsg[j] = wsg[j]*waux;
            }  

            // outputs
            delta[0] = (xmax[0]-xmin[0])/(nMF-1);
            delta[1] = (xmax[1]-xmin[1])/(nMF-1);
            for (int k = 0; k < mFP; k++){
                for (int l = 0; j < nE; l++){
                    bs[l][k] = xmin[k][l] + (k-1)*delta[l];
                }
            }
            zs[j] = nfn(x, delta, bs, xmin[j], xmax[j], w[j], C[j], S);
                
        }

        // weighted average 
        double yaux  = 0.0, den = 0.0;
        for (int i = 0; i < mFP; i++){
            yaux = yaux + wsg[i]*zs[i];   
            den = den  + wsg[i];
        }

        ys = yaux/den;

        return ys;         
    }

    MatrixXd nfn_adaptive_covariance(double fCorner, double fSurf){
        Eigen::MatrixXd Q(6,6);
        double cov_x, cov_y, cov_z, cov_phi, cov_psi, cov_theta;

        double x[2] = {fCorner, fSurf};

        cov_x = sugeno(x, wx, xming, xmaxg, Cx, Sx);
        cov_y = sugeno(x, wy, yming, ymaxg, Cy, Sy);
        cov_z = sugeno(x, wz, zming, zmaxg, Cz, Sz);
        cov_phi = sugeno(x, wr, rming, rmaxg, Cr, Sr);
        cov_theta = sugeno(x, wp, pming, pmaxg, Cp, Sp);
        cov_psi = sugeno(x, wya, yaming, yamaxg, Cya, Sya);

        Q = Eigen::MatrixXd::Identity(6,6);
        Q(0,0) = cov_x;
        Q(1,1) = cov_y;
        Q(2,2) = cov_z;
        Q(3,3) = cov_phi;
        Q(4,4) = cov_theta;
        Q(5,5) = cov_psi;

        return Q;
    }

    //-----------------
    // predict function
    //-----------------
    void prediction_stage(double dt){
        Eigen::MatrixXd F(N_STATES,N_STATES);

        // jacobian's computation
        F = jacobian_state(X, dt);

        // Priori state and covariance estimated
        X = f_prediction_model(X, dt);

        // Priori covariance
        P = F*P*F.transpose() + E_pred;
    }

    //-----------------
    // correction stage
    //-----------------
    void correction_wheel_stage(double dt){
        Eigen::VectorXd Y(N_WHEEL), hx(N_WHEEL);
        Eigen::MatrixXd H(N_WHEEL,N_STATES), K(N_STATES,N_WHEEL), E(N_WHEEL,N_WHEEL), S(N_WHEEL,N_WHEEL);

        // measure model of wheel odometry (only foward linear velocity)
        hx(0) = X(6);
        hx(1) = X(11);
        // measurement
        Y = wheelMeasure;

        // Jacobian of hx with respect to the states
        H = Eigen::MatrixXd::Zero(N_WHEEL,N_STATES);
        H(0,6) = 1; 
        H(1,11) = 1;

        // covariance matrices
        E << E_wheel;

        // Kalman's gain
        S = H*P*H.transpose() + E;
        K = P*H.transpose()*S.inverse();

        // correction
        X = X + K*(Y - hx);
        P = P - K*H*P;
    }

    void correction_imu_stage(double dt){
        Eigen::Matrix3d S, E;
        Eigen::Vector3d Y, hx;
        Eigen::MatrixXd H(3,N_STATES), K(N_STATES,3);

        // measure model
        hx = X.block(9,0,3,1);
        // wheel measurement
        Y = imuMeasure.block(3,0,3,1);

        // Jacobian of hx with respect to the states
        H = Eigen::MatrixXd::Zero(3,N_STATES);
        H.block(0,9,3,3) = Eigen::MatrixXd::Identity(3,3);

        // covariance matrices
        E = E_imu.block(3,3,3,3);

        // Kalman's gain
        S = H*P*H.transpose() + E;
        K = P*H.transpose()*S.inverse();

        // correction
        X = X + K*(Y - hx);
        P = P - K*H*P;
    }

    void correction_lidar_stage(double dt){
        Eigen::MatrixXd K(N_STATES,N_LIDAR), S(N_LIDAR,N_LIDAR), G(N_LIDAR,N_LIDAR), Gl(N_LIDAR,N_LIDAR), Q(N_LIDAR,N_LIDAR);
        Eigen::VectorXd Y(N_LIDAR), hx(N_LIDAR);
        Eigen::MatrixXd H(N_LIDAR,N_STATES); 

        // measure model
        hx = X.block(6,0,6,1);
        // wheel measurement
        Y = indirect_lidar_measurement(lidarMeasure, lidarMeasureL, dt);

        // Jacobian of hx with respect to the states
        H = Eigen::MatrixXd::Zero(N_LIDAR,N_STATES);
        H.block(0,6,6,6) = Eigen::MatrixXd::Identity(N_LIDAR,N_LIDAR);

        // Error propagation
        G = jacobian_lidar_measurement(lidarMeasure, lidarMeasureL, dt);
        Gl = jacobian_lidar_measurementL(lidarMeasure, lidarMeasureL, dt);

        Q =  G*E_lidar*G.transpose() + Gl*E_lidarL*Gl.transpose();
        // Q =  G*E_lidar*G.transpose();

        // data save 
        publish_indirect_lidar_measurement(Y, Q);        

        // Kalman's gain
        S = H*P*H.transpose() + Q;
        K = P*H.transpose()*S.inverse();

        // correction
        X = X + K*(Y - hx);
        P = P - K*H*P;

        // last measurement
        lidarMeasureL = lidarMeasure;
        E_lidarL = E_lidar;
    }
    
    //---------
    // Models
    //---------
    VectorXd f_prediction_model(VectorXd x, double dt){ 
        // state: {x, y, z, roll, pitch, yaw, vx, vy, vz, wx, wy, wz}
        //        {         (world)         }{        (body)        }
        Eigen::Matrix3d R, Rx, Ry, Rz, J;
        Eigen::VectorXd xp(N_STATES);
        Eigen::MatrixXd A(6,6);    

        // Rotation matrix
        Rx = Eigen::AngleAxisd(x(3), Eigen::Vector3d::UnitX());
        Ry = Eigen::AngleAxisd(x(4), Eigen::Vector3d::UnitY());
        Rz = Eigen::AngleAxisd(x(5), Eigen::Vector3d::UnitZ());
        R = Rz*Ry*Rx;
        
        // Jacobian matrix
        J << 1.0, sin(x(3))*tan(x(4)), cos(x(3))*tan(x(4)),
             0.0, cos(x(3)), -sin(x(3)),
             0.0, sin(x(3))/cos(x(4)), cos(x(3))/cos(x(4));
        
        // model
        A = Eigen::MatrixXd::Identity(6,6);
        A.block(0,0,3,3) = R;
        A.block(3,3,3,3) = J;

        xp.block(0,0,6,1) = x.block(0,0,6,1) + A*x.block(6,0,6,1)*dt;
        xp.block(6,0,6,1) = x.block(6,0,6,1);

        return xp;
    }

    VectorXd indirect_lidar_measurement(VectorXd u, VectorXd ul, double dt){
        Eigen::Matrix3d R, Rx, Ry, Rz, J;
        Eigen::VectorXd up(N_LIDAR), u_diff(N_LIDAR);  
        Eigen::MatrixXd A(N_LIDAR,N_LIDAR);  

        // Rotation matrix
        Rx = Eigen::AngleAxisd(ul(3), Eigen::Vector3d::UnitX());
        Ry = Eigen::AngleAxisd(ul(4), Eigen::Vector3d::UnitY());
        Rz = Eigen::AngleAxisd(ul(5), Eigen::Vector3d::UnitZ());
        R = Rz*Ry*Rx;
        
        // Jacobian matrix
        J << 1.0, sin(ul(3))*tan(ul(4)), cos(ul(3))*tan(ul(4)),
             0.0, cos(ul(3)), -sin(ul(3)),
             0.0, sin(ul(3))/cos(ul(4)), cos(ul(3))/cos(ul(4));
        
        // model
        u_diff.block(0,0,3,1) = (u.block(0,0,3,1) - ul.block(0,0,3,1));
        u_diff(3) = atan2(sin(u(3) - ul(3)),cos(u(3) - ul(3)));
        u_diff(4) = atan2(sin(u(4) - ul(4)),cos(u(4) - ul(4)));
        u_diff(5) = atan2(sin(u(5) - ul(5)),cos(u(5) - ul(5)));

        A = Eigen::MatrixXd::Zero(N_LIDAR,N_LIDAR);
        A.block(0,0,3,3) = R.transpose();
        A.block(3,3,3,3) = J.inverse();

        up = A*u_diff/dt;

        return up;

    }

    //----------
    // Jacobians
    //----------
    MatrixXd jacobian_state(VectorXd x, double dt){
        Eigen::MatrixXd J(N_STATES,N_STATES);
        Eigen::VectorXd f0(N_STATES), f1(N_STATES), x_plus(N_STATES);

        f0 = f_prediction_model(x, dt);

        double delta = 0.0001;
        for (size_t i = 0; i < N_STATES; i++){
            x_plus = x;
            x_plus(i) = x_plus(i) + delta;

            f1 = f_prediction_model(x_plus, dt);
           
            J.block(0,i,N_STATES,1) = (f1 - f0)/delta;       
            J(3,i) = sin(f1(3) - f0(3))/delta;
            J(4,i) = sin(f1(4) - f0(4))/delta;
            J(5,i) = sin(f1(5) - f0(5))/delta; 
        }

        return J;
    }

    MatrixXd jacobian_lidar_measurement(VectorXd u, VectorXd ul, double dt){ 
        Eigen::MatrixXd J(N_LIDAR,N_LIDAR);
        Eigen::VectorXd f0(N_LIDAR), f1(N_LIDAR), u_plus(N_LIDAR);

        f0 = indirect_lidar_measurement(u, ul, dt);

        double delta = 0.0000001;
        for (size_t i = 0; i < N_LIDAR; i++){
            u_plus = u;
            u_plus(i) = u_plus(i) + delta;

            f1 = indirect_lidar_measurement(u_plus, ul, dt);
           
            J.block(0,i,N_LIDAR,1) = (f1 - f0)/delta;       
            J(3,i) = sin(f1(3) - f0(3))/delta;
            J(4,i) = sin(f1(4) - f0(4))/delta;
            J(5,i) = sin(f1(5) - f0(5))/delta; 
        }

        return J;
    }

    MatrixXd jacobian_lidar_measurementL(VectorXd u, VectorXd ul, double dt){ 
        Eigen::MatrixXd J(N_LIDAR,N_LIDAR);
        Eigen::VectorXd f0(N_LIDAR), f1(N_LIDAR), ul_plus(N_LIDAR);

        f0 = indirect_lidar_measurement(u, ul, dt);

        double delta = 0.0000001;
        for (size_t i = 0; i < N_LIDAR; i++){
            ul_plus = ul;
            ul_plus(i) = ul_plus(i) + delta;

            f1 = indirect_lidar_measurement(u, ul_plus, dt);
           
            J.block(0,i,N_LIDAR,1) = (f1 - f0)/delta;       
            J(3,i) = sin(f1(3) - f0(3))/delta;
            J(4,i) = sin(f1(4) - f0(4))/delta;
            J(5,i) = sin(f1(5) - f0(5))/delta; 
        }

        return J;
    }

    //----------
    // callbacks
    //----------
    void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn){
        double timeL = ros::Time::now().toSec();

        // time
        if (imuActivated){
            imuTimeLast = imuTimeCurrent;
            imuTimeCurrent = imuIn->header.stamp.toSec();
        }else{
            imuTimeCurrent = imuIn->header.stamp.toSec();
            imuTimeLast = imuTimeCurrent + 0.01;
            imuActivated = true;
        }       

        // roll, pitch and yaw 
        double roll, pitch, yaw;
        geometry_msgs::Quaternion orientation = imuIn->orientation;
        tf::Matrix3x3(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w)).getRPY(roll, pitch, yaw);

        // measure
        imuMeasure.block(0,0,3,1) << imuIn->linear_acceleration.x, imuIn->linear_acceleration.y, imuIn->linear_acceleration.z;
        imuMeasure.block(3,0,3,1) << imuIn->angular_velocity.x, imuIn->angular_velocity.y, imuIn->angular_velocity.z; 
        imuMeasure.block(6,0,3,1) << roll, pitch, yaw;

        // covariance
        E_imu.block(0,0,3,3) << imuIn->linear_acceleration_covariance[0], imuIn->linear_acceleration_covariance[1], imuIn->linear_acceleration_covariance[2],
                                imuIn->linear_acceleration_covariance[3], imuIn->linear_acceleration_covariance[4], imuIn->linear_acceleration_covariance[5],
                                imuIn->linear_acceleration_covariance[6], imuIn->linear_acceleration_covariance[7], imuIn->linear_acceleration_covariance[8];
        E_imu.block(3,3,3,3) << imuIn->angular_velocity_covariance[0], imuIn->angular_velocity_covariance[1], imuIn->angular_velocity_covariance[2],
                                imuIn->angular_velocity_covariance[3], imuIn->angular_velocity_covariance[4], imuIn->angular_velocity_covariance[5],
                                imuIn->angular_velocity_covariance[6], imuIn->angular_velocity_covariance[7], imuIn->angular_velocity_covariance[8];
        E_imu.block(6,6,3,3) << imuIn->orientation_covariance[0], imuIn->orientation_covariance[1], imuIn->orientation_covariance[2],
                                imuIn->orientation_covariance[3], imuIn->orientation_covariance[4], imuIn->orientation_covariance[5],
                                imuIn->orientation_covariance[6], imuIn->orientation_covariance[7], imuIn->orientation_covariance[8];

        E_imu.block(6,6,3,3) = 0.01*E_imu.block(3,3,3,3);

        // time
        imu_dt = imuTimeCurrent - imuTimeLast;
        // imu_dt = 0.01;

        // header
        double timediff = ros::Time::now().toSec() - timeL + imuTimeCurrent;
        headerI = imuIn->header;
        headerI.stamp = ros::Time().fromSec(timediff);

        imuNew = true;
    }

    void wheelOdometryHandler(const nav_msgs::Odometry::ConstPtr& wheelOdometry){
        double timeL = ros::Time::now().toSec();

        // time
        if (wheelActivated){
            wheelTimeLast = wheelTimeCurrent;
            wheelTimeCurrent = wheelOdometry->header.stamp.toSec();
        }else{
            wheelTimeCurrent = wheelOdometry->header.stamp.toSec();
            wheelTimeLast = wheelTimeCurrent + 0.05;
            wheelActivated = true;
        } 

        // measure
        wheelMeasure << 1.0*wheelOdometry->twist.twist.linear.x, wheelOdometry->twist.twist.angular.z;

        // covariance
        E_wheel(0,0) = 0.01*wheelOdometry->twist.covariance[0];
        E_wheel(1,1) = 100*wheelOdometry->twist.covariance[35];

        // time
        wheel_dt = wheelTimeCurrent - wheelTimeLast;
        // wheel_dt = 0.05;

        // header
        double timediff = ros::Time::now().toSec() - timeL + wheelTimeCurrent;
        headerW = wheelOdometry->header;
        headerW.stamp = ros::Time().fromSec(timediff);

        // new measure
        wheelNew =  true;
    }

    void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry){
        double timeL = ros::Time::now().toSec();

        if (lidarActivated){
            lidarTimeLast = lidarTimeCurrent;
            lidarTimeCurrent = laserOdometry->header.stamp.toSec();
        }else{
            lidarTimeCurrent = laserOdometry->header.stamp.toSec();
            lidarTimeLast = lidarTimeCurrent + 0.1;
            lidarActivated = true;
        }  
        
        // roll, pitch and yaw 
        double roll, pitch, yaw;
        geometry_msgs::Quaternion orientation = laserOdometry->pose.pose.orientation;
        tf::Matrix3x3(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w)).getRPY(roll, pitch, yaw);

        lidarMeasure.block(0,0,3,1) << laserOdometry->pose.pose.position.x, laserOdometry->pose.pose.position.y, laserOdometry->pose.pose.position.z;
        lidarMeasure.block(3,0,3,1) << roll, pitch, yaw;    

        // covariance
        double corner = double(laserOdometry->twist.twist.linear.x);
        double surf = double(laserOdometry->twist.twist.angular.x); 

        // E_lidar = adaptive_covariance(corner, surf);
        E_lidar = nfn_adaptive_covariance(corner, surf);

        // time
        lidar_dt = lidarTimeCurrent - lidarTimeLast;
        // lidar_dt = 0.1;

        // header
        double timediff = ros::Time::now().toSec() - timeL + lidarTimeCurrent;
        headerL = laserOdometry->header;
        headerL.stamp = ros::Time().fromSec(timediff);
        
        //New measure
        lidarNew = true;
    }

    //----------
    // publisher
    //----------
    void publish_odom(char model){
        switch(model){
                case 'i':
                    filteredOdometry.header = headerI;
                    break;
                case 'w':
                    filteredOdometry.header = headerW;
                    break;
                case 'l':
                    filteredOdometry.header = headerL;
            }
        
        filteredOdometry.header.frame_id = "chassis_init";
        filteredOdometry.child_frame_id = "/ekf_odom_frame";

        geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw (X(3), X(4), X(5));

        // pose
        filteredOdometry.pose.pose.orientation.x = geoQuat.x;
        filteredOdometry.pose.pose.orientation.y = geoQuat.y;
        filteredOdometry.pose.pose.orientation.z = geoQuat.z;
        filteredOdometry.pose.pose.orientation.w = geoQuat.w;
        filteredOdometry.pose.pose.position.x = X(0); 
        filteredOdometry.pose.pose.position.y = X(1);
        filteredOdometry.pose.pose.position.z = X(2);

        // pose convariance
        int k = 0;
        for (int i = 0; i < 6; i++){
            for (int j = 0; j < 6; j++){
                filteredOdometry.pose.covariance[k] = P(i,j);
                k++;
            }
        }      

        // twist
        filteredOdometry.twist.twist.linear.x = X(6);
        filteredOdometry.twist.twist.linear.y = X(7);
        filteredOdometry.twist.twist.linear.z = X(8);
        filteredOdometry.twist.twist.angular.x = X(9);
        filteredOdometry.twist.twist.angular.y = X(10);
        filteredOdometry.twist.twist.angular.z = X(11);

        // twist convariance
        k = 0;
        for (int i = 6; i < 12; i++){
            for (int j = 6; j < 12; j++){
                filteredOdometry.twist.covariance[k] = P(i,j);
                k++;
            }
        } 

        pubFilteredOdometry.publish(filteredOdometry);
    }

    void publish_indirect_lidar_measurement(VectorXd y, MatrixXd Pi){
        indLiDAROdometry.header = headerL;
        indLiDAROdometry.header.frame_id = "chassis_init";
        indLiDAROdometry.child_frame_id = "/ind_lidar_frame";

        // twist
        indLiDAROdometry.twist.twist.linear.x = y(0);
        indLiDAROdometry.twist.twist.linear.y = y(1);
        indLiDAROdometry.twist.twist.linear.z = y(2);
        indLiDAROdometry.twist.twist.angular.x = y(3);
        indLiDAROdometry.twist.twist.angular.y = y(4);
        indLiDAROdometry.twist.twist.angular.z = y(5);

        // twist convariance
        int k = 0;
        for (int i = 0; i < 6; i++){
            for (int j = 0; j < 6; j++){
                indLiDAROdometry.twist.covariance[k] = Pi(i,j);
                k++;
            }
        } 

        pubIndLiDARMeasurement.publish(indLiDAROdometry);
    }

    //----------
    // runs
    //----------
    void run(){
        // rate
        ros::Rate r(200);        

        double t_last = ros::Time::now().toSec();
        double t_now;
        double dt_now;

        while (ros::ok())
        {
            // Prediction
            if (enableFilter){
                // prediction stage
                t_now = ros::Time::now().toSec();
                dt_now = t_now-t_last;
                t_last = t_now;

                // prediction_stage(1/200.0);
                prediction_stage(dt_now);
                
                // publish state
                if (filterFreq == "p"){
                    publish_odom('p');
                }
            }

            //Correction IMU
            if (enableFilter && enableImu && imuActivated && imuNew){
                // correction stage
                correction_imu_stage(imu_dt);

                // publish state
                if (filterFreq == "i"){
                    publish_odom('i');
                }

                // control variable
                imuNew =  false;
            }

            // Correction wheel
            if (enableFilter && enableWheel && wheelActivated && wheelNew){                
                // correction stage
                correction_wheel_stage(wheel_dt);

                if (filterFreq == "w"){
                    publish_odom('w');
                }                

                // control variable
                wheelNew =  false;
            }

            //Corection LiDAR
            if (enableFilter && enableLidar && lidarActivated && lidarNew){                
                // correction stage
                correction_lidar_stage(lidar_dt);

                // publish state
                if (filterFreq == "l"){
                    publish_odom('l');
                }

                // controle variable
                lidarNew =  false;
            }
            
            ros::spinOnce();
            r.sleep();        
        }
    }

};


//-----------------------------
// Main 
//-----------------------------
int main(int argc, char** argv)
{
    ros::init(argc, argv, "adaptive_filter");

    //Parameters init:    
    ros::NodeHandle nh_;
    try
    {
        nh_.param("/ekf_loam/enableFilter", enableFilter, false);
        nh_.param("/adaptive_filter/enableImu", enableImu, false);
        nh_.param("/adaptive_filter/enableWheel", enableWheel, false);
        nh_.param("/adaptive_filter/enableLidar", enableLidar, false);
        nh_.param("/adaptive_filter/filterFreq", filterFreq, std::string("l"));

        //nfn
        
        nh_.getParam("/nfn/g1wx1", g1wx1);
        nh_.getParam("/nfn/g1wx2", g1wx2);
        nh_.getParam("/nfn/g1wy1", g1wy1);
        nh_.getParam("/nfn/g1wy2", g1wy2);
        nh_.getParam("/nfn/g1wz1", g1wz1);
        nh_.getParam("/nfn/g1wz2", g1wz2);
        nh_.getParam("/nfn/g1wr1", g1wr1);
        nh_.getParam("/nfn/g1wr2", g1wr2);
        nh_.getParam("/nfn/g1wp1", g1wp1);
        nh_.getParam("/nfn/g1wp2", g1wp2);
        nh_.getParam("/nfn/g1wya1", g1wya1);
        nh_.getParam("/nfn/g1wya2", g1wya2);
        nh_.getParam("/nfn/g1gxmin", g1xmin);
        nh_.getParam("/nfn/g1gymin", g1ymin);
        nh_.getParam("/nfn/g1gzmin", g1zmin);
        nh_.getParam("/nfn/g1grmin", g1rmin);
        nh_.getParam("/nfn/g1gpmin", g1pmin);
        nh_.getParam("/nfn/g1gyamin", g1yamin);
        nh_.getParam("/nfn/g1gxmax", g1xmax);
        nh_.getParam("/nfn/g1gymax", g1ymax);
        nh_.getParam("/nfn/g1gzmax", g1zmax);
        nh_.getParam("/nfn/g1grmax", g1rmax);
        nh_.getParam("/nfn/g1gpmax", g1pmax);
        nh_.getParam("/nfn/g1gyamax", g1yamax);

        nh_.getParam("/nfn/g2wx1", g2wx1);
        nh_.getParam("/nfn/g2wx2", g2wx2);
        nh_.getParam("/nfn/g2wy1", g2wy1);
        nh_.getParam("/nfn/g2wy2", g2wy2);
        nh_.getParam("/nfn/g2wz1", g2wz1);
        nh_.getParam("/nfn/g2wz2", g2wz2);
        nh_.getParam("/nfn/g2wr1", g2wr1);
        nh_.getParam("/nfn/g2wr2", g2wr2);
        nh_.getParam("/nfn/g2wp1", g2wp1);
        nh_.getParam("/nfn/g2wp2", g2wp2);
        nh_.getParam("/nfn/g2wya1", g2wya1);
        nh_.getParam("/nfn/g2wya2", g2wya2);
        nh_.getParam("/nfn/g2gxmin", g2xmin);
        nh_.getParam("/nfn/g2gymin", g2ymin);
        nh_.getParam("/nfn/g2gzmin", g2zmin);
        nh_.getParam("/nfn/g2grmin", g2rmin);
        nh_.getParam("/nfn/g2gpmin", g2pmin);
        nh_.getParam("/nfn/g2gyamin", g2yamin);
        nh_.getParam("/nfn/g2gxmax", g2xmax);
        nh_.getParam("/nfn/g2gymax", g2ymax);
        nh_.getParam("/nfn/g2gzmax", g2zmax);
        nh_.getParam("/nfn/g2grmax", g2rmax);
        nh_.getParam("/nfn/g2gpmax", g2pmax);
        nh_.getParam("/nfn/g2gyamax", g2yamax);

        nh_.getParam("/nfn/g3wx1", g3wx1);
        nh_.getParam("/nfn/g3wx2", g3wx2);
        nh_.getParam("/nfn/g3wy1", g3wy1);
        nh_.getParam("/nfn/g3wy2", g3wy2);
        nh_.getParam("/nfn/g3wz1", g3wz1);
        nh_.getParam("/nfn/g3wz2", g3wz2);
        nh_.getParam("/nfn/g3wr1", g3wr1);
        nh_.getParam("/nfn/g3wr2", g3wr2);
        nh_.getParam("/nfn/g3wp1", g3wp1);
        nh_.getParam("/nfn/g3wp2", g3wp2);
        nh_.getParam("/nfn/g3wya1", g3wya1);
        nh_.getParam("/nfn/g3wya2", g3wya2);
        nh_.getParam("/nfn/g3gxmin", g3xmin);
        nh_.getParam("/nfn/g3gymin", g3ymin);
        nh_.getParam("/nfn/g3gzmin", g3zmin);
        nh_.getParam("/nfn/g3grmin", g3rmin);
        nh_.getParam("/nfn/g3gpmin", g3pmin);
        nh_.getParam("/nfn/g3gyamin", g3yamin);
        nh_.getParam("/nfn/g3gxmax", g3xmax);
        nh_.getParam("/nfn/g3gymax", g3ymax);
        nh_.getParam("/nfn/g3gzmax", g3zmax);
        nh_.getParam("/nfn/g3grmax", g3rmax);
        nh_.getParam("/nfn/g3gpmax", g3pmax);
        nh_.getParam("/nfn/g3gyamax", g3yamax);

        nh_.getParam("/nfn/g4wx1", g4wx1);
        nh_.getParam("/nfn/g4wx2", g4wx2);
        nh_.getParam("/nfn/g4wy1", g4wy1);
        nh_.getParam("/nfn/g4wy2", g4wy2);
        nh_.getParam("/nfn/g4wz1", g4wz1);
        nh_.getParam("/nfn/g4wz2", g4wz2);
        nh_.getParam("/nfn/g4wr1", g4wr1);
        nh_.getParam("/nfn/g4wr2", g4wr2);
        nh_.getParam("/nfn/g4wp1", g4wp1);
        nh_.getParam("/nfn/g4wp2", g4wp2);
        nh_.getParam("/nfn/g4wya1", g4wya1);
        nh_.getParam("/nfn/g4wya2", g4wya2);
        nh_.getParam("/nfn/g4gxmin", g4xmin);
        nh_.getParam("/nfn/g4gymin", g4ymin);
        nh_.getParam("/nfn/g4gzmin", g4zmin);
        nh_.getParam("/nfn/g4grmin", g4rmin);
        nh_.getParam("/nfn/g4gpmin", g4pmin);
        nh_.getParam("/nfn/g4gyamin", g4yamin);
        nh_.getParam("/nfn/g4gxmax", g4xmax);
        nh_.getParam("/nfn/g4gymax", g4ymax);
        nh_.getParam("/nfn/g4gzmax", g4zmax);
        nh_.getParam("/nfn/g4grmax", g4rmax);
        nh_.getParam("/nfn/g4gpmax", g4pmax);
        nh_.getParam("/nfn/g4gyamax", g4yamax);

        nh_.getParam("/nfn/g5wx1", g5wx1);
        nh_.getParam("/nfn/g5wx2", g5wx2);
        nh_.getParam("/nfn/g5wy1", g5wy1);
        nh_.getParam("/nfn/g5wy2", g5wy2);
        nh_.getParam("/nfn/g5wz1", g5wz1);
        nh_.getParam("/nfn/g5wz2", g5wz2);
        nh_.getParam("/nfn/g5wr1", g5wr1);
        nh_.getParam("/nfn/g5wr2", g5wr2);
        nh_.getParam("/nfn/g5wp1", g5wp1);
        nh_.getParam("/nfn/g5wp2", g5wp2);
        nh_.getParam("/nfn/g5wya1", g5wya1);
        nh_.getParam("/nfn/g5wya2", g5wya2);
        nh_.getParam("/nfn/g5gxmin", g5xmin);
        nh_.getParam("/nfn/g5gymin", g5ymin);
        nh_.getParam("/nfn/g5gzmin", g5zmin);
        nh_.getParam("/nfn/g5grmin", g5rmin);
        nh_.getParam("/nfn/g5gpmin", g5pmin);
        nh_.getParam("/nfn/g5gyamin", g5yamin);
        nh_.getParam("/nfn/g5gxmax", g5xmax);
        nh_.getParam("/nfn/g5gymax", g5ymax);
        nh_.getParam("/nfn/g5gzmax", g5zmax);
        nh_.getParam("/nfn/g5grmax", g5rmax);
        nh_.getParam("/nfn/g5gpmax", g5pmax);
        nh_.getParam("/nfn/g5gyamax", g5yamax);

        nh_.getParam("/nfn/g6wx1", g6wx1);
        nh_.getParam("/nfn/g6wx2", g6wx2);
        nh_.getParam("/nfn/g6wy1", g6wy1);
        nh_.getParam("/nfn/g6wy2", g6wy2);
        nh_.getParam("/nfn/g6wz1", g6wz1);
        nh_.getParam("/nfn/g6wz2", g6wz2);
        nh_.getParam("/nfn/g6wr1", g6wr1);
        nh_.getParam("/nfn/g6wr2", g6wr2);
        nh_.getParam("/nfn/g6wp1", g6wp1);
        nh_.getParam("/nfn/g6wp2", g6wp2);
        nh_.getParam("/nfn/g6wya1", g6wya1);
        nh_.getParam("/nfn/g6wya2", g6wya2);
        nh_.getParam("/nfn/g6gxmin", g6xmin);
        nh_.getParam("/nfn/g6gymin", g6ymin);
        nh_.getParam("/nfn/g6gzmin", g6zmin);
        nh_.getParam("/nfn/g6grmin", g6rmin);
        nh_.getParam("/nfn/g6gpmin", g6pmin);
        nh_.getParam("/nfn/g6gyamin", g6yamin);
        nh_.getParam("/nfn/g6gxmax", g6xmax);
        nh_.getParam("/nfn/g6gymax", g6ymax);
        nh_.getParam("/nfn/g6gzmax", g6zmax);
        nh_.getParam("/nfn/g6grmax", g6rmax);
        nh_.getParam("/nfn/g6gpmax", g6pmax);
        nh_.getParam("/nfn/g6gyamax", g6yamax);

        nh_.getParam("/nfn/g7wx1", g7wx1);
        nh_.getParam("/nfn/g7wx2", g7wx2);
        nh_.getParam("/nfn/g7wy1", g7wy1);
        nh_.getParam("/nfn/g7wy2", g7wy2);
        nh_.getParam("/nfn/g7wz1", g7wz1);
        nh_.getParam("/nfn/g7wz2", g7wz2);
        nh_.getParam("/nfn/g7wr1", g7wr1);
        nh_.getParam("/nfn/g7wr2", g7wr2);
        nh_.getParam("/nfn/g7wp1", g7wp1);
        nh_.getParam("/nfn/g7wp2", g7wp2);
        nh_.getParam("/nfn/g7wya1", g7wya1);
        nh_.getParam("/nfn/g7wya2", g7wya2);
        nh_.getParam("/nfn/g7gxmin", g7xmin);
        nh_.getParam("/nfn/g7gymin", g7ymin);
        nh_.getParam("/nfn/g7gzmin", g7zmin);
        nh_.getParam("/nfn/g7grmin", g7rmin);
        nh_.getParam("/nfn/g7gpmin", g7pmin);
        nh_.getParam("/nfn/g7gyamin", g7yamin);
        nh_.getParam("/nfn/g7gxmax", g7xmax);
        nh_.getParam("/nfn/g7gymax", g7ymax);
        nh_.getParam("/nfn/g7gzmax", g7zmax);
        nh_.getParam("/nfn/g7grmax", g7rmax);
        nh_.getParam("/nfn/g7gpmax", g7pmax);
        nh_.getParam("/nfn/g7gyamax", g7yamax);

        nh_.getParam("/nfn/g8wx1", g8wx1);
        nh_.getParam("/nfn/g8wx2", g8wx2);
        nh_.getParam("/nfn/g8wy1", g8wy1);
        nh_.getParam("/nfn/g8wy2", g8wy2);
        nh_.getParam("/nfn/g8wz1", g8wz1);
        nh_.getParam("/nfn/g8wz2", g8wz2);
        nh_.getParam("/nfn/g8wr1", g8wr1);
        nh_.getParam("/nfn/g8wr2", g8wr2);
        nh_.getParam("/nfn/g8wp1", g8wp1);
        nh_.getParam("/nfn/g8wp2", g8wp2);
        nh_.getParam("/nfn/g8wya1", g8wya1);
        nh_.getParam("/nfn/g8wya2", g8wya2);
        nh_.getParam("/nfn/g8gxmin", g8xmin);
        nh_.getParam("/nfn/g8gymin", g8ymin);
        nh_.getParam("/nfn/g8gzmin", g8zmin);
        nh_.getParam("/nfn/g8grmin", g8rmin);
        nh_.getParam("/nfn/g8gpmin", g8pmin);
        nh_.getParam("/nfn/g8gyamin", g8yamin);
        nh_.getParam("/nfn/g8gxmax", g8xmax);
        nh_.getParam("/nfn/g8gymax", g8ymax);
        nh_.getParam("/nfn/g8gzmax", g8zmax);
        nh_.getParam("/nfn/g8grmax", g8rmax);
        nh_.getParam("/nfn/g8gpmax", g8pmax);
        nh_.getParam("/nfn/g8gyamax", g8yamax);

        nh_.getParam("/nfn/g9wx1", g9wx1);
        nh_.getParam("/nfn/g9wx2", g9wx2);
        nh_.getParam("/nfn/g9wy1", g9wy1);
        nh_.getParam("/nfn/g9wy2", g9wy2);
        nh_.getParam("/nfn/g9wz1", g9wz1);
        nh_.getParam("/nfn/g9wz2", g9wz2);
        nh_.getParam("/nfn/g9wr1", g9wr1);
        nh_.getParam("/nfn/g9wr2", g9wr2);
        nh_.getParam("/nfn/g9wp1", g9wp1);
        nh_.getParam("/nfn/g9wp2", g9wp2);
        nh_.getParam("/nfn/g9wya1", g9wya1);
        nh_.getParam("/nfn/g9wya2", g9wya2);
        nh_.getParam("/nfn/g9gxmin", g9xmin);
        nh_.getParam("/nfn/g9gymin", g9ymin);
        nh_.getParam("/nfn/g9gzmin", g9zmin);
        nh_.getParam("/nfn/g9grmin", g9rmin);
        nh_.getParam("/nfn/g9gpmin", g9pmin);
        nh_.getParam("/nfn/g9gyamin", g9yamin);
        nh_.getParam("/nfn/g9gxmax", g9xmax);
        nh_.getParam("/nfn/g9gymax", g9ymax);
        nh_.getParam("/nfn/g9gzmax", g9zmax);
        nh_.getParam("/nfn/g9grmax", g9rmax);
        nh_.getParam("/nfn/g9gpmax", g9pmax);
        nh_.getParam("/nfn/g9gyamax", g9yamax);


        nh_.getParam("/nfn/g10wx1", g10wx1);
        nh_.getParam("/nfn/g10wx2", g10wx2);
        nh_.getParam("/nfn/g10wy1", g10wy1);
        nh_.getParam("/nfn/g10wy2", g10wy2);
        nh_.getParam("/nfn/g10wz1", g10wz1);
        nh_.getParam("/nfn/g10wz2", g10wz2);
        nh_.getParam("/nfn/g10wp1", g10wp1);
        nh_.getParam("/nfn/g10wp2", g10wp2);
        nh_.getParam("/nfn/g10gxmin", g10xmin);
        nh_.getParam("/nfn/g10gymin", g10ymin);
        nh_.getParam("/nfn/g10gzmin", g10zmin);
        nh_.getParam("/nfn/g10gpmin", g10pmin);
        nh_.getParam("/nfn/g10gxmax", g10xmax);
        nh_.getParam("/nfn/g10gymax", g10ymax);
        nh_.getParam("/nfn/g10gzmax", g10zmax);
        nh_.getParam("/nfn/g10gpmax", g10pmax);
        
    }
    catch (int e)
    {
        ROS_INFO("\033[1;31m---->\033[0m Exception occurred when importing parameters in Adaptive Filter Node. Exception Nr. %d", e);
    }
    
    AdaptiveFilter AF;

    if (enableFilter){
        ROS_INFO("\033[1;32m---->\033[0m Adaptive Filter Started.");
        // runs
        AF.run();
    }else{
        ROS_INFO("\033[1;32m---->\033[0m Adaptive Filter Stopped.");
    }
    
    ros::spin();
    return 0;
}

