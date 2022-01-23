// Author of SSL_SLAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

//c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>

//ros lib
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//local lib
#include "lidar.h"
#include "odomEstimationClass.h"

OdomEstimationClass odomEstimation;
std::mutex mutex_lock;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudEdgeBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudSurfBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudBuf;
lidar::Lidar lidar_param;

ros::Publisher pubLaserOdometry;
void velodyneSurfHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudSurfBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}
void velodyneEdgeHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudEdgeBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}
void velodyneHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}

bool is_odom_inited = false;
double total_time =0;
int total_frame=0;
void odom_estimation(){
    while(1){
        if(!pointCloudEdgeBuf.empty() && !pointCloudSurfBuf.empty()&& !pointCloudBuf.empty()){

            //read data
            mutex_lock.lock();
            if(!pointCloudBuf.empty() && (pointCloudBuf.front()->header.stamp.toSec()<pointCloudSurfBuf.front()->header.stamp.toSec()-0.5*lidar_param.scan_period || pointCloudBuf.front()->header.stamp.toSec()<pointCloudEdgeBuf.front()->header.stamp.toSec()-0.5*lidar_param.scan_period)){
                ROS_WARN("time stamp unaligned error and odom discarded, pls check your data --> odom correction"); 
                pointCloudBuf.pop();
                mutex_lock.unlock();
                continue;              
            }

            if(!pointCloudSurfBuf.empty() && (pointCloudSurfBuf.front()->header.stamp.toSec()<pointCloudBuf.front()->header.stamp.toSec()-0.5*lidar_param.scan_period || pointCloudSurfBuf.front()->header.stamp.toSec()<pointCloudEdgeBuf.front()->header.stamp.toSec()-0.5*lidar_param.scan_period)){
                pointCloudSurfBuf.pop();
                ROS_INFO("time stamp unaligned with extra point cloud, pls check your data --> odom correction");
                mutex_lock.unlock();
                continue;  
            }

            if(!pointCloudEdgeBuf.empty() && (pointCloudEdgeBuf.front()->header.stamp.toSec()<pointCloudBuf.front()->header.stamp.toSec()-0.5*lidar_param.scan_period || pointCloudEdgeBuf.front()->header.stamp.toSec()<pointCloudSurfBuf.front()->header.stamp.toSec()-0.5*lidar_param.scan_period)){
                pointCloudEdgeBuf.pop();
                ROS_INFO("time stamp unaligned with extra point cloud, pls check your data --> odom correction");
                mutex_lock.unlock();
                continue;  
            }
            //if time aligned 

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_surf_in(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_edge_in(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::fromROSMsg(*pointCloudEdgeBuf.front(), *pointcloud_edge_in);
            pcl::fromROSMsg(*pointCloudSurfBuf.front(), *pointcloud_surf_in);
            pcl::fromROSMsg(*pointCloudBuf.front(), *pointcloud_in);
            ros::Time pointcloud_time = (pointCloudBuf.front())->header.stamp;
            pointCloudEdgeBuf.pop();
            pointCloudSurfBuf.pop();
            pointCloudBuf.pop();
            mutex_lock.unlock();

            if(is_odom_inited == false){
                odomEstimation.initMapWithPoints(pointcloud_edge_in, pointcloud_surf_in);
                is_odom_inited = true;
                ROS_INFO("odom inited");
            }else{
                std::chrono::time_point<std::chrono::system_clock> start, end;
                start = std::chrono::system_clock::now();
                odomEstimation.updatePointsToMap(pointcloud_edge_in, pointcloud_surf_in);
                end = std::chrono::system_clock::now();
                std::chrono::duration<float> elapsed_seconds = end - start;
                total_frame++;
                float time_temp = elapsed_seconds.count() * 1000;
                total_time+=time_temp;
                if(total_frame%100==0)
                    ROS_INFO("average odom estimation time %f ms \n \n", total_time/total_frame);
            }

            Eigen::Quaterniond q_current(odomEstimation.odom.rotation());
            //q_current.normalize();
            Eigen::Vector3d t_current = odomEstimation.odom.translation();

            static tf::TransformBroadcaster br;
            tf::Transform transform;

//            ------------with modification------------
            double theta_degree = lidar_param.vertical_angle;
//            std::cout << std::endl;
//            std::cout << "vertical angle: " << theta_degree << std::endl;
//            std::cout << std::endl;
            double theta_radians = theta_degree/180*M_PI; // in radians
            tf::Vector3 t(cos(0.5*M_PI - theta_radians)*t_current.x() + cos(theta_radians)*t_current.z(),
                          t_current.y(),
                          -cos(theta_radians)*t_current.x() + cos(0.5*M_PI - theta_radians)*t_current.z());
            transform.setOrigin(t);

            tf::Matrix3x3 camera_rotation_m(cos(0.5*M_PI - theta_radians) , 0, sin(0.5*M_PI - theta_radians),
                                            0                  , 1,                  0,
                                            -sin(0.5*M_PI - theta_radians), 0, cos(0.5*M_PI - theta_radians)); // rotation around Y - axis

            tf::Quaternion q(q_current.x(),q_current.y(),q_current.z(),q_current.w());
            tf::Matrix3x3 original_q_m(q);
            camera_rotation_m *= original_q_m;
            tf::Quaternion composed_q = tf::Quaternion();
            camera_rotation_m.getRotation(composed_q);
//
//            std::cout << std::endl;
//            std::cout << camera_rotation_m[0][0] << " " << camera_rotation_m[0][1] << " " << camera_rotation_m[0][2] << std::endl;
//            std::cout << camera_rotation_m[1][0] << " " << camera_rotation_m[1][1] << " " << camera_rotation_m[1][2] << std::endl;
//            std::cout << camera_rotation_m[2][0] << " " << camera_rotation_m[2][1] << " " << camera_rotation_m[2][2] << std::endl;
//            std::cout << std::endl;
//
//            std::cout << "Quaternion: " << std::endl;
//            std::cout << composed_q[0] << " " << composed_q[1] << " " << composed_q[2] << " " << composed_q[3] << std::endl;
//            std::cout << std::endl;
//
            transform.setRotation(composed_q);

//            tf::Quaternion q(0, 0.2588223, 0, 0.9659249); // quaternion for 60 degree

//            ------------no modification------------
//            tf::Vector3 t(t_current.x(), t_current.y(), t_current.z());
//            transform.setOrigin(t);
//            tf::Quaternion q(q_current.x(),q_current.y(),q_current.z(),q_current.w());
//
//            tf::Quaternion test_q(0, 0.258819, 0, 0.9659258);
//            tf::Matrix3x3 test_m(test_q);
//            tf::Matrix3x3 identity_m(1, 0, 0, 0, 1, 0, 0, 0, 1);
//
//            test_m *= identity_m;
//            double roll, pitch, yaw;
//            test_m.getRPY(roll, pitch, yaw); // row, pitch, and yaw is represented in ZYX euler angle
//
//            std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;
//            std::cout << std::endl;
//            std::cout << test_m[0][0] << " " << test_m[0][1] << " " << test_m[0][2] << std::endl;
//            std::cout << test_m[1][0] << " " << test_m[1][1] << " " << test_m[1][2] << std::endl;
//            std::cout << test_m[2][0] << " " << test_m[2][1] << " " << test_m[2][2] << std::endl;
//            std::cout << std::endl;
//
//            tf::Quaternion test_q_again = tf::Quaternion();
//            test_m.getRotation(test_q_again);
//
//            std::cout << std::endl;
//            std::cout << "Quat" << std::endl;
//            std::cout << test_q_again[0] << " " << test_q_again[1] << " " << test_q_again[2] << std::endl;
//            std::cout << std::endl;
//
//            transform.setRotation(q);





            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));

            // publish odometry
            nav_msgs::Odometry laserOdometry;
            laserOdometry.header.frame_id = "map"; 
            laserOdometry.child_frame_id = "base_link"; 
            laserOdometry.header.stamp = pointcloud_time;
//          ------------with modification------------
//            laserOdometry.pose.pose.orientation.x = 0;
//            laserOdometry.pose.pose.orientation.y = 0.2588223;
//            laserOdometry.pose.pose.orientation.z = 0;
//            laserOdometry.pose.pose.orientation.w = 0.9659249;
//            laserOdometry.pose.pose.position.x = 0.866*t_current.x() + 0.5*t_current.z();
//            laserOdometry.pose.pose.position.y = t_current.y();
//            laserOdometry.pose.pose.position.z = -0.5*t_current.x() + 0.866*t_current.z();
            laserOdometry.pose.pose.orientation.x = composed_q[0];
            laserOdometry.pose.pose.orientation.y = composed_q[1];
            laserOdometry.pose.pose.orientation.z = composed_q[2];
            laserOdometry.pose.pose.orientation.w = composed_q[3];
            laserOdometry.pose.pose.position.x = t[0];
            laserOdometry.pose.pose.position.y = t[1];
            laserOdometry.pose.pose.position.z = t[2];


//          ------------no modification------------
//            laserOdometry.pose.pose.orientation.x = q_current.x();
//            laserOdometry.pose.pose.orientation.y = q_current.y();
//            laserOdometry.pose.pose.orientation.z = q_current.z();
//            laserOdometry.pose.pose.orientation.w = q_current.w();
//            laserOdometry.pose.pose.position.x = t_current.x();
//            laserOdometry.pose.pose.position.y = t_current.y();
//            laserOdometry.pose.pose.position.z = t_current.z();

            pubLaserOdometry.publish(laserOdometry);

        }
        //sleep 2 ms every time
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    int scan_line = 64;
    double vertical_angle = 2.0;
    double scan_period= 0.1;
    double max_dis = 60.0;
    double min_dis = 2.0;
    double map_resolution = 0.4;
    nh.getParam("/scan_period", scan_period); 
    nh.getParam("/vertical_angle", vertical_angle); 
    nh.getParam("/max_dis", max_dis);
    nh.getParam("/min_dis", min_dis);
    nh.getParam("/scan_line", scan_line);
    nh.getParam("/map_resolution", map_resolution);

    lidar_param.setScanPeriod(scan_period);
    lidar_param.setVerticalAngle(vertical_angle);
    lidar_param.setLines(scan_line);
    lidar_param.setMaxDistance(max_dis);
    lidar_param.setMinDistance(min_dis);

    odomEstimation.init(lidar_param, map_resolution);
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points_filtered", 100, velodyneHandler);
    ros::Subscriber subEdgeLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_edge", 100, velodyneEdgeHandler);
    ros::Subscriber subSurfLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf", 100, velodyneSurfHandler);

    pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/odom", 100);
    std::thread odom_estimation_process{odom_estimation};

    ros::spin();

    return 0;
}
