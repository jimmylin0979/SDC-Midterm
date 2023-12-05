    #include <iostream>
#include <string>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/common/io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

using namespace std;
using namespace cv;
using namespace ros;

const float range_resolution = 0.175;
Publisher radar_pub;

bool intensity_compare(pcl::PointXYZI a, pcl::PointXYZI b)
{
    return a.intensity > b.intensity;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr create_radar_pc(Mat img)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr new_pc(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr new_pc_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    
    // TODO Transform Polar Image to Cartesian Pointcloud
    // Mat img's shape (rows, cols) (2856, 400)
    float max_intensity = 0, min_intensity = 255;
    
    // for (int r = 4; r < img.rows; r++) {
    //     for (int c = 0; c < img.cols; c++) {
    for (int c = 0; c < img.cols; c++) {

        vector<pcl::PointXYZI> points;
        for (int r = 4; r < img.rows; r++) {

            float range = double(r) * range_resolution;
            if (range > 100.0)
                continue;

            float azimuth = (double(c) * (2 * M_PI / img.cols));
            float x = range * cos(azimuth);
            float y = range * -sin(azimuth);    // Flip 
            float z = 0;
            float intensity = (float)img.at<uchar>(r, c);

            max_intensity = max(max_intensity, intensity);
            min_intensity = min(min_intensity, intensity);
            if (intensity < 50.0)  continue;
            
            pcl::PointXYZI point;
            point.x = x;
            point.y = y;    
            point.z = z;
            point.intensity = intensity;
            points.push_back(point);
        }

        // 
        sort(points.begin(), points.end(), intensity_compare);
        for (int i = 0; i < points.size(); i++) {
            if (i > 30)   break;
            new_pc->push_back(points[i]);
        }
    }

    // cout << "min_intensity = " << min_intensity << ", max_intensity = " << max_intensity << "." << endl; 

    return new_pc;

    // Create the filtering object
    // https://pcl.readthedocs.io/projects/tutorials/en/latest/statistical_outlier.html
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(new_pc);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*new_pc_filtered);

    return new_pc_filtered;
}

void radarCallback(const sensor_msgs::ImageConstPtr &msg)
{
    Mat img;
    cv_bridge::CvImageConstPtr cv_ptr;
    cv_ptr = cv_bridge::toCvShare(msg, "mono8");
    img = cv_ptr->image;
    pcl::PointCloud<pcl::PointXYZI>::Ptr radar_pc_ptr = create_radar_pc(img);
    sensor_msgs::PointCloud2 pc_msg;
    pcl::toROSMsg(*radar_pc_ptr, pc_msg);
    pc_msg.header.stamp = ros::Time::now();
    pc_msg.header.frame_id = "navtech";
    radar_pub.publish(pc_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "radar_polar_to_pointcloud");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    radar_pub = nh.advertise<sensor_msgs::PointCloud2>("/radar_pc", 1);
    image_transport::Subscriber sub = it.subscribe("/Navtech/Polar", 1, radarCallback);

    ros::spin();
    return 0;
}