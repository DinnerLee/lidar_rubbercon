#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <math.h>
#include <std_msgs/Float32.h>

#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
//OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <cv_bridge/cv_bridge.h>

#include <Eigen/Dense>
#include <pcl/common/transformation_from_correspondences.h>
#include <geometry_msgs/Twist.h>

class LIDAR
{
private:
    ros::NodeHandle node;
    ros::Subscriber scan_sub_;
    ros::Subscriber scan_image_;
    ros::Subscriber sub_key;
    ros::Subscriber sub_person;
    ros::Publisher path_pub;
    ros::Publisher theta_pub;
    ros::Publisher stop_pub;


    laser_geometry::LaserProjection projector_;
    tf::TransformListener tfListener_;
    pcl::PointCloud <pcl::PointXYZI> point;

    std::vector<bool> obj_rl;

    cv_bridge::CvImagePtr rawImagePtr;
    cv::Mat rawImage;
    bool mode; float theta; float person;
public:
    LIDAR();
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);    //라이다 콜백함수
    void subImgCallback(const sensor_msgs::Image& subImgMsgs);
    void subPersonCallback(const std_msgs::Float32& subMsgs);
    void keycallback(const geometry_msgs::Twist::ConstPtr &msg);
    void init();

    pcl::PointCloud <pcl::PointXYZI> Passthrough_ob(pcl::PointCloud <pcl::PointXYZI> point,
                                                        double minx, double maxx, double miny, double maxy); //ROI LiDAR
    pcl::PointCloud <pcl::PointXYZI> Clustering(pcl::PointCloud<pcl::PointXYZI> point);
    pcl::PointCloud <pcl::PointXYZI> Make_Path(pcl::PointCloud<pcl::PointXYZI> point);
};
