#include "LIDAR.h"

LIDAR::LIDAR(){
    mode = false; theta = 0;

    scan_sub_ = node.subscribe<sensor_msgs::LaserScan> ("/scan", 100, &LIDAR::scanCallback, this); //기존의 라이다토픽 가져오기
    sub_person = node.subscribe("/path/detection", 100, &LIDAR::subPersonCallback, this);
    path_pub = node.advertise<sensor_msgs::PointCloud2>("lidar/path", 100, false);
    theta_pub = node.advertise<std_msgs::Float32>("path/theta", 100, false);
    stop_pub = node.advertise<std_msgs::Float32>("path/stop", 100, false);
    sub_key = node.subscribe<geometry_msgs::Twist>("/cmd_vel", 1000, &LIDAR::keycallback, this);

}

void LIDAR::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    sensor_msgs::PointCloud2 cloud;

    projector_.transformLaserScanToPointCloud("laser", *scan, cloud, tfListener_);
    pcl::fromROSMsg(cloud, point);
    init();
}

void LIDAR::subPersonCallback(const std_msgs::Float32 &subMsgs){
    person = subMsgs.data;
}

void LIDAR::keycallback(const geometry_msgs::Twist::ConstPtr &msg){
    if(msg->linear.x == 0.5 && msg->angular.z == -1){mode = true;}
    else{mode = false;}
}

void LIDAR::subImgCallback(const sensor_msgs::Image& subImgMsgs){
    if(subImgMsgs.data.size())
    {
        rawImagePtr = cv_bridge::toCvCopy(subImgMsgs, sensor_msgs::image_encodings::BGR8);
        rawImage = rawImagePtr->image;
        rawImagePtr->image = rawImage;
    }
}

void LIDAR::init(){ //total

    pcl::PointCloud<pcl::PointXYZI> roi_point;
    pcl::PointCloud<pcl::PointXYZI> cluster_point;
    pcl::PointCloud<pcl::PointXYZI> path_point;

    roi_point = Passthrough_ob(point, 0, 2, -1, 1);
    cluster_point = Clustering(roi_point);
    path_point = Make_Path(cluster_point);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(path_point, output);
    output.header.frame_id = "laser";
    path_pub.publish(output);
}

pcl::PointCloud <pcl::PointXYZI> LIDAR::Passthrough_ob(pcl::PointCloud <pcl::PointXYZI> point, double minx, double maxx, double miny, double maxy){ //장애물의 전체적인 범위 설정을 통해서 필요없는 부분 제거
    pcl::PointCloud <pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud <pcl::PointXYZI>::Ptr cloud_filter (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud <pcl::PointXYZI> filter;
    pcl::PassThrough <pcl::PointXYZI> pass;

    *cloud = point;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(minx, maxx);
    pass.filter(*cloud_filter);
    pass.setInputCloud(cloud_filter);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(miny, maxy);
    pass.filter(*cloud_filter);
    filter = *cloud_filter;

    return filter;
}

pcl::PointCloud <pcl::PointXYZI> LIDAR::Clustering(pcl::PointCloud<pcl::PointXYZI> point){
    pcl::PointCloud <pcl::PointXYZI> result_Path;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_obstacle(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_obstacle = point.makeShared();
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cloud_obstacle);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(0.1);
    ec.setMinClusterSize(3);
    ec.setMaxClusterSize(1000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_obstacle);
    ec.extract(cluster_indices);

    int j = 0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        float sum_y = 0; float sum_x = 0; int count = 0;
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
            pcl::PointXYZI pt = cloud_obstacle->points[*pit];
            pt.intensity = j;
            //result_Path.push_back(pt);
            sum_x += pt._PointXYZI::x;
            sum_y += pt._PointXYZI::y;
            count++;
        }
        sum_x = sum_x / count;
        sum_y = sum_y / count;
        pcl::PointXYZI pt_arr;
        pt_arr._PointXYZI::x = sum_x;
        pt_arr._PointXYZI::y = sum_y;
        pt_arr._PointXYZI::z = 0;
        pt_arr._PointXYZI::intensity = j;
        result_Path.push_back(pt_arr);

        if(sum_y >= 0){obj_rl.push_back(true);}
        else{obj_rl.push_back(false);}
        j++;
    }

    return result_Path;
}

pcl::PointCloud<pcl::PointXYZI> LIDAR::Make_Path(pcl::PointCloud<pcl::PointXYZI> point){
    pcl::PointCloud<pcl::PointXYZI> result_path;
    cv::Mat img; img = cv::Mat::zeros(200, 150, CV_8UC3);
    double v = 0; double w = 0;
    for(int i = 0; i < point.size(); i++){
        v = 100 - (point.at(i).x)*50; w = 75 - (point.at(i).y)*50;
        img.at<cv::Vec3b>(v, w)[0] = 255;
        img.at<cv::Vec3b>(v, w)[1] = 255; img.at<cv::Vec3b>(v, w)[2] = 255; 

        if(point.at(i).y >= 0){
            cv::rectangle(img, cv::Point((75 - (point.at(i).y - 0.2)*50), (100 - (point.at(i).x - 0.2)*50)),
                          cv::Point((75 - (point.at(i).y + 0.2)*50), (100 - (point.at(i).x + 0.2)*50)), cv::Scalar(255, 0, 0), -1);

            cv::line(img, cv::Point((75 - (point.at(i).y - 0.4)*50), (100 - (point.at(i).x - 0.15)*50)),
                     cv::Point((75 - (point.at(i).y - 0.4)*50), (100 - (point.at(i).x + 0.15)*50)), cv::Scalar(255, 0, 255), 1);
        }
        else{
            cv::rectangle(img, cv::Point((75 - (point.at(i).y - 0.2)*50), (100 - (point.at(i).x - 0.2)*50)),
                          cv::Point((75 - (point.at(i).y + 0.2)*50), (100 - (point.at(i).x + 0.2)*50)), cv::Scalar(0, 255, 0), -1);

            cv::line(img, cv::Point((75 - (point.at(i).y + 0.4)*50), (100 - (point.at(i).x - 0.15)*50)),
                     cv::Point((75 - (point.at(i).y + 0.4)*50), (100 - (point.at(i).x + 0.15)*50)), cv::Scalar(0, 255, 255), 1);
        }
    }
    cv::Mat roi(img, cv::Rect(0, 0, 150, 100));

    cv::imshow("result", roi);
    cv::waitKey(4);

    int left_in = 0;
    int right_in = 255;

    for(int y = 0; y < roi.rows; y++){
        for(int x = 0; x < roi.cols; x++){
            if(roi.at<cv::Vec3b>(y, x)[0] == 255
                    && roi.at<cv::Vec3b>(y, x)[1] == 0
                    && roi.at<cv::Vec3b>(y, x)[2] == 255){
                pcl::PointXYZI arr;
                arr._PointXYZI::x = (float)(100 - y)/50;
                arr._PointXYZI::y = (float)(75 - x)/50;
                arr._PointXYZI::z = 0; arr._PointXYZI::intensity = 127 - abs(arr._PointXYZI::y*100);
                result_path.push_back(arr);
                if(arr._PointXYZI::x >= 0.25 && arr._PointXYZI::x < 0.5 && left_in < arr._PointXYZI::intensity){
                    left_in = arr._PointXYZI::intensity;
                }
            }
            if(roi.at<cv::Vec3b>(y, x)[0] == 0
                    && roi.at<cv::Vec3b>(y, x)[2] == 255
                    && roi.at<cv::Vec3b>(y, x)[1] == 255){
                pcl::PointXYZI arr;
                arr._PointXYZI::x = (float)(100 - y)/50;
                arr._PointXYZI::y = (float)(75 - x)/50;
                arr._PointXYZI::z = 0; arr._PointXYZI::intensity = 127 + abs(arr._PointXYZI::y*100);
                result_path.push_back(arr);
                if(arr._PointXYZI::x >= 0.25 && arr._PointXYZI::x < 0.5 && right_in > arr._PointXYZI::intensity){
                    right_in = arr._PointXYZI::intensity;
                }
            }
        }
    }
    if(left_in == 0 || right_in ==255){
        for(int y = 0; y < roi.rows; y++){
            for(int x = 0; x < roi.cols; x++){
                if(roi.at<cv::Vec3b>(y, x)[0] == 255
                        && roi.at<cv::Vec3b>(y, x)[1] == 0
                        && roi.at<cv::Vec3b>(y, x)[2] == 255){
                    pcl::PointXYZI arr;
                    arr._PointXYZI::x = (float)(100 - y)/50;
                    arr._PointXYZI::y = (float)(75 - x)/50;
                    arr._PointXYZI::z = 0; arr._PointXYZI::intensity = 127 - abs(arr._PointXYZI::y*100);
                    result_path.push_back(arr);
                    if(arr._PointXYZI::x >= 0.0 && arr._PointXYZI::x < 0.25 && left_in < arr._PointXYZI::intensity){
                        left_in = arr._PointXYZI::intensity;
                    }
                }
                if(roi.at<cv::Vec3b>(y, x)[0] == 0
                        && roi.at<cv::Vec3b>(y, x)[2] == 255
                        && roi.at<cv::Vec3b>(y, x)[1] == 255){
                    pcl::PointXYZI arr;
                    arr._PointXYZI::x = (float)(100 - y)/50;
                    arr._PointXYZI::y = (float)(75 - x)/50;
                    arr._PointXYZI::z = 0; arr._PointXYZI::intensity = 127 + abs(arr._PointXYZI::y*100);
                    result_path.push_back(arr);
                    if(arr._PointXYZI::x >= 0.0 && arr._PointXYZI::x < 0.25 && right_in > arr._PointXYZI::intensity){
                        right_in = arr._PointXYZI::intensity;
                    }
                }
            }
        }
    }
    pcl::PointCloud <pcl::PointXYZI> result;
    std::vector<float> x; x.assign(2, 0);
    std::vector<float> y; y.assign(2, 0);
    int count_l = 0; int count_r = 0;

    for(int i = 0; i < result_path.size(); i++){
        if(result_path.at(i)._PointXYZI::intensity == left_in){
            count_l++;
            x.at(0) += result_path.at(i)._PointXYZI::x;
            y.at(0) += result_path.at(i)._PointXYZI::y;
        }
        if(result_path.at(i)._PointXYZI::intensity == right_in){
            count_r++;
            x.at(1) += result_path.at(i)._PointXYZI::x;
            y.at(1) += result_path.at(i)._PointXYZI::y;
        }
    }
    x.at(0) = x.at(0) / count_l;
    y.at(0) = y.at(0) / count_l;
    x.at(1) = x.at(1) / count_r;
    y.at(1) = y.at(1) / count_r;
    pcl::PointXYZI pt;
    pt._PointXYZI::x = (x.at(0)+x.at(1))/2;
    pt._PointXYZI::y = (y.at(0)+y.at(1))/2;
    pt._PointXYZI::z = 0; pt._PointXYZI::intensity = 255;
    result.push_back(pt);
    result_path.push_back(pt);


//    std::vector<float> x; x.assign(8, 0);
//    std::vector<float> y; y.assign(8, 0);
//    std::vector<float> count; count.assign(8, 0);

//    for(int i = 0; i < result_path.size(); i++){
//        if(result_path.at(i).x >= 0 && result_path.at(i).x < 0.25){
//            if(result_path.at(i)._PointXYZI::intensity == 0){
//                x.at(0) += result_path.at(i)._PointXYZI::x;
//                y.at(0) += result_path.at(i)._PointXYZI::y;
//                count.at(0)++;
//            }
//            else{
//                x.at(1) += result_path.at(i)._PointXYZI::x;
//                y.at(1) += result_path.at(i)._PointXYZI::y;
//                count.at(1)++;
//            }
//        }

//        if(result_path.at(i).x >= 0.25 && result_path.at(i).x < 0.5){
//            if(result_path.at(i)._PointXYZI::intensity == 0){
//                x.at(2) += result_path.at(i)._PointXYZI::x;
//                y.at(2) += result_path.at(i)._PointXYZI::y;
//                count.at(2)++;
//            }
//            else{
//                x.at(3) += result_path.at(i)._PointXYZI::x;
//                y.at(3) += result_path.at(i)._PointXYZI::y;
//                count.at(3)++;
//            }
//        }

//        if(result_path.at(i).x >= 0.5 && result_path.at(i).x < 0.75){
//            if(result_path.at(i)._PointXYZI::intensity == 0){
//                x.at(4) += result_path.at(i)._PointXYZI::x;
//                y.at(4) += result_path.at(i)._PointXYZI::y;
//                count.at(4)++;
//            }
//            else{
//                x.at(5) += result_path.at(i)._PointXYZI::x;
//                y.at(5) += result_path.at(i)._PointXYZI::y;
//                count.at(5)++;
//            }
//        }

//        if(result_path.at(i).x >= 0.75 && result_path.at(i).x < 1){
//            if(result_path.at(i)._PointXYZI::intensity == 0){
//                x.at(6) += result_path.at(i)._PointXYZI::x;
//                y.at(6) += result_path.at(i)._PointXYZI::y;
//                count.at(6)++;
//            }
//            else{
//                x.at(7) += result_path.at(i)._PointXYZI::x;
//                y.at(7) += result_path.at(i)._PointXYZI::y;
//                count.at(7)++;
//            }
//        }
//    }
//    pcl::PointCloud <pcl::PointXYZI> result;
//    pcl::PointXYZI arr1;
////    arr1._PointXYZI::x = ((x.at(0)/count.at(0))+(x.at(1)/count.at(1)))/2;
////    arr1._PointXYZI::y = ((y.at(0)/count.at(0))+(y.at(1)/count.at(1)))/2;
////    arr1._PointXYZI::z = 0; arr1._PointXYZI::intensity = 125;
////    result_path.push_back(arr1);
//    if(count.at(2) != 0 && count.at(3) != 0){
//        arr1._PointXYZI::x = ((x.at(2)/count.at(2))+(x.at(3)/count.at(3)))/2;
//        arr1._PointXYZI::y = ((y.at(2)/count.at(2))+(y.at(3)/count.at(3)))/2;
//        arr1._PointXYZI::z = 0; arr1._PointXYZI::intensity = 125;
//        result.push_back(arr1);
//        result_path.push_back(arr1);
//    }
//    else if(count.at(2) == 0){
//        arr1._PointXYZI::x = (x.at(3)/count.at(3));
//        arr1._PointXYZI::y = (y.at(3)/count.at(3)) - 0.3;
//        arr1._PointXYZI::z = 0; arr1._PointXYZI::intensity = 125;
//        result.push_back(arr1);
//        result_path.push_back(arr1);
//    }
//    else if(count.at(3) == 0){
//        arr1._PointXYZI::x = (x.at(3)/count.at(3));
//        arr1._PointXYZI::y = (y.at(3)/count.at(3)) + 0.3;
//        arr1._PointXYZI::z = 0; arr1._PointXYZI::intensity = 125;
//        result.push_back(arr1);
//        result_path.push_back(arr1);
//    }

    if(result.size() != 0 && mode == true && person == 1){
        theta = -180*atan(result.at(0)._PointXYZI::y/result.at(0)._PointXYZI::x)/3.141592;
    }
    else if(mode == true && person == 0){
        theta = 0;
    }
    else if(mode == false){
        theta = 200;
    }
    std_msgs::Float32 a;
    a.data = theta;
    theta_pub.publish(a);
//    arr1._PointXYZI::x = ((x.at(4)/count.at(4))+(x.at(5)/count.at(5)))/2;
//    arr1._PointXYZI::y = ((y.at(4)/count.at(4))+(y.at(5)/count.at(5)))/2;
//    arr1._PointXYZI::z = 0; arr1._PointXYZI::intensity = 125;
//    result_path.push_back(arr1);

//    arr1._PointXYZI::x = ((x.at(6)/count.at(6))+(x.at(7)/count.at(7)))/2;
//    arr1._PointXYZI::y = ((y.at(6)/count.at(6))+(y.at(7)/count.at(7)))/2;
//    arr1._PointXYZI::z = 0; arr1._PointXYZI::intensity = 125;
//    result_path.push_back(arr1);

    return result_path;
}
