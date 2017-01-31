
#include <rgbd_detector/rgbd_detector.hpp>

RGBDDetector::RGBDDetector() {

    this->onInit();
}

void RGBDDetector::onInit() {
    this->subscribe();
    this->pub_cloud_ = this->pnh_.advertise<sensor_msgs::PointCloud2>(
       "/rgbd_detector/output/cloud", 1);
}

void RGBDDetector::subscribe() {
    this->sub_cloud_.subscribe(this->pnh_, "input_cloud", 1);
    this->sub_cinfo_.subscribe(this->pnh_, "input_cinfo", 1);
    this->sync_ = boost::make_shared<message_filters::Synchronizer<
       SyncPolicy> >(100);
    this->sync_->connectInput(this->sub_cloud_, this->sub_cinfo_);
    this->sync_->registerCallback(
       boost::bind(&RGBDDetector::cloudCB, this, _1, _2));
}

void RGBDDetector::unsubscribe() {
   
}

void RGBDDetector::cloudCB(
    const sensor_msgs::PointCloud2::ConstPtr &cloud_msg,
    const sensor_msgs::CameraInfo::ConstPtr &cinfo_msg) {
    PointCloud::Ptr cloud(new PointCloud);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    if (cloud->empty()) {
       ROS_ERROR_ONCE("-Input cloud is empty in callback");
       return;
    }

    PointCloud::Ptr out_cloud(new PointCloud);
    pcl::copyPointCloud<PointT, PointT>(*cloud, *out_cloud);
    this->normalizePointCloud(out_cloud, cloud);
    
    boost::shared_ptr<sensor_msgs::PointCloud2> ros_cloud(
       new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*out_cloud, *ros_cloud);
    ros_cloud->header = cloud_msg->header;
    this->pub_cloud_.publish(*ros_cloud);    
}

void RGBDDetector::normalizePointCloud(
    PointCloud::Ptr &out_cloud, const PointCloud::Ptr in_cloud) {
    if (in_cloud->empty()) {
       ROS_ERROR("[::normalizePointCloud]: EMPTY INPUT");
       return;
    }

    float min_x = FLT_MAX;
    float min_y = FLT_MAX;
    float min_z = FLT_MAX;
    float max_x = 0.0f;
    float max_y = 0.0f;
    float max_z = 0.0f;
    PointT pt;
    for (int i = 0; i < in_cloud->size(); i++) {
       pt = in_cloud->points[i];
       min_x = (pt.x < min_x) ? pt.x : min_x;
       min_y = (pt.y < min_y) ? pt.y : min_y;
       min_z = (pt.z < min_z) ? pt.z : min_z;
       max_x = (pt.x > max_x) ? pt.x : max_x;
       max_y = (pt.y > max_y) ? pt.y : max_y;
       max_z = (pt.z > max_z) ? pt.z : max_z;
    }

    float dist_x = std::abs(max_x) + std::abs(min_x);
    float dist_y = std::abs(max_y) + std::abs(min_y);
    float dist_z = std::abs(max_z) + std::abs(min_z);
    
    for (int i = 0; i < in_cloud->size(); i++) {
       out_cloud->points[i].x += std::abs(min_x);
       out_cloud->points[i].x /= dist_x;
       
       out_cloud->points[i].y += std::abs(min_y);
       out_cloud->points[i].y /= dist_y;
       
       out_cloud->points[i].z += std::abs(min_z);
       out_cloud->points[i].z /= dist_z;
    }

    min_x = FLT_MAX;
    min_y = FLT_MAX;
    min_z = FLT_MAX;
    max_x = 0.0f;
    max_y = 0.0f;
    max_z = 0.0f;
    
    //! test
    for (int i = 0; i < in_cloud->size(); i++) {
       pt = out_cloud->points[i];
       min_x = (pt.x < min_x) ? pt.x : min_x;
       min_y = (pt.y < min_y) ? pt.y : min_y;
       min_z = (pt.z < min_z) ? pt.z : min_z;
       max_x = (pt.x > max_x) ? pt.x : max_x;
       max_y = (pt.y > max_y) ? pt.y : max_y;
       max_z = (pt.z > max_z) ? pt.z : max_z;
    }

    
    std::cout << min_x << " " << max_x  << "\n";
    std::cout << min_y << " " << max_y  << "\n";
    std::cout << min_z << " " << max_z  << "\n\n";
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "rgbd_detector");
    RGBDDetector rdect;
    ros::spin();
    return 0;
}

