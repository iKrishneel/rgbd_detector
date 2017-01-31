
#pragma once
#ifndef _RGBD_DETECTOR_H_
#define _RGBD_DETECTOR_H_

#include <ros/ros.h>
#include <ros/console.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types_conversion.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/distances.h>
#include <pcl/common/centroid.h>

#include <sensor_msgs/CameraInfo.h>

#include <opencv2/opencv.hpp>
#include <boost/foreach.hpp>

class RGBDDetector {

 private:
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::Normal NormalT;
    typedef pcl::PointCloud<PointT> PointCloud;
   
    typedef  message_filters::sync_policies::ApproximateTime<
    sensor_msgs::PointCloud2, sensor_msgs::CameraInfo> SyncPolicy;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_cloud_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_cinfo_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;

 protected:
    ros::NodeHandle pnh_;
    void onInit();
    void subscribe();
    void unsubscribe();

    ros::Publisher pub_cloud_;
   
 public:
    RGBDDetector();
    void cloudCB(const sensor_msgs::PointCloud2::ConstPtr &,
                 const sensor_msgs::CameraInfo::ConstPtr &);
    void normalizePointCloud(PointCloud::Ptr &, const PointCloud::Ptr);
   
};


#endif /* _RGBD_DETECTOR_H_ */
