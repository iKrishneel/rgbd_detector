
#include <rgbd_detector/preprocessing_and_annotation.hpp>

RGBDProcessingAndAnnotation::RGBDProcessingAndAnnotation() {
   
}

void RGBDProcessingAndAnnotation::processDepthImage(
    const cv::Mat im_depth, const double lower, const double upper) {
    if (im_depth.empty()) {
      ROS_ERROR("[::processDepthImage]: EMPTY INPUT");
      return;
    }

    //! 
    
    cv::Mat depth_norm;
    cv::normalize(im_depth, depth_norm, lower, upper, cv::NORM_MINMAX);

    
    
}

cv::Mat RGBDProcessingAndAnnotation::depthInterpolateAndInpaint(
    const cv::Mat im_depth) {
    if (im_depth.empty()) {
       ROS_ERROR("[::depthInterpolateAndInpaint]: EMPTY INPUT");
       return cv::Mat();
    }
    double scale_factor = 0.2;
    cv::Mat depth_reduce;
    cv::resize(im_depth, depth_reduce, cv::Size(), scale_factor, scale_factor);

    // cv::inpaint(depth_reduce, )
    
}

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "preprocessing_and_annotation");
    RGBDProcessingAndAnnotation rpaa;
    ros::spin();
    return 0;
}

