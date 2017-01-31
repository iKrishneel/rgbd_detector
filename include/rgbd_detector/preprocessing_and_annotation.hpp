
#pragma once
#ifndef _PREPROCESSING_AND_ANNOTATION_H_
#define _PREPROCESSING_AND_ANNOTATION_H_

#include <ros/ros.h>
#include <ros/console.h>

#include <opencv2/opencv.hpp>

class RGBDProcessingAndAnnotation {

 private:
   
   
 public:
    RGBDProcessingAndAnnotation();
    void processDepthImage(const cv::Mat, const double, const double);
    cv::Mat depthInterpolateAndInpaint(const cv::Mat);
};


#endif /* _PREPROCESSING_AND_ANNOTATION_H_ */
