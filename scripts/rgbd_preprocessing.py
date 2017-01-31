#!/usr/bin/env python

import rospy
import math
import sys
import os

import numpy as np
import cv2 as cv
import matplotlib.pylab as plt
import message_filters as MF

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def decode_ros_image(img_msg, encoding = 'bgr8'):
    bridge = CvBridge()
    cv_img = None
    try:
        cv_img = bridge.imgmsg_to_cv2(img_msg, encoding)
    except Exception as e:
        print e
    return cv_img


def depth_filling_and_interpolation(im_depth):
    if im_depth is None:
        rospy.logerr("[::depth_filling_and_interpolation]: EMPTY")
        return False

    # convert and scale depth
    #max_val = np.max(im_depth)
    im_depth *= 255
    im_depth = im_depth.astype(np.uint8)

    scale_factor = 0.5
    depth_reduce = cv.resize(im_depth, (int(im_depth.shape[1] * scale_factor), 
                                        int(im_depth.shape[0] * scale_factor)))
    
    im_mask = np.zeros((depth_reduce.shape), np.uint8)
    im_mask[depth_reduce == 0] = 255
    depth_image = cv.inpaint(depth_reduce, im_mask, 5, cv.INPAINT_TELEA)

    depth_image = cv.resize(depth_image, (im_depth.shape[1], im_depth.shape[0]))
    #depth_image = cv.applyColorMap(depth_image, cv.COLORMAP_JET)

    #cv.imshow("depth_redu", im_mask)
    #cv.imshow("painted", depth_image)
    return depth_image

def grid_overlay(image, wsize = 16):
    if not image.any():
        return
        
    dist_diff = 0.1

    img = np.zeros((image.shape[0], image.shape[1], 3), dtype=np.uint8)
    for j in xrange(0, image.shape[0], wsize):
        for i in xrange(0, image.shape[1], wsize):
            
            roi = image[j : j+wsize, i : i + wsize].copy()
            min_dist = np.min(roi)
            max_dist = np.max(roi)
            
            count = 0
            if max_dist - min_dist > dist_diff:
                temp = []
                for y in xrange(0, roi.shape[0]):
                    for x in xrange(0, roi.shape[1]):
                        if roi[x,y] - min_dist > dist_diff:
                            roi[x, y] = 0
                        else:
                            count = count + 1
            else:
                count = np.array(roi)
                count = count.shape[0] * count.shape[1]

            sum = 0.0
            sum = np.sum(roi)
            sum /= count
            img[j:j+wsize, i:i+wsize, :] = sum  * 255
            
    img = cv.applyColorMap(img, cv.COLORMAP_HSV)    
    cv.imshow("plot", img)

def callback(image_msg, depth_msg):
    im_rgb = decode_ros_image(image_msg)
    im_dep = decode_ros_image(depth_msg, '32FC1')

    ## normalize by the max_distance
    im_dep /= np.max(im_dep)
    
    #im_dep = depth_filling_and_interpolation(im_dep)
    grid_overlay(im_dep, 8)
    
    cv.imshow("image", im_dep)
    cv.waitKey(3)
    
    
def on_init():
    topic_rgb = '/camera/rgb/image_rect_color'
    topic_depth = '/camera/depth/image_rect_raw'
    
    rgb_sub = MF.Subscriber(topic_rgb, Image)
    depth_sub = MF.Subscriber(topic_depth, Image)

    ats = MF.ApproximateTimeSynchronizer([rgb_sub, depth_sub], 10, 10)
    ats.registerCallback(callback)
    
def main(argv):
    rospy.init_node('rgbd_grid', anonymous = True)
    on_init()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
