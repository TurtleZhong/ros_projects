﻿#ifndef VO_BASE_H
#define VO_BASE_H


# pragma once

// Include Libraries
// C++ standerd libraries
#include <iostream>
#include <fstream>
#include <vector>
#include <map>
using namespace std;

// Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
using namespace cv;

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>


#define PARAM_FILE_PATH "/home/m/ws/src/ros_projects/image_read/src/param.yaml"


// 类型定义
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// 相机内参结构
class CAMERA_INTRINSIC_PARAMETERS
{
public:
    double cx, cy, fx, fy, scale;
    CAMERA_INTRINSIC_PARAMETERS()
    {
        FileStorage fs("/home/m/ws/src/ros_projects/image_read/src/param.yaml", FileStorage::READ);
        fs["camera.cx"] >> this->cx;
        fs["camera.cy"] >> this->cy;
        fs["camera.fx"] >> this->fx;
        fs["camera.fy"] >> this->fy;
        fs["camera.scale"] >> this->scale;
        fs.release();
    }
    bool getCameraParam();
    cv::Mat creatCameraMatrix(); // This function is wrong cuz could not return an ARRAY!

};

// 帧结构
struct FRAME
{
    int frameID;
    cv::Mat rgb, depth; //该帧对应的彩色图与深度图
    cv::Mat desp;       //特征描述子, 这里默认使用ORB
    vector<cv::KeyPoint> kp; //关键点
};

// PnP 结果
struct RESULT_OF_PNP
{
    cv::Mat rvec, tvec;
    int inliers;
};



/*Useful function*/
// computeKeyPointsAndDesp 同时提取关键点与特征描述子

void computeKeyPointsAndDesp( FRAME& frame, string detector, string descriptor );

// estimateMotion 计算两个帧之间的运动
// 输入：帧1和帧2, 相机内参
RESULT_OF_PNP estimateMotion( FRAME& frame1, FRAME& frame2, CAMERA_INTRINSIC_PARAMETERS& camera );

// cvMat2Eigen
Eigen::Isometry3d cvMat2Eigen( cv::Mat& rvec, cv::Mat& tvec );

class PARAM_READER
{
public:
    PARAM_READER(string filename = PARAM_FILE_PATH)
    {
        ifstream fin( filename.c_str() );
        if (!fin)
        {
            cerr<<"parameter file does not exist."<<endl;
            return;
        }
    }
    double getDoubleData(string keyName, double& key)
    {
        FileStorage fs(PARAM_FILE_PATH, FileStorage::READ);
        fs[keyName] >> key;
        fs.release();
        return key;
    }
    bool getIntData(string keyName, int& key)
    {
        FileStorage fs(PARAM_FILE_PATH, FileStorage::READ);
        fs[keyName] >> key;
        fs.release();
        return true;
    }
    bool getStringData(string keyName, string& key)
    {
        FileStorage fs(PARAM_FILE_PATH, FileStorage::READ);
        fs[keyName] >> key;
        fs.release();
        return true;
    }
};










#endif // VO_BASE_H
