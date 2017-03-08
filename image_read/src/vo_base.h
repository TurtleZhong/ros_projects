#ifndef VO_BASE_H
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
#include <opencv2/opencv.hpp>
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


#define PARAM_FILE_PATH "/home/m/ws/src/ros_projects/image_read/src/param.xml"


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
        FileStorage fs("/home/m/ws/src/ros_projects/image_read/src/param.xml", FileStorage::READ);
        fs["camera_cx"] >> this->cx;
        fs["camera_cy"] >> this->cy;
        fs["camera_fx"] >> this->fx;
        fs["camera_fy"] >> this->fy;
        fs["camera_scale"] >> this->scale;
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

// point2dTo3d 将单个点从图像坐标转换为空间坐标
// input: 3维点Point3f (u,v,d)
cv::Point3f point2dTo3d( cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera );



// computeKeyPointsAndDesp 同时提取关键点与特征描述子

void computeKeyPointsAndDesp( FRAME& frame, string detector, string descriptor );

// estimateMotion 计算两个帧之间的运动
// 输入：帧1和帧2, 相机内参
RESULT_OF_PNP estimateMotion(FRAME& frame1, FRAME& frame2, CAMERA_INTRINSIC_PARAMETERS& camera , string matchType = "BF");

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
        fin.close();
    }
    double getDoubleData(string keyName)
    {
        double key;
        FileStorage fs(PARAM_FILE_PATH, FileStorage::READ);
        fs[keyName] >> key;
        fs.release();
        return key;
    }
    int getIntData(string keyName)
    {

        int key;
        FileStorage fs(PARAM_FILE_PATH, FileStorage::READ);
        key = (int)fs[keyName];
        fs.release();
        return key;
    }
    string getStringData(string keyName)
    {
        string key;
        FileStorage fs(PARAM_FILE_PATH, FileStorage::READ);
        fs[keyName] >> key;
        fs.release();
        return key;
    }
};


//Read a frame from the image path
FRAME readFrame( int index, PARAM_READER& pd );









#endif // VO_BASE_H
