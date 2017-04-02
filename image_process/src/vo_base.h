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
using namespace cv;

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>


#define PARAM_FILE_PATH "/home/m/ws/src/ros_projects/image_process/src/param.xml"


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
        FileStorage fs("/home/m/ws/src/ros_projects/image_process/src/param.xml", FileStorage::READ);
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

// 函数接口
// image2PonitCloud 将rgb图转换为点云
PointCloud::Ptr image2PointCloud( cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera );



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


//the following are UBUNTU/LINUX ONLY terminal color
#define RESET "\033[0m"
#define BLACK "\033[30m" /* Black */
#define RED "\033[31m" /* Red */
#define GREEN "\033[32m" /* Green */
#define YELLOW "\033[33m" /* Yellow */
#define BLUE "\033[34m" /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m" /* Cyan */
#define WHITE "\033[37m" /* White */
#define BOLDBLACK "\033[1m\033[30m" /* Bold Black */
#define BOLDRED "\033[1m\033[31m" /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m" /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m" /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m" /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m" /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m" /* Bold White */


#endif // VO_BASE_H
