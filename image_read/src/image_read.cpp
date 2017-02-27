#include <iostream>
#include <string>
#include <sstream>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace std;
using namespace cv;

void imageCallback(const cv_bridge::CvImage::ConstPtr &imgMsg);
void depthCallback(const cv_bridge::CvImage::ConstPtr &depthMsg);

string rgbFileName = "";
string depthFileName = "";
long rgbNum = 0;
long depthNum = 0;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "image_read");
    ros::NodeHandle nh;

    ros::Subscriber imgSub, depthSub;

    //subscribe the image from the Kinect
    imgSub   = nh.subscribe("/camera/rgb/image_color", 5, imageCallback);
    depthSub = nh.subscribe("/camera/depth_registered/image_raw",5,depthCallback);

    ros::spin();
    return 0;
}
/**
 * @brief imageCallback
 * @param imgMsg
 */
void imageCallback(const cv_bridge::CvImage::ConstPtr &imgMsg)
{
    Mat inImage;
    inImage = imgMsg->image;

    vector<int> compressionQuality;
    compressionQuality.push_back(CV_IMWRITE_PNG_COMPRESSION);
    //cout << "The encoding of image is: " << imgMsg->encoding << endl;
    //imshow("image_from_kinect", inImage);
    stringstream ss;
    ss << rgbNum;
    rgbFileName = "/home/m/ws/src/ros_projects/rgb/rgb_" + ss.str() + ".png";
    imwrite(rgbFileName, inImage, compressionQuality);
    cout << "write " << "  rgb_" << ss.str() << " suscessfully!" << endl;
    waitKey(30);
    rgbNum += 1;
}
/**
 * @brief depthCallback
 * @param depthMsg
 */
void depthCallback(const cv_bridge::CvImage::ConstPtr &depthMsg)
{
    Mat depthImage(depthMsg->image.cols,depthMsg->image.rows, CV_16UC1);
    depthImage = depthMsg->image;
    vector<int> pgmFormat;
    pgmFormat.push_back(CV_IMWRITE_PXM_BINARY);
    stringstream ss;
    ss << depthNum;
    depthFileName = "/home/m/ws/src/ros_projects/depth/depth_" + ss.str() + ".pgm";
    imwrite(depthFileName, depthImage, pgmFormat);

    cout << "write " << "depth_" << ss.str() << " suscessfully!" << endl;
    //imshow("depth_from_kinect",depthImage);
    waitKey(30);
    depthNum += 1;
}
