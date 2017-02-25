
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

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "image_read");
    ros::NodeHandle nh;

    ros::Subscriber imgSub, depthSub;

    //subscribe the image from the Kinect
    imgSub   = nh.subscribe("/camera/rgb/image_color", 5, imageCallback);
    depthSub = nh.subscribe("/camera/depth/image_raw",5,depthCallback);

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
    //cout << "The encoding of image is: " << imgMsg->encoding << endl;

    imshow("image_from_kinect", inImage);
    waitKey(30);
}
/**
 * @brief depthCallback
 * @param depthMsg
 */
void depthCallback(const cv_bridge::CvImage::ConstPtr &depthMsg)
{
    Mat depthImage(depthMsg->image.cols,depthMsg->image.rows, CV_32FC1);
    depthImage = depthMsg->image;
    imshow("depth_from_kinect",depthImage);
    waitKey(30);
}
