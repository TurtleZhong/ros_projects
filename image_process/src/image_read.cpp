#include <iostream>
#include <string>
#include <sstream>

#include <ros/ros.h>
#include <ros/package.h>
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
void showHelp();


string rgbFileName = "";
string depthFileName = "";
long rgbNum = 0;
long depthNum = 0;
string param;
bool save_image = false;


int main(int argc, char *argv[])
{

    showHelp();
    ros::init(argc, argv, "image_read");
    ros::NodeHandle nh;

    ros::Subscriber imgSub, depthSub;
    if (argc > 1)
    {
        param = argv[1];
        if(param == "-s")
        {
            save_image = true;
            cout << "saving image sequence....." << endl;
        }
    }
    sleep(1);
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
    //imshow("image_from_kinect", inImage);
    stringstream ss;
    ss << rgbNum;
    rgbFileName = "/home/m/ws/src/ros_projects/dataset/rgb_my/" + ss.str() + ".png";
    if(save_image)
    {
        imwrite(rgbFileName, inImage, compressionQuality);
        //cout << "write " << "  rgb_" << ss.str() << " suscessfully!" << endl;
        rgbNum += 1;
    }
    waitKey(30);
}
/**
 * @brief depthCallback
 * @param depthMsg
 */
void depthCallback(const cv_bridge::CvImage::ConstPtr &depthMsg)
{
    Mat depthImage(depthMsg->image.cols,depthMsg->image.rows, CV_16UC1);
    depthImage = depthMsg->image;
    vector<int> compressionQuality;
    compressionQuality.push_back(CV_IMWRITE_PNG_COMPRESSION);
    stringstream ss;
    ss << depthNum;
    depthFileName = "/home/m/ws/src/ros_projects/dataset/depth_my/" + ss.str() + ".png";
    if(save_image)
    {
        imwrite(depthFileName, depthImage, compressionQuality);
        //cout << "write " << "depth_" << ss.str() << " suscessfully!" << endl;
        depthNum += 1;
    }
    //cout << "the value of save_image is :" << save_image << endl;
    //imshow("depth_from_kinect",depthImage);
    waitKey(30);
}


void showHelp()
{

    cout << "***********Author: Mr.zhong*************" << endl;
    cout << "This node is presented to get rgb and depth image from Kinect V1." << endl;
    cout << "The image is synchronized in time, So you can use rosrun image_read image_read -s to save both image!" << endl;
    cout << "-s " << "save image in the project directory." << endl;
    cout << "if this code do not save image, Please check weather you have open the kinect!" << endl;
    cout << "usually use roslaunch freenect_launch *** to start Kinect V1." << endl;
}
