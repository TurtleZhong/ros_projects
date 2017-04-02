#include <iostream>
#include <string>
#include <sstream>

#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

//message_filters
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace std;
using namespace cv;
using namespace message_filters;

void imageCallback(const cv_bridge::CvImage::ConstPtr &imgMsg);
void depthCallback(const cv_bridge::CvImage::ConstPtr &depthMsg);
void showHelp();


string rgbFileName = "";
string depthFileName = "";
long rgbNum = 0;
long depthNum = 0;
string param;
bool save_image = false;


void callback(const cv_bridge::CvImage::ConstPtr &imgMsg,const cv_bridge::CvImage::ConstPtr &depthMsg)
{
    cout << "suscessfully" << endl;
    Mat inImage;
    inImage = imgMsg->image;

    vector<int> rgbCompressionQuality;
    rgbCompressionQuality.push_back(CV_IMWRITE_PNG_COMPRESSION);
    //imshow("image_from_kinect", inImage);
    stringstream rgb_ss;
    rgb_ss << rgbNum;
    rgbFileName = "/home/m/ws/src/ros_projects/kinect_read/save_data/rgb/" + rgb_ss.str() + ".png";
    if(save_image)
    {
        imwrite(rgbFileName, inImage, rgbCompressionQuality);
        cout << "write " << "  rgb_" << rgb_ss.str() << " suscessfully!" << endl;
        rgbNum += 1;
    }

    Mat depthImage(depthMsg->image.cols,depthMsg->image.rows, CV_16UC1);
    depthImage = depthMsg->image;
    vector<int> depthCompressionQuality;
    depthCompressionQuality.push_back(CV_IMWRITE_PNG_COMPRESSION);
    stringstream depth_ss;
    depth_ss << depthNum;
    depthFileName = "/home/m/ws/src/ros_projects/kinect_read/save_data/depth/" + depth_ss.str() + ".png";
    if(save_image)
    {
        imwrite(depthFileName, depthImage, depthCompressionQuality);
        cout << "write " << "depth_" << depth_ss.str() << " suscessfully!" << endl;
        depthNum += 1;
    }

}

int main(int argc, char *argv[])
{

    showHelp();
    ros::init(argc, argv, "kinect_read");
    ros::NodeHandle nh;

    //ros::Subscriber imgSub, depthSub;

    message_filters::Subscriber<cv_bridge::CvImage> imgSub(nh, "/camera/rgb/image_color", 10);
    message_filters::Subscriber<cv_bridge::CvImage> depthSub(nh, "/camera/depth_registered/sw_registered/image_rect_raw", 10);
    if (argc > 1)
    {
        param = argv[1];
        if(param == "-s")
        {
            save_image = true;
            cout << "saving image sequence....." << endl;
        }
    }
    //sleep(1);

    typedef message_filters::sync_policies::ApproximateTime<cv_bridge::CvImage, cv_bridge::CvImage> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), imgSub, depthSub);
    sync.registerCallback(boost::bind(&callback, _1, _2));
    
    //subscribe the image from the Kinect
    //imgSub   = nh.subscribe("/camera/rgb/image_color", 5, imageCallback);
    //depthSub = nh.subscribe("/camera/depth_registered/image_raw",5,depthCallback);
    //depthSub = nh.subscribe("/camera/depth_registered/sw_registered/image_rect_raw",5,depthCallback);

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
    rgbFileName = "/home/m/ws/src/ros_projects/kinect_read/save_data/rgb/" + ss.str() + ".png";
    if(save_image)
    {
        imwrite(rgbFileName, inImage, compressionQuality);
        cout << "write " << "  rgb_" << ss.str() << " suscessfully!" << endl;
        rgbNum += 1;
    }
    //waitKey(30);
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
    depthFileName = "/home/m/ws/src/ros_projects/kinect_read/save_data/depth/" + ss.str() + ".png";
    if(save_image)
    {
        imwrite(depthFileName, depthImage, compressionQuality);
        cout << "write " << "depth_" << ss.str() << " suscessfully!" << endl;
        depthNum += 1;
    }
    //cout << "the value of save_image is :" << save_image << endl;
    //imshow("depth_from_kinect",depthImage);
    //waitKey(30);
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
