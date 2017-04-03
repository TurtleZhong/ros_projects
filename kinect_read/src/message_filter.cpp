#include <iostream>
#include <string>
#include <sstream>

#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <std_msgs/String.h>

#include <cv_bridge/cv_bridge.h>

//message_filters
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <time.h>


using namespace std;
using namespace cv;
using namespace message_filters;

void imageCallback(const cv_bridge::CvImage::ConstPtr &imgMsg);
void depthCallback(const cv_bridge::CvImage::ConstPtr &depthMsg);
void showHelp();


string rgbFileName = "";
string depthFileName = "";
long Num = 0;
string param;
bool save_image = false;
clock_t start,finish;




void callback(const cv_bridge::CvImage::ConstPtr &imgMsg,const cv_bridge::CvImage::ConstPtr &depthMsg)
{

    start = clock();

    cout << "suscessfully" << endl;
    Mat inImage;
    inImage = imgMsg->image;

    Mat depthImage(depthMsg->image.cols,depthMsg->image.rows, CV_16UC1);
    depthImage = depthMsg->image;

    vector<int> CompressionQuality;
    CompressionQuality.push_back(CV_IMWRITE_PNG_COMPRESSION);
    stringstream ss;
    ss << Num;
    rgbFileName = "/home/m/ws/src/ros_projects/kinect_read/save_data/rgb/" + ss.str() + ".png";
    depthFileName = "/home/m/ws/src/ros_projects/kinect_read/save_data/depth/" + ss.str() + ".png";
    if(save_image)
    {
        imwrite(rgbFileName, inImage);
        imwrite(depthFileName, depthImage);
        cout << "write " << "  rgb and depth " << ss.str() << " suscessfully!" << endl;
        Num += 1;
    }

    finish = clock();
    double time = (double)(finish - start)/ CLOCKS_PER_SEC;
    cout << "use time = " << time << "s" <<  endl;

}

int main(int argc, char *argv[])
{

    showHelp();
    ros::init(argc, argv, "kinect_read");
    ros::NodeHandle nh;

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
    
    ros::spin();
    return 0;
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
