#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "vo_base.h"


using namespace std;
using namespace cv;



int main(int argc, char *argv[])
{

    FileStorage fs("/home/m/ws/src/ros_projects/image_read/src/param.yaml", FileStorage::READ);
    double voxel_grid;
    fs["voxel_grid"] >> voxel_grid;
    cout << "voxel_grid = " << voxel_grid << endl;


//    PARAM_READER pd;
//    double voxel_grid = pd.getDoubleData("voxel_grid", voxel_grid);
//    cout << "voxel_grid = " << voxel_grid << endl;
//    int startIndex = pd.getIntData("start_index", startIndex);
//    int endIndex   = pd.getIntData("end_index", endIndex);


    /*KeyFrames saved*/
//    vector<FRAME> keyframes;
//    int currIndex = startIndex;   /**/
//    FRAME lastFrame = readFrame(currIndex, pd);

//    string detector   = pd.getStringData("detector", detector);
//    string descriptor = pd.getStringData("descriptor", descriptor);
//    cout << "detector = " << detector << endl;

//    CAMERA_INTRINSIC_PARAMETERS camera;
//    computeKeyPointsAndDesp(lastFrame, detector, descriptor);

    //keyframes.push_back(currFrame);

//    for (currIndex = startIndex +1; currIndex < endIndex; currIndex++)
//    {
//        RESULT_OF_PNP resultPnp;
//        cout << "Reading files " << currIndex << endl;
//        FRAME currFrame = readFrame(currIndex, pd);
//        computeKeyPointsAndDesp(currFrame, detector, descriptor);

//        resultPnp = estimateMotion(lastFrame, currFrame, camera);
//        cout << "We got " << resultPnp.inliers << " inliers this time " << endl;
//        lastFrame = currFrame;

//        waitKey(27);
//    }



    waitKey(0);

    return 0;
}
