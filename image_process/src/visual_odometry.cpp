#include <iostream>
#include <opencv2/opencv.hpp>


#include "vo_base.h"


using namespace std;
using namespace cv;



int main(int argc, char *argv[])
{


    PARAM_READER pd;
    int startIndex = pd.getIntData("start_index");
    int endIndex   = pd.getIntData("end_index");

/*KeyFrames saved*/
    vector<FRAME> keyframes;
    int currIndex = startIndex;   /**/
    FRAME lastFrame = readFrame(currIndex, pd);

    string detector   = pd.getStringData("detector");
    string descriptor = pd.getStringData("descriptor");
//    cout << "detector = " << detector << endl;
//    cout << "descriptor = " << descriptor << endl;

    CAMERA_INTRINSIC_PARAMETERS camera;
    computeKeyPointsAndDesp(lastFrame, detector, descriptor);

    keyframes.push_back(lastFrame);

    for (currIndex = startIndex +1; currIndex < endIndex; currIndex++)
    {
        RESULT_OF_PNP resultPnp;
        cout << "Reading files " << currIndex << endl;
        FRAME currFrame = readFrame(currIndex, pd);
        computeKeyPointsAndDesp(currFrame, detector, descriptor);

        resultPnp = estimateMotion(lastFrame, currFrame, camera, "knn");
        cout << "We got " << resultPnp.inliers << " inliers this time " << endl;
        lastFrame = currFrame;

        waitKey(27);
    }

    waitKey(0);

    return 0;
}
