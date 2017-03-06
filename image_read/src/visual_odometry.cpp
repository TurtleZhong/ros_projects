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

    PARAM_READER pd;
    double voxel_grid = pd.getDoubleData("voxel_grid", voxel_grid);


    waitKey(0);

    return 0;
}
