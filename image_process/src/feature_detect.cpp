#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;

using namespace cv;



int main(int argc, char *argv[])
{

//    FileStorage fs("/home/m/ws/src/ros_projects/image_read/src/param.yml", FileStorage::READ);
//    string rgb_path;
//    fs["rgb_path"] >> rgb_path;

    FileStorage fs("/home/m/ws/src/ros_projects/image_read/src/param_test.xml" ,FileStorage::WRITE);
    string rgb_path = "/home/m/ws/src/ros_projects/dataset/rgb/";
    int camera_cx = 5;
    double num = 444.44;
    fs << "rgb_path" << rgb_path;
    fs << "camera_cx" << camera_cx;
    fs << "num" << num;
    fs.release();

    FileStorage fsr("/home/m/ws/src/ros_projects/image_read/src/param_test.xml" ,FileStorage::READ);
    double camera;
    fsr["num"] >> camera;
    cout << "camera = " << camera << endl;


    /*
     * step1: load the original image turn it into gray;
     */

    Mat srcImage = imread(rgb_path + "46.png" , 1);
    imshow("original image", srcImage);
    Mat grayImage;
    cvtColor(srcImage, grayImage, CV_BGR2GRAY);

    /*
     * define the detect param: use ORB
     */

    //Ptr<ORB> detect = ORB::create();

    OrbFeatureDetector featureDetector;
    vector<KeyPoint> keyPoints;
    Mat descriptors;

    /*
     * use detect() function to detect keypoints
     */
    //detect->detect(grayImage, keyPoints);
    featureDetector.detect(grayImage, keyPoints);

    /*
     * conpute the extractor and show the keypoints
     */
    OrbDescriptorExtractor featureEvaluator;
    featureEvaluator.compute(grayImage, keyPoints, descriptors);
    //detect->compute(grayImage, keyPoints, descriptors);

    Mat pointsImage;
    drawKeypoints(srcImage, keyPoints, pointsImage);
    imshow("keyPoints Image", pointsImage);
    cout << "Totally we got "<< keyPoints.size() << " key points." << endl;

    /*
     * FLANN
     */
    flann::Index flannIndex(descriptors, flann::LshIndexParams(12, 20, 2), cvflann::FLANN_DIST_HAMMING);

    /*
     * Actually this is a sequence of image but this is just an example.
     */
    Mat testImage = imread(rgb_path + "48.png" , 1);
    Mat testImage_gray;
    cvtColor(testImage, testImage_gray, CV_BGR2GRAY);

    vector<KeyPoint> testKeyPoints;
    Mat testDescriptors;
    featureDetector.detect(testImage_gray, testKeyPoints);
    featureEvaluator.compute(testImage_gray, testKeyPoints,testDescriptors);
    //detect->detect(testImage_gray, testKeyPoints);
    //detect->compute(testImage_gray, testKeyPoints,testDescriptors);

    /*Match the feature*/
    Mat matchIndex(testDescriptors.rows, 2, CV_32SC1);
    Mat matchDistance(testDescriptors.rows, 2, CV_32SC1);
    flannIndex.knnSearch(testDescriptors, matchIndex, matchDistance, 2, flann::SearchParams());

    vector<DMatch> goodMatches;
    for (int i = 0; i < matchDistance.rows; i++)
    {
        if(matchDistance.at<float>(i,0) < 0.6 * matchDistance.at<float>(i, 1))
        {
            DMatch dmatchs(i, matchIndex.at<int>(i,0), matchDistance.at<float>(i,1));
            goodMatches.push_back(dmatchs);
        }
    }
    cout << "We got " << goodMatches.size() << " good Matchs" << endl;

//    vector<DMatch> goodMatches;
//    double minDis = 9999;
//    double good_match_threshold = 10;
//    for ( size_t i=0; i<goodMatches.size(); i++ )
//    {
//        if ( goodMatches[i].distance < minDis )
//            minDis = goodMatches[i].distance;
//    }

//    if ( minDis < 10 )
//        minDis = 10;

//    for ( size_t i=0; i<goodMatches.size(); i++ )
//    {
//        if (goodMatches[i].distance < good_match_threshold*minDis)
//            goodMatches.push_back( goodMatches[i] );
//    }

    Mat resultImage;
    drawMatches(testImage, testKeyPoints, srcImage, keyPoints, goodMatches, resultImage);
    imshow("result of Image", resultImage);

    cout << "We got " << goodMatches.size() << " good Matchs" << endl;



    waitKey(0);

    return 0;
}
