#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/features2d/features2d.hpp>



using namespace std;
using namespace cv;




int main(int argc, char *argv[])
{
    FileStorage fs("/home/m/ws/src/ros_projects/image_read/src/param.yaml", FileStorage::READ);
    string rgb_path;
    fs["rgb_path"] >> rgb_path;

    /*
     * step1: load the original image turn it into gray;
     */

    Mat srcImage = imread(rgb_path + "rgb_46.png" , 1);
    imshow("original image", srcImage);
    Mat grayImage;
    cvtColor(srcImage, grayImage, CV_BGR2GRAY);

    /*
     * define the detect param: use ORB
     */
    OrbFeatureDetector featureDetector;
    vector<KeyPoint> keyPoints;
    Mat descriptors;

    /*
     * use detect() function to detect keypoints
     */
    featureDetector.detect(grayImage, keyPoints);

    /*
     * conpute the extractor and show the keypoints
     */
    OrbDescriptorExtractor featureEvaluator;
    featureEvaluator.compute(grayImage, keyPoints, descriptors);

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
    Mat testImage = imread(rgb_path + "rgb_54.png" , 1);
    Mat testImage_gray;
    cvtColor(testImage, testImage_gray, CV_BGR2GRAY);

    vector<KeyPoint> testKeyPoints;
    Mat testDescriptors;
    featureDetector.detect(testImage_gray, testKeyPoints);
    featureEvaluator.compute(testImage_gray, testKeyPoints,testDescriptors);

    /*Match the feature*/
    Mat matchIndex(testDescriptors.rows, 2, CV_32SC1);
    Mat matchDistance(testDescriptors.rows, 2, CV_32SC1);
    flannIndex.knnSearch(testDescriptors, matchIndex, matchDistance, 2, flann::SearchParams());

    vector<DMatch> goodMatchs;
    for (int i = 0; i < matchDistance.rows; i++)
    {
        if(matchDistance.at<float>(i,0) < 0.6 * matchDistance.at<float>(i, 1))
        {
            DMatch dmatchs(i, matchIndex.at<int>(i,0), matchDistance.at<float>(i,1));
            goodMatchs.push_back(dmatchs);
        }
    }

    Mat resultImage;
    drawMatches(testImage, testKeyPoints, srcImage, keyPoints, goodMatchs, resultImage);
    imshow("result of Image", resultImage);

    cout << "We got " << goodMatchs.size() << " good Matchs" << endl;


    waitKey(0);

    return 0;
}
