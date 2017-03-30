﻿#include "vo_base.h"

PointCloud::Ptr image2PointCloud( cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera )
{
    PointCloud::Ptr cloud ( new PointCloud );

    for (int m = 0; m < depth.rows; m+=2)
        for (int n=0; n < depth.cols; n+=2)
        {
            // 获取深度图中(m,n)处的值
            ushort d = depth.ptr<ushort>(m)[n];
            // d 可能没有值，若如此，跳过此点
            if (d == 0)
                continue;
            // d 存在值，则向点云增加一个点
            PointT p;

            // 计算这个点的空间坐标

            p.z = double(d) / camera.scale;
            p.x = (n - camera.cx) * p.z / camera.fx;
            p.y = (m - camera.cy) * p.z / camera.fy;

            // 从rgb图像中获取它的颜色
            // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
            p.b = rgb.ptr<uchar>(m)[n*3];
            p.g = rgb.ptr<uchar>(m)[n*3+1];
            p.r = rgb.ptr<uchar>(m)[n*3+2];

            // 把p加入到点云中
            cloud->points.push_back( p );
        }
    // 设置并保存点云
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;

    return cloud;
}

cv::Point3f point2dTo3d( cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera )
{
    cv::Point3f p; // 3D 点
    p.z = double( point.z ) / camera.scale;
    p.x = ( point.x - camera.cx) * p.z / camera.fx;
    p.y = ( point.y - camera.cy) * p.z / camera.fy;
    return p;
}

FRAME readFrame( int index, PARAM_READER& pd )
{
    FRAME f;
    string rgbDir   =   pd.getStringData("rgb_dir");
    string depthDir =   pd.getStringData("depth_dir");

    string rgbExt   =   ".png";
    string depthExt =   ".png";

    stringstream ss;
    ss<<rgbDir<<index<<rgbExt;
    string filename;
    ss>>filename;
    f.rgb = cv::imread( filename );

    ss.clear();
    filename.clear();
    ss<<depthDir<<index<<depthExt;
    ss>>filename;

    f.depth = cv::imread( filename, -1 );
    f.frameID = index;
    return f;
}


// computeKeyPointsAndDesp --->extrat the keyPoints and desp
void computeKeyPointsAndDesp( FRAME& frame, string detector, string descriptor )
{

    cv::Ptr<cv::FeatureDetector> _detector;
    cv::Ptr<cv::DescriptorExtractor> _descriptor;

    _detector = cv::FeatureDetector::create( detector.c_str() );
    _descriptor = cv::DescriptorExtractor::create( descriptor.c_str() );

    if (!_detector || !_descriptor)
    {
        cerr<<"Unknown detector or discriptor type !"<<detector<<","<<descriptor<<endl;
        return;
    }

    _detector->detect( frame.rgb, frame.kp );
    _descriptor->compute( frame.rgb, frame.kp, frame.desp );
//    Ptr<ORB> detect = ORB::create();
//    detect->detect(frame.rgb, frame.kp);
//    detect->compute(frame.rgb, frame.kp, frame.desp);

    return;
}


/*
 *  estimateMotion ---> Compute the motion between two frames!
 *  Input: frame1 and frame2
 *  output: rvec and tvec and inliers
 */
RESULT_OF_PNP estimateMotion( FRAME& frame1, FRAME& frame2, CAMERA_INTRINSIC_PARAMETERS& camera, string matchType)
{
    static PARAM_READER pd;

    /* PARAM use BF match type */
    vector< cv::DMatch > matches;
    cv::BFMatcher matcher;
    RESULT_OF_PNP result;
    vector< cv::DMatch > goodMatches;
    double minDis = 9999;
    double good_match_threshold = pd.getDoubleData("good_match_threshold");

    /*PARAM use KNN match type*/

    /* FLANN */
    flann::Index flannIndex(frame2.desp, flann::LshIndexParams(12, 20, 2), cvflann::FLANN_DIST_HAMMING);
    vector<vector<DMatch>> matchesKnn;
    cv::BFMatcher knnMatcher;



    if (matchType == "BF")
    {
        matcher.match( frame1.desp, frame2.desp, matches );
        for ( size_t i=0; i<matches.size(); i++ )
        {
            if ( matches[i].distance < minDis )
                minDis = matches[i].distance;
        }

        if ( minDis < 10 )
            minDis = 10;

        for ( size_t i=0; i<matches.size(); i++ )
        {
            if (matches[i].distance < good_match_threshold*minDis)
                goodMatches.push_back( matches[i] );
        }

    }
    else if((matchType == "KNN") || (matchType == "knn"))
    {
        cout << "using knn to match the feature" << endl;
        /*Match the feature*/
                Mat matchIndex(frame1.desp.rows, 2, CV_32SC1);
                Mat matchDistance(frame1.desp.rows, 2, CV_32SC1);
                flannIndex.knnSearch(frame1.desp, matchIndex, matchDistance, 2, flann::SearchParams());

                //vector<DMatch> goodMatches;
                for (int i = 0; i < matchDistance.rows; i++)
                {
                    if(matchDistance.at<float>(i,0) < 0.7 * matchDistance.at<float>(i, 1))
                    {
                        DMatch dmatchs(i, matchIndex.at<int>(i,0), matchDistance.at<float>(i,0));
                        goodMatches.push_back(dmatchs);
                    }
                }
        cout << "We got " << goodMatches.size() << " good Matchs" << endl;
//        knnMatcher.knnMatch(frame1.desp, frame2.desp, matchesKnn, 2);
//        for (int i = 0; i < matchesKnn.size(); i++)
//        {
//            DMatch& bestMatch = matchesKnn[i][0];
//            DMatch& betterMatch = matchesKnn[i][1];
//            float distanceRatio = bestMatch.distance / betterMatch.distance;
//            if(distanceRatio < 0.6)
//            {
//                goodMatches.push_back(bestMatch);
//            }
//        }
    }

    Mat resultImage;
    drawMatches(frame1.rgb, frame1.kp, frame2.rgb, frame2.kp, goodMatches, resultImage);
    imshow("result of Image", resultImage);
    cout << "We got " << goodMatches.size() << " good Matchs" << endl;


    if (goodMatches.size() <= 5)
    {
        result.inliers = -1;
        return result;
    }
    // 第一个帧的三维点
    vector<cv::Point3f> pts_obj;
    // 第二个帧的图像点
    vector< cv::Point2f > pts_img;

    // 相机内参
    for (size_t i=0; i<goodMatches.size(); i++)
    {
        // query 是第一个, train 是第二个
        cv::Point2f p = frame1.kp[goodMatches[i].queryIdx].pt;
        // 获取d是要小心！x是向右的，y是向下的，所以y才是行，x是列！
        ushort d = frame1.depth.ptr<ushort>( int(p.y) )[ int(p.x) ];
        if (d == 0)
            continue;
        pts_img.push_back( cv::Point2f( frame2.kp[goodMatches[i].trainIdx].pt ) );

        // 将(u,v,d)转成(x,y,z)
        cv::Point3f pt ( p.x, p.y, d );
        cv::Point3f pd = point2dTo3d( pt, camera );
        pts_obj.push_back( pd );
    }

    if (pts_obj.size() ==0 || pts_img.size()==0)
    {
        result.inliers = -1;
        return result;
    }

    double camera_matrix_data[3][3] = {
        {camera.fx, 0, camera.cx},
        {0, camera.fy, camera.cy},
        {0, 0, 1}
    };

    // 构建相机矩阵
    cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );
    cv::Mat rvec, tvec, inliers;
    // 求解pnp
    cv::solvePnPRansac( pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 1.0, 100, inliers );

    result.rvec = rvec;
    result.tvec = tvec;
    result.inliers = inliers.rows;
    cout << "inliers = " << result.inliers << endl;
    cout << "rvec = " << rvec << endl;
    cout << "tvec = " << tvec << endl;
    return result;
}


// cvMat2Eigen
Eigen::Isometry3d cvMat2Eigen( cv::Mat& rvec, cv::Mat& tvec )
{
    cv::Mat R;
    cv::Rodrigues( rvec, R );
    Eigen::Matrix3d r;
    for ( int i=0; i<3; i++ )
        for ( int j=0; j<3; j++ )
            r(i,j) = R.at<double>(i,j);

    // 将平移向量和旋转矩阵转换成变换矩阵
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    Eigen::AngleAxisd angle(r);
    T = angle;
    T(0,3) = tvec.at<double>(0,0);
    T(1,3) = tvec.at<double>(1,0);
    T(2,3) = tvec.at<double>(2,0);
    return T;

}
