#include "test.h"
#include "delaunay.h"
#include <opencv2/opencv.hpp>
#include "Dlib.h"
Test::Test()
{

}

void Test::testDelaunay()
{
    using namespace std;
    using namespace cv;
    //    cv::Point left(150,250);
    //    cv::Point top(250,150);
    //    cv::Point right(300,290);
    //    std::vector<cv::Point> points;
    //    points.push_back(left);
    //    points.push_back(top);
    //    points.push_back(right);
    //    Delaunay<float,cv::Point> delaunay(points);
    //    std::vector<std::array<size_t,3>> result=delaunay.triangle();
    //    std::cout<<"result.size():"<<result.size()<<std::endl;
    //    std::for_each(result.begin(),result.end(),[&](std::array<size_t,3>& tri){
    //        std::cout<<tri[0]<<","<<tri[1]<<","<<tri[2]<<std::endl;
    //    });
    cv::Mat img=cv::imread("data//frame_0004.png");
    DlibInit("data//shape_predictor_68_face_landmarks.dat");
    vector<Rect>rectangles;
    vector<vector<Point>> keypoints;
    if(DlibFace(img,rectangles,keypoints)){
//        keypoints[0].push_back(Point(0,0));
//        keypoints[0].push_back(Point(img.cols-1,0));
//        keypoints[0].push_back(Point(0,img.rows-1));
//        keypoints[0].push_back(Point(img.cols-1,img.rows-1));
        Delaunay<float,cv::Point> delaunay(keypoints[0]);
        std::vector<std::array<size_t,3>> result=delaunay.triangle();
        for(int i=0;i<result.size();i++){
            size_t j=result[i][0];
            size_t k=result[i][1];
            size_t l=result[i][2];

            cv::line(img,keypoints[0][j],keypoints[0][k],cv::Scalar(0,0,255),1,CV_AA);


            cv::line(img,keypoints[0][k],keypoints[0][l],cv::Scalar(0,0,255),1,CV_AA);


            cv::line(img,keypoints[0][l],keypoints[0][j],cv::Scalar(0,0,255),1,CV_AA);

        }
        cv::imshow("img",img);
        cv::waitKey(0);
    }
}
