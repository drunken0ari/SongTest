//
// Created by hanta on 24. 5. 10.
//
#include <opencv2/highgui.hpp>
#include <opencv2/stitching.hpp>
#include <vector>

using namespace cv;
using namespace std;

int main() {
    vector<Mat> imgs;
    Mat result;

    Mat img1 = imread("../../img0.jpg");
    imgs.push_back(img1);
    Mat img2 = imread("../../img1.jpg");
    imgs.push_back(img2);
    Mat img3 = imread("../../img2.jpg");
    imgs.push_back(img3);
    Mat img4 = imread("../../img3.jpg");
    imgs.push_back(img4);
    Mat img5 = imread("../../img4.jpg");
    imgs.push_back(img5);

    Ptr<Stitcher> stitcher = Stitcher::create();
    Stitcher::Status status = stitcher->stitch(imgs, result);

    imshow("Result", result);
    waitKey(0);
}