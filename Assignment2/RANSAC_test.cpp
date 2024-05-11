//
// Created by hanta on 24. 5. 10.
//
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>
#include "SONG_RANSAC.h"

using namespace cv;
using namespace std;

int main() {
    Ptr<ORB> orb = ORB::create();
    Mat img1 = imread("../../img0.jpg");
    Mat img2 = imread("../../img1.jpg");

    vector<KeyPoint> RefKeys, DstKeys;
    Mat RefDesc, DstDesc;

    Ptr<BFMatcher> matcher = BFMatcher::create();

    vector<DMatch> matches;

    orb->detectAndCompute(img1, Mat(), RefKeys, RefDesc);
    orb->detectAndCompute(img2, Mat(), DstKeys, DstDesc);

    matcher->match(RefDesc, DstDesc, matches);

    sort(matches.begin(), matches.end());
    vector<DMatch> goodMatches(matches.begin(), matches.begin() + 50);

    std::vector<SPoint2f> RefGood, DstGood;
    for (int i = 0; i < goodMatches.size(); i++)
    {
        SPoint2f ref;
        ref.x = RefKeys[goodMatches[i].queryIdx].pt.x;
        ref.y = RefKeys[goodMatches[i].queryIdx].pt.y;
        RefGood.push_back(ref);
        SPoint2f dst;
        dst.x = DstKeys[goodMatches[i].trainIdx].pt.x;
        dst.y = DstKeys[goodMatches[i].trainIdx].pt.y;
        DstGood.push_back(dst);
    }

    vector<float> H = SONG_RANSAC::SRANSAC(RefGood, DstGood, 0.75, 1000, 15.0);

    Mat warp = Mat(img1.size(), CV_8UC3, Scalar(0, 0, 0));

    for(int row = 0; row < img1.rows; row++)
    {
        for(int col = 0; col < img1.cols; col++)
        {
            int hrow, hcol;
            hcol = ((float)col * H[0] + (float)row * H[1] + H[2]) / ((float)col * H[6] + (float)row * H[7] + H[8]);
            hrow = ((float)col * H[3] + (float)row * H[4] + H[5]) / ((float)col * H[6] + (float)row * H[7] + H[8]);

            if(hcol > -1 && hcol < img1.cols) {
                if(hrow > -1 && hrow < img1.rows) {
                    warp.at<Vec3b>(row, col) = img2.at<Vec3b>(hrow, hcol);
                }
            }
        }
    }

    imshow("img_ref", img1);
    imshow("img_dst", img2);
    imshow("warp_img", warp);
    waitKey(0);

}