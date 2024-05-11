//
// Created by hanta on 24. 5. 11.
//
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <vector>
#include <fstream>
#include <Eigen/Core>

using namespace cv;
using namespace std;
using namespace Eigen;

const double fx = 9.597910e+02;
const double fy = 9.569251e+02;
const double cx = 6.960217e+02;
const double cy = 2.241806e+02;

int main() {
    vector<Vector4f> lidarPts;
    vector<Vector4f> convertedPts;
    vector<Vector3f> pts2d;
    vector<float> colors;

    Matrix<float, 4, 4> P2;
    P2 << 7.215377e+02, 0.000000e+00, 6.095593e+02, 4.485728e+01
        , 0.000000e+00, 7.215377e+02, 1.728540e+02, 2.163791e-01
        , 0.000000e+00, 0.000000e+00, 1.000000e+00, 2.745884e-03
        , 0, 0, 0, 1;

    Matrix<float, 4, 4> R0;
    R0 << 9.999239e-01, 9.837760e-03, -7.445048e-03, 0
        , -9.869795e-03, 9.999421e-01, -4.278459e-03, 0
        , 7.402527e-03, 4.351614e-03, 9.999631e-01, 0
        , 0, 0, 0, 1;

    Matrix<float, 4, 4> Rt;

    Rt << 7.533745e-03, -9.999714e-01, -6.166020e-04, -4.069766e-03
        , 1.480249e-02, 7.280733e-04, -9.998902e-01, -7.631618e-02
        , 9.998621e-01, 7.523790e-03, 1.480755e-02, -2.717806e-01
        , 0, 0, 0, 1;

    ifstream is("../../0000000000.bin", ifstream::binary);
    is.seekg(0, is.end);
    int length = (int)is.tellg();
    is.seekg(0, is.beg);

    // malloc으로 메모리 할당
    unsigned char * buffer = (unsigned char*)malloc(length);

    // read data as a block:
    is.read((char*)buffer, length);
    is.close();

    int LidarLength = length / 16;

    for(int i = 0; i < LidarLength; i++)
    {
        float x;
        memcpy(&x, buffer + i * 16, sizeof(float));
        float y;
        memcpy(&y, buffer + i  * 16 + 4, sizeof(float));
        float z;
        memcpy(&z, buffer + i * 16 + 8, sizeof(float));
        float r;
        memcpy(&r, buffer + i * 16 + 12, sizeof(float));
        lidarPts.push_back(Vector4f(x, y, z, 1));
        colors.push_back(r);
        //cout << x << ", " << y << ", " <<  z << ", " <<  r << endl;
    }

    for(int i = 0; i < lidarPts.size(); i++)
    {
        convertedPts.push_back(P2 * R0 * Rt * lidarPts[i]);
        cout << convertedPts[i](0) << ", " << convertedPts[i](1) << ", " <<  convertedPts[i](2) << ", " <<  colors[i] << endl;
    }

    Mat img = imread("../../0000000000.png");
    imshow("orig", img);

    for(int i = 0; i < convertedPts.size(); i++) {
        if(convertedPts[i](2) > 0) {
            float u, v;
            //u = (convertedPts[i](0) * fx) / (2.0 * convertedPts[i](2)) + cx;
            //v = (convertedPts[i](1) * fy) / (2.0 * convertedPts[i](2)) + cy;
            //u = (convertedPts[i](0) * img.cols) / (2.0 * convertedPts[i](2)) + img.cols / 2.0;
            //v = (convertedPts[i](1) * img.rows) / (2.0 * convertedPts[i](2)) + img.rows / 2.0;
            //u = (convertedPts[i](0) - cx) * (fx / convertedPts[i](2)) + cx;
            //v = (convertedPts[i](1) - cy) * (fy / convertedPts[i](2)) + cy;
            u = convertedPts[i](0) /convertedPts[i](2);
            v = convertedPts[i](1) / convertedPts[i](2);
            pts2d.push_back(Vector3f(u, v, colors[i]));

        }
    }

    Mat lidar = Mat(img.size(), CV_8UC1, Scalar(255));

    for(int i = 0; i < pts2d.size(); i++) {
        if(pts2d[i](0) >= 0 && pts2d[i](0) <= img.cols - 1) {
            if(pts2d[i](1) >= 0 && pts2d[i](1) <= img.rows - 1) {
                //cout << convertedPts[i](0) << ", " << convertedPts[i](1) << ", " <<  colors[i] << endl;
                lidar.at<uchar>((int)pts2d[i](1), (int)pts2d[i](0)) = (int)(pts2d[i](2) * 255.0);
            }
        }
    }
    imshow("lidar", lidar);
    waitKey(0);
}