//
// Created by hanta on 24. 5. 11.
//

#include "Data.h"
#include <vector>
#include <opencv2/core/core.hpp>
#include <Eigen/Core>
#include <pangolin/pangolin.h>
#include <unistd.h>

using namespace cv;
using namespace std;
using namespace Eigen;

void showPointCloud(const vector<Vector4d, Eigen::aligned_allocator<Vector4d>> &pointcloud) {

    if (pointcloud.empty()) {
        cerr << "Point cloud is empty!" << endl;
        return;
    }

    pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glPointSize(2);
        glBegin(GL_POINTS);
        for (auto &p: pointcloud) {
            if (p[3] == 0.0)
                glColor3f(0.0f, 1.0f, 0.0f);
            else if (p[3] == 1.0)
                glColor3f(0.0f, 0.0f, 1.0f);
            else
                glColor3f(1.0f, 0.0f, 0.0f);
            glVertex3d(p[0], p[1], p[2]);
        }
        glEnd();
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
    return;
}

bool HasSameValue(vector<int> vec)
{
    for(int i = 0; i < vec.size(); i++)
    {
        for(int j = i; j < vec.size(); j++)
        {
            if(i != j)
            {
                if(vec[i] == vec[j])
                {
                    return true;
                }
            }
        }
    }
    return false;
}

bool IsParallel(Point3d vec1, Point3d vec2)
{
    if(vec1.x == 0 && vec1.y == 0 && vec1.z == 0)
        return true;
    if(vec2.x == 0 && vec2.y == 0 && vec2.z == 0)
        return true;

    //bool ip = false;
    if(vec1.x == 0 && vec2.x != 0)
        return false;
    if(vec1.y == 0 && vec2.y != 0)
        return false;
    if(vec1.z == 0 && vec2.z != 0)
        return false;

    if(vec1.x == 0 && vec1.y == 0 && vec2.x == 0 && vec2.y == 0)
        return true;
    if(vec1.x == 0 && vec1.z == 0 && vec2.x == 0 && vec2.z == 0)
        return true;
    if(vec1.z == 0 && vec1.y == 0 && vec2.z == 0 && vec2.y == 0)
        return true;

    if(vec1.x == 0 && vec2.x == 0)
        return vec1.y / vec2.y == vec1.z / vec2.z;
    if(vec1.y == 0 && vec2.y == 0)
        return vec1.x / vec2.x == vec1.z / vec2.z;
    if(vec1.z == 0 && vec2.z == 0)
        return vec1.y / vec2.y == vec1.x / vec2.x;

    if(vec2.x == 0)
    {
        return true;
    }
    if(vec2.y == 0)
    {
        return true;
    }
    if(vec2.z == 0)
    {
        return true;
    }
    return (vec1.x / vec2.x == vec1.y / vec2.y) && (vec1.y / vec2.y == vec1.z / vec2.z);
}

int main() {
    vector<Point3d> vecPts;

    for (int i = 0; i < 1459; i++) {
        vecPts.push_back(Point3d(points[i][0], points[i][1], points[i][2]));
    }

    bool isSuccess = false;

    const int numOfrp = 3;
    const int maxIter = 100;
    const double dstInlierRate = 0.65;
    const double inlierThreshold = 0.1;//meter


    vector<int> rp;

    double inlierRate = 0.0;
    int iter = 0;



    do {
        do {
            for (int e = 0; e < numOfrp; e++) {
                rp.push_back(rand() % vecPts.size());
            }
            if (!HasSameValue(rp)) {
                break;
            } else {
                while (!rp.empty()) {
                    rp.erase(rp.begin());
                }
            }
        } while (true);


        vector<bool> innerIsInlier;

        for (int g = 0; g < vecPts.size(); g++) {
            innerIsInlier.push_back(false);
        }


        Point3d vec1 = vecPts[rp[1]] - vecPts[rp[0]];
        Point3d vec2 = vecPts[rp[2]] - vecPts[rp[0]];
        //cout << "rp: " << rp[0] << "," << rp[1] << "," << rp[2] << endl;

        if (IsParallel(vec1, vec2)) {
            while (!rp.empty()) {
                rp.erase(rp.begin());
            }
            iter++;
            continue;
        }

        Point3d outerProduct = Point3d(vec1.y * vec2.z - vec1.z * vec2.y,
                                       vec1.z * vec2.x - vec1.x * vec2.z,
                                       vec1.x * vec2.y - vec1.y * vec2.x);

        double rh =
                outerProduct.x * vecPts[rp[0]].x + outerProduct.y * vecPts[rp[0]].y +
                outerProduct.z * vecPts[rp[0]].z;

        for (int f = 0; f < vecPts.size(); f++) {
            double distance =
                    abs(outerProduct.x * vecPts[f].x + outerProduct.y * vecPts[f].y +
                        outerProduct.z * vecPts[f].z - rh)
                    / sqrt(outerProduct.x * outerProduct.x + outerProduct.y + outerProduct.y +
                           outerProduct.z * outerProduct.z);
            if (distance != NAN)
                innerIsInlier[f] = distance < inlierThreshold;
        }

        int numOfInlierMin = 0;
        int numOfOutlierMin = 0;

        for (int f = 0; f < innerIsInlier.size(); f++) {
            if (innerIsInlier[f]) {
                numOfInlierMin++;
            } else {
                numOfOutlierMin++;
            }
        }

        if (innerIsInlier.size() < 1) {
            while (!rp.empty()) {
                rp.erase(rp.begin());
            }
            iter++;
            continue;
        }

        inlierRate = (double) numOfInlierMin / (double) innerIsInlier.size();

        if (inlierRate > dstInlierRate) {
            //cout << "inlierRate1: " << inlierRate1 << ", iterations: " << iter << endl;
            isSuccess = true;
            //break;
        }

        if (isSuccess) {
            vector<Vector4d, Eigen::aligned_allocator<Vector4d>> pointcloud;

            for (int v = 0; v < 460 + 999; v++) {
                double brightness = 1.0;
                if(innerIsInlier[v])
                {
                    brightness = 0.0;
                }
                else
                {
                    brightness = 0.5;
                }

                Vector4d point(vecPts[v].x, vecPts[v].y, vecPts[v].z, brightness);

                pointcloud.push_back(point);
            }

            showPointCloud(pointcloud);

            break;
        } else {
            while (!rp.empty()) {
                rp.erase(rp.begin());
            }
            iter++;
            continue;
        }
    } while (iter < maxIter);

}