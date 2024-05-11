//
// Created by hanta on 24. 5. 10.
//
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/QR>
#include <eigen3/Eigen/SVD>
#include <iostream>

using namespace Eigen;
using namespace std;

int main() {
    double points3D[6][3] =  {0.613428, 0.172717, 1.623
        , 0.646371, 0.171547, 1.612
        , 0.666511, 0.16729, 1.572
        , 0.686091, 0.183806, 1.648
        , 0.69747, 0.193052, 1.655
        , 0.592523, 0.501775, 1.758};
    double points2D[6][2] = {473, 282
        , 482, 282
        , 491, 282
        , 465, 283
        , 474, 283
        , 482, 283};

    const double fx = 320.0;
    const double fy = 320.0;
    const double cx = 320.0;
    const double cy = 240.0;

    MatrixXd A;
    A.resize ( 12, 12 );
    for(int i = 0; i < 6; i++)
    {
        double x = points3D[i][0];
        double y = points3D[i][1];
        double z = points3D[i][2];
        double u = points2D[i][0];
        double v = points2D[i][1];

        A ( 2*i, 0 ) = x*fx;
        A ( 2*i, 1 ) = y*fx;
        A ( 2*i, 2 ) = z*fx;
        A ( 2*i, 3 ) = fx;
        A ( 2*i, 4 ) = 0.0;
        A ( 2*i, 5 ) = 0.0;
        A ( 2*i, 6 ) = 0.0;
        A ( 2*i, 7 ) = 0.0;
        A ( 2*i, 8 ) = x*cx-u*x;
        A ( 2*i, 9 ) = y*cx-u*y;
        A ( 2*i, 10 ) = z*cx-u*z;
        A ( 2*i, 11 ) = cx-u;


        A ( 2*i+1, 0 ) = 0.0;
        A ( 2*i+1, 1 ) = 0.0;
        A ( 2*i+1, 2 ) = 0.0;
        A ( 2*i+1, 3 ) = 0.0;
        A ( 2*i+1, 4 ) = x*fy;
        A ( 2*i+1, 5 ) = y*fy;
        A ( 2*i+1, 6 ) = z*fy;
        A ( 2*i+1, 7 ) = fy;
        A ( 2*i+1, 8 ) = x*cy-v*x;
        A ( 2*i+1, 9 ) = y*cy-v*y;
        A ( 2*i+1, 10 ) = z*cy-v*z;
        A ( 2*i+1, 11 ) = cy-v;
    }

// Step 2. Solve Ax = 0 by SVD
    JacobiSVD<MatrixXd> svd_A ( A, ComputeThinV );
    MatrixXd V_A = svd_A.matrixV();
    MatrixXd Sigma_A = svd_A.singularValues();

    // TODO It is better to more singular vectors, as the null space may contain more than one singular vectors.
    // std::cout << Sigma_A.transpose() << std::endl;

    // a1-a12 bar
    double a1 = V_A ( 0, 11 );
    double a2 = V_A ( 1, 11 );
    double a3 = V_A ( 2, 11 );
    double a4 = V_A ( 3, 11 );
    double a5 = V_A ( 4, 11 );
    double a6 = V_A ( 5, 11 );
    double a7 = V_A ( 6, 11 );
    double a8 = V_A ( 7, 11 );
    double a9 = V_A ( 8, 11 );
    double a10 = V_A ( 9, 11 );
    double a11 = V_A ( 10, 11 );
    double a12 = V_A ( 11, 11 );

    
    Matrix3d R_bar;
    R_bar << a1, a2, a3, a5, a6, a7, a9, a10, a11;
    JacobiSVD<MatrixXd> svd_R ( R_bar, ComputeFullU | ComputeFullV );
    Matrix3d U_R = svd_R.matrixU();
    Matrix3d V_R = svd_R.matrixV();
    Vector3d V_Sigma = svd_R.singularValues();

    Matrix3d R;
    Vector3d t;

    R = U_R * V_R.transpose();
    double beta = 1.0 / ( ( V_Sigma ( 0 ) +V_Sigma ( 1 ) +V_Sigma ( 2 ) ) / 3.0 );

    // Step 4. Compute t
    Vector3d t_bar ( a4, a8, a12 );
    t = beta * t_bar;


    // Check + -
    int num_positive = 0;
    int num_negative = 0;
    for ( int i = 0; i < 6; i ++ ) {
        const Vector3d& pt3d = Vector3d(points3D[i][0], points3D[i][1], points3D[i][2]);
        const double& x = pt3d[0];
        const double& y = pt3d[1];
        const double& z = pt3d[2];

        double lambda = beta * ( x * a9 + y* a10 + z* a11 + a12 );
        if ( lambda >= 0 ) {
            num_positive ++;
        } else {
            num_negative ++;
        }
    }

    if ( num_positive < num_negative ) {
        R = -R;
        t = -t;
    }

    cout << "R: \n"
            << R(0, 0) << ", " << R(0, 1) << ", " << R(0, 2) << "\n"
            << R(1, 0) << ", " << R(1, 1) << ", " << R(1, 2) << "\n"
            << R(2, 0) << ", " << R(2, 1) << ", " << R(2, 2) << endl;

    cout << "t: " << t(0) << ", " << t(1) << ", " << t(2) << endl;
}