//
// Created by hanta on 24. 5. 10.
//


#include "SONG_RANSAC.h"
#include <iostream>
#include <math.h>
#include "matrix.h"

bool SONG_RANSAC::HasSameValue(vector<int> vec)
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

vector<float> SONG_RANSAC::DirectLinearTransform(vector<SPoint2f> ref, vector<SPoint2f> dst){
    matrix A = matrix(8, 8);
    float B[8];

    A.cell[0][0] = ref[0].x;
    A.cell[0][1] = ref[0].y;
    A.cell[0][2] = 1;
    A.cell[0][3] = 0;
    A.cell[0][4] = 0;
    A.cell[0][5] = 0;
    A.cell[0][6] = -ref[0].x * dst[0].x;
    A.cell[0][7] = -ref[0].y * dst[0].x;

    A.cell[1][0] = 0;
    A.cell[1][1] = 0;
    A.cell[1][2] = 0;
    A.cell[1][3] = ref[0].x;
    A.cell[1][4] = ref[0].y;
    A.cell[1][5] = 1;
    A.cell[1][6] = -ref[0].x * dst[0].y;
    A.cell[1][7] = -ref[0].y * dst[0].y;

    A.cell[2][0] = ref[1].x;
    A.cell[2][1] = ref[1].y;
    A.cell[2][2] = 1;
    A.cell[2][3] = 0;
    A.cell[2][4] = 0;
    A.cell[2][5] = 0;
    A.cell[2][6] = -ref[1].x * dst[1].x;
    A.cell[2][7] = -ref[1].y * dst[1].x;

    A.cell[3][0] = 0;
    A.cell[3][1] = 0;
    A.cell[3][2] = 0;
    A.cell[3][3] = ref[1].x;
    A.cell[3][4] = ref[1].y;
    A.cell[3][5] = 1;
    A.cell[3][6] = -ref[1].x * dst[1].y;
    A.cell[3][7] = -ref[1].y * dst[1].y;

    A.cell[4][0] = ref[2].x;
    A.cell[4][1] = ref[2].y;
    A.cell[4][2] = 1;
    A.cell[4][3] = 0;
    A.cell[4][4] = 0;
    A.cell[4][5] = 0;
    A.cell[4][6] = -ref[2].x * dst[2].x;
    A.cell[4][7] = -ref[2].y * dst[2].x;

    A.cell[5][0] = 0;
    A.cell[5][1] = 0;
    A.cell[5][2] = 0;
    A.cell[5][3] = ref[2].x;
    A.cell[5][4] = ref[2].y;
    A.cell[5][5] = 1;
    A.cell[5][6] = -ref[2].x * dst[2].y;
    A.cell[5][7] = -ref[2].y * dst[2].y;

    A.cell[6][0] = ref[3].x;
    A.cell[6][1] = ref[3].y;
    A.cell[6][2] = 1;
    A.cell[6][3] = 0;
    A.cell[6][4] = 0;
    A.cell[6][5] = 0;
    A.cell[6][6] = -ref[3].x * dst[3].x;
    A.cell[6][7] = -ref[3].y * dst[3].x;

    A.cell[7][0] = 0;
    A.cell[7][1] = 0;
    A.cell[7][2] = 0;
    A.cell[7][3] = ref[3].x;
    A.cell[7][4] = ref[3].y;
    A.cell[7][5] = 1;
    A.cell[7][6] = -ref[3].x * dst[3].y;
    A.cell[7][7] = -ref[3].y * dst[3].y;

    B[0] = dst[0].x;
    B[1] = dst[0].y;
    B[2] = dst[1].x;
    B[3] = dst[1].y;
    B[4] = dst[2].x;
    B[5] = dst[2].y;
    B[6] = dst[3].x;
    B[7] = dst[3].y;

    matrix invA = invertMatrix(A);

    vector<float> res;
    res.push_back(0.0);
    res.push_back(0.0);
    res.push_back(0.0);
    res.push_back(0.0);
    res.push_back(0.0);
    res.push_back(0.0);
    res.push_back(0.0);
    res.push_back(0.0);
    res.push_back(1.0);
    for (int row = 0; row < 8; ++row) {
        float sum = 0;
        for (int col = 0; col < 8; ++col)
        {
            sum += invA.cell[row][col] * B[col];
        }
        res[row] = sum;
    }

    return res;
}

bool SONG_RANSAC::IsInlierBySymmetricTransferError(SPoint2f ref, SPoint2f dst, float threshold, vector<float> H){

    float A[3][3] = {H[0], H[1], H[2], H[3], H[4], H[5], H[6], H[7], H[8]};

    float invA[3][3];

    /*
    float det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
            - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
            + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
            */

    invA[0][0] = (A[1][1] * A[2][2] - A[1][2] * A[2][1]) / (A[0][0] * A[1][1] - A[0][1] * A[1][0]);
    invA[0][1] = -(A[0][1] * A[2][2] - A[0][2] * A[2][1]) / (A[0][0] * A[1][1] - A[0][1] * A[1][0]);
    invA[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) / (A[0][0] * A[1][1] - A[0][1] * A[1][0]);
    invA[1][0] = -(A[1][0] * A[2][2] - A[1][2] * A[2][0]) / (A[0][0] * A[1][1] - A[0][1] * A[1][0]);
    invA[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) / (A[0][0] * A[1][1] - A[0][1] * A[1][0]);
    invA[1][2] = -(A[0][0] * A[1][2] - A[0][2] * A[1][0]) / (A[0][0] * A[1][1] - A[0][1] * A[1][0]);
    invA[2][0] = (A[1][0] * A[2][1] - A[1][1] * A[2][0]) / (A[0][0] * A[1][1] - A[0][1] * A[1][0]);
    invA[2][1] = -(A[0][0] * A[2][1] - A[0][1] * A[2][0]) / (A[0][0] * A[1][1] - A[0][1] * A[1][0]);
    invA[2][2] = (A[0][0] * A[1][1] - A[0][1] * A[1][0]) / (A[0][0] * A[1][1] - A[0][1] * A[1][0]);

    SPoint2f refA;
    refA.x = (ref.x * A[0][0] + ref.y * A[0][1] + A[0][2]) / (ref.x * A[2][0] + ref.y * A[2][1] + A[2][2]);
    refA.y = (ref.x * A[1][0] + ref.y * A[1][1] + A[1][2]) / (ref.x * A[2][0] + ref.y * A[2][1] + A[2][2]);

    SPoint2f dstInvA;
    dstInvA.x = (dst.x * invA[0][0] + dst.y * invA[0][1] + invA[0][2]) / (dst.x * invA[2][0] + dst.y * invA[2][1] + invA[2][2]);
    dstInvA.y = (dst.x * invA[1][0] + dst.y * invA[1][1] + invA[1][2]) / (dst.x * invA[2][0] + dst.y * invA[2][1] + invA[2][2]);


    float refDis = sqrt((ref.x - dstInvA.x) * (ref.x - dstInvA.x) + (ref.y - dstInvA.y) * (ref.y - dstInvA.y));
    float dstDis = sqrt((dst.x - refA.x) * (dst.x - refA.x) + (dst.y - refA.y) * (dst.y - refA.y));

    return (refDis + dstDis) < threshold;
}

vector<float> SONG_RANSAC::SRANSAC(vector<SPoint2f> ref, vector<SPoint2f> dst, float dstInlierRate, int maxIter, float threshold){
    srand(time(NULL));
    vector<int> rp;
    int numOfRp = 4;
    int iter = 0;
    vector<float> H;

    do {
        do {
            for (int e = 0; e < numOfRp; e++) {
                rp.push_back(rand() % ref.size());
            }
            if (!SONG_RANSAC::HasSameValue(rp)) {
                break;
            } else {
                while (!rp.empty()) {
                    rp.erase(rp.begin());
                }
            }
        } while (true);

        vector<SPoint2f> selectedRef, selectedDst;

        for(int i = 0; i < rp.size(); i++)
        {
            selectedRef.push_back(ref[rp[i]]);
            selectedDst.push_back(dst[rp[i]]);
        }

        H = DirectLinearTransform(selectedRef, selectedDst);

        int numOfInlier = 0;

        for(int i = 0; i < ref.size(); i++)
        {
            if(IsInlierBySymmetricTransferError(ref[i], dst[i], threshold, H))
            {
                numOfInlier++;
            }
        }

        float inlierRate = (float)numOfInlier / (float)ref.size();

        if(inlierRate >= dstInlierRate)
        {
            cout << "Inlier rate: " << inlierRate << ", Iterations: " << iter << endl;
            return H;
        }

        iter++;
    } while (iter < maxIter);

    if(iter >= maxIter)
    {
        cerr << "RANSANC failed." << endl;
    }

    return H;
}
