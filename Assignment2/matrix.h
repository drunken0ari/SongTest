//
// Created by hanta on 24. 5. 10.
//

#ifndef SONGTEST_MATRIX_H
#define SONGTEST_MATRIX_H

#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

class matrix
{
public:
    int w, h;
    std::vector<std::vector<float>> cell;

    matrix(int width, int height)
    {
        w = width;
        h = height;
        cell.resize(width);
        for (int i = 0; i < cell.size(); i++)
        {
            cell[i].resize(height);
        }
    }
};

matrix transponseMatrix(matrix M)
{
    matrix A(M.h, M.w);

    for (int i = 0; i < M.h; i++)
    {
        for (int j = 0; j < M.w; j++)
        {
            A.cell[i][j] = M.cell[j][i];
        }
    }

    return A;
}

float getMatrixDeterminant(matrix M)
{
    if (M.w != M.h)
    {
        std::cout << "ERROR! Matrix isn't of nXn type.\n";
        return NULL;
    }
    float determinante = 0;
    if (M.w == 1)
    {
        determinante = M.cell[0][0];
    }
    if (M.w == 2)
    {
        determinante = M.cell[0][0] * M.cell[1][1] - M.cell[1][0] * M.cell[0][1];
    }
    else
    {
        for (int i = 0; i < M.w; i++)
        {
            matrix A(M.w - 1, M.h - 1);
            int cy = 0;
            for (int y = 1; y < M.h; y++)
            {
                int cx = 0;
                for (int x = 0; x < M.w; x++)
                {
                    if (x != i)
                    {
                        A.cell[cx][cy] = M.cell[x][y];
                        cx++;
                    }
                }
                cy++;
            }

            determinante += M.cell[i][0] * pow(-1, i + 0) * getMatrixDeterminant(A);
        }
    }

    return determinante;
}

float getComplementOf(matrix M, int X, int Y)
{
    float det;

    if (M.w != M.h)
    {
        std::cout << "ERROR! Matrix isn't of nXn type.\n";
        return NULL;
    }

    if (M.w == 2)
    {
        det = M.cell[1 - X][1 - Y];
    }
    else
    {
        matrix A(M.w - 1, M.h - 1);
        int cy = 0;
        for (int y = 0; y < M.h; y++)
        {
            if (y != Y)
            {
                int cx = 0;
                for (int x = 0; x < M.w; x++)
                {
                    if (x != X)
                    {
                        A.cell[cx][cy] = M.cell[x][y];
                        cx++;
                    }
                }
                cy++;
            }
        }
        det = getMatrixDeterminant(A);
    }

    return (pow(-1, X + Y) * det);
}

matrix invertMatrix(matrix M)
{
    matrix A(M.w, M.h);
    float det = getMatrixDeterminant(M);
    if (det == 0)
    {
        std::cout << "ERROR! Matrix inversion impossible (determinant is equal to 0).\n";
        return A;
    }

    for (int i = 0; i < M.h; i++)
    {
        for (int j = 0; j < M.w; j++)
        {
            A.cell[j][i] = getComplementOf(M, j, i) / det;
        }
    }

    A = transponseMatrix(A);

    return A;
}


#endif //SONGTEST_MATRIX_H
