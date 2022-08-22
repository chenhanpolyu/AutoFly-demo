#pragma once

#include <iostream>
#include <math.h>
#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;

struct States
{
    // raw states
    Vector3d A_Eraw, Rate_Braw;
    Vector3d Rate_Eraw;//////////

    // for ctrl
    Vector3d P_E, V_E, A_E;

    Matrix3d Rota, Rota_EB;
    Quaterniond Quat;
    Vector3d Euler;

    Vector3d Rate_E, Rate_B;
};

double clip(double x, double minv, double maxv);

// Quaterniond normalizeQ(Quaterniond q);

Quaterniond Euler2Quaternion(Vector3d euler);

Quaterniond Rota2Quaternion(Matrix3d Rota);

// convert quaternion to euler angles (with theta singularity)
Vector3d Quaternion2Euler(Quaterniond q);

// convert quaternion to rotation matrix
Matrix3d Quaternion2Rota(Quaterniond q);

// convert rotation matrix to euler angles
Vector3d Rota2Euler(Matrix3d Rota);
