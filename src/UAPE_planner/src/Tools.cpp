#pragma once
#include <Tools.h>

double clip(double x, double minv, double maxv)
{
    if (x<=minv)
    {
        return minv;
    }
    else if (x>=maxv)
    {
        return maxv;
    }
    else
    {
        return x;
    }
}

// Quaterniond normalizeQ(Quaterniond q)
// {
//     double norm = sqrt(q.x() * q.x() + q.y() * q.y() + q.z() * q.z() + q.w() * q.w());
//     q.x() = q.x() / norm;
//     q.y() = q.y() / norm;
//     q.z() = q.z() / norm;
//     q.w() = q.w() / norm;

//     return q;
// }

Quaterniond Euler2Quaternion(Vector3d euler)
{
    double cr = std::cos(euler(0) * 0.5);
    double sr = std::sin(euler(0) * 0.5);
    double cp = std::cos(euler(1) * 0.5);
    double sp = std::sin(euler(1) * 0.5);
    double cy = std::cos(euler(2) * 0.5);
    double sy = std::sin(euler(2) * 0.5);

    Quaterniond q;
    q.w() = cr * cp * cy + sr * sp * sy;
    q.x() = sr * cp * cy - cr * sp * sy;
    q.y() = cr * sp * cy + sr * cp * sy;
    q.z() = cr * cp * sy - sr * sp * cy;

    return q.normalized();
}

Quaterniond Rota2Quaternion(Matrix3d Rota)
{
    Quaterniond q;
    q.w() = std::sqrt(Rota.trace() + 1) / 2;
    q.x() = (Rota(2, 1) - Rota(1, 2)) / 4.0 / q.w();
    q.y() = (Rota(0, 2) - Rota(2, 0)) / 4.0 / q.w();
    q.z() = (Rota(1, 0) - Rota(0, 1)) / 4.0 / q.w();

    return q.normalized();
}

// convert quaternion to euler angles (with theta singularity)
Vector3d Quaternion2Euler(Quaterniond q)
{       
    Vector3d euler;

    // roll
    euler(0) = std::atan2(2 * (q.w() * q.x() + q.y() * q.z()), 1 - 2 * (q.x() * q.x() + q.y() * q.y()));

    // pitch
    double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
    if (std::abs(sinp) >= 1)
        euler(1) = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        euler(1) = std::asin(sinp);

    // yaw
    euler(2) = std::atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));

    return euler;
}

// convert quaternion to rotation matrix
Matrix3d Quaternion2Rota(Quaterniond q)
{
    Matrix3d rota;
    double r, i, j, k;
    r = q.w();
    i = q.x();
    j = q.y();
    k = q.z();

    // convert to rota
    rota(0, 0) = 1 - 2 * (j * j + k * k);
    rota(0, 1) = 2 * (i * j - k * r);
    rota(0, 2) = 2 * (i * k + j * r);
    //
    rota(1, 0) = 2 * (i * j + k * r);
    rota(1, 1) = 1 - 2 * (i * i + k * k);
    rota(1, 2) = 2 * (j * k - i * r);
    //
    rota(2, 0) = 2 * (i * k - j * r);
    rota(2, 1) = 2 * (j * k + i * r);
    rota(2, 2) = 1 - 2 * (i * i + j * j);

    return rota;
}

// convert rotation matrix to euler angles
Vector3d Rota2Euler(Matrix3d Rota)
{
    Quaterniond q = Rota2Quaternion(Rota);
    Vector3d euler = Quaternion2Euler(q);
    return euler;
}
