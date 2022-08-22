/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#ifndef SE3GCOPTER_HPP
#define SE3GCOPTER_HPP

#include "trajectory.hpp"
#include <call_states/ros_communicate.h>
// @article{WANG2021GCOPTER,
//     title={Geometrically Constrained Trajectory Optimization for Multicopters},
//     author={Wang, Zhepei and Zhou, Xin and Xu, Chao and Gao, Fei},
//     journal={arXiv preprint arXiv:2103.00190},
//     year={2021}
// }

class MINCO_S3
{
public:
    MINCO_S3() = default;
    ~MINCO_S3() { A.destroy(); }
    double pena = 0.0;
    double pena_ball = 0.0;
    double pena_sta = 0.0;
    double pena_acc = 0.0;
    double cost_total;
    double dyn_coe = 15.0;
    // my
    // double compute_time = 0;
private:
    int N;
    Eigen::Matrix3d headPVA;
    Eigen::Matrix3d tailPVA;
    Eigen::VectorXd T1;
    BandedSystem A;
    Eigen::MatrixXd b;

    // Temp variables
    Eigen::VectorXd T2;
    Eigen::VectorXd T3;
    Eigen::VectorXd T4;
    Eigen::VectorXd T5;
    Eigen::MatrixXd gdC;
    double *cost_block;

private:
    template <typename EIGENVEC>
    inline void addGradJbyT(EIGENVEC &gdT) const
    {
        for (int i = 0; i < N; i++)
        {
            gdT(i) += 36.0 * b.row(6 * i + 3).squaredNorm() +
                      288.0 * b.row(6 * i + 4).dot(b.row(6 * i + 3)) * T1(i) +
                      576.0 * b.row(6 * i + 4).squaredNorm() * T2(i) +
                      720.0 * b.row(6 * i + 5).dot(b.row(6 * i + 3)) * T2(i) +
                      2880.0 * b.row(6 * i + 5).dot(b.row(6 * i + 4)) * T3(i) +
                      3600.0 * b.row(6 * i + 5).squaredNorm() * T4(i);
        }
        return;
    }

    template <typename EIGENMAT>
    inline void addGradJbyC(EIGENMAT &gdC) const
    {
        for (int i = 0; i < N; i++)
        {
            gdC.row(6 * i + 5) += 240.0 * b.row(6 * i + 3) * T3(i) +
                                  720.0 * b.row(6 * i + 4) * T4(i) +
                                  1440.0 * b.row(6 * i + 5) * T5(i);
            gdC.row(6 * i + 4) += 144.0 * b.row(6 * i + 3) * T2(i) +
                                  384.0 * b.row(6 * i + 4) * T3(i) +
                                  720.0 * b.row(6 * i + 5) * T4(i);
            gdC.row(6 * i + 3) += 72.0 * b.row(6 * i + 3) * T1(i) +
                                  144.0 * b.row(6 * i + 4) * T2(i) +
                                  240.0 * b.row(6 * i + 5) * T3(i);
        }
        return;
    }

    inline void solveAdjGradC(Eigen::MatrixXd &gdC) const
    {
        A.solveAdj(gdC);
        return;
    }

    template <typename EIGENVEC>
    inline void addPropCtoT(const Eigen::MatrixXd &adjGdC, EIGENVEC &gdT) const
    {
        Eigen::MatrixXd B1(6, 3), B2(3, 3);

        Eigen::RowVector3d negVel, negAcc, negJer, negSnp, negCrk;

        for (int i = 0; i < N - 1; i++)
        {
            negVel = -(b.row(i * 6 + 1) +
                       2.0 * T1(i) * b.row(i * 6 + 2) +
                       3.0 * T2(i) * b.row(i * 6 + 3) +
                       4.0 * T3(i) * b.row(i * 6 + 4) +
                       5.0 * T4(i) * b.row(i * 6 + 5));
            negAcc = -(2.0 * b.row(i * 6 + 2) +
                       6.0 * T1(i) * b.row(i * 6 + 3) +
                       12.0 * T2(i) * b.row(i * 6 + 4) +
                       20.0 * T3(i) * b.row(i * 6 + 5));
            negJer = -(6.0 * b.row(i * 6 + 3) +
                       24.0 * T1(i) * b.row(i * 6 + 4) +
                       60.0 * T2(i) * b.row(i * 6 + 5));
            negSnp = -(24.0 * b.row(i * 6 + 4) +
                       120.0 * T1(i) * b.row(i * 6 + 5));
            negCrk = -120.0 * b.row(i * 6 + 5);

            B1 << negSnp, negCrk, negVel, negVel, negAcc, negJer;

            gdT(i) += B1.cwiseProduct(adjGdC.block<6, 3>(6 * i + 3, 0)).sum();
        }

        negVel = -(b.row(6 * N - 5) +
                   2.0 * T1(N - 1) * b.row(6 * N - 4) +
                   3.0 * T2(N - 1) * b.row(6 * N - 3) +
                   4.0 * T3(N - 1) * b.row(6 * N - 2) +
                   5.0 * T4(N - 1) * b.row(6 * N - 1));
        negAcc = -(2.0 * b.row(6 * N - 4) +
                   6.0 * T1(N - 1) * b.row(6 * N - 3) +
                   12.0 * T2(N - 1) * b.row(6 * N - 2) +
                   20.0 * T3(N - 1) * b.row(6 * N - 1));
        negJer = -(6.0 * b.row(6 * N - 3) +
                   24.0 * T1(N - 1) * b.row(6 * N - 2) +
                   60.0 * T2(N - 1) * b.row(6 * N - 1));

        B2 << negVel, negAcc, negJer;

        gdT(N - 1) += B2.cwiseProduct(adjGdC.block<3, 3>(6 * N - 3, 0)).sum();

        return;
    }

    template <typename EIGENMAT>
    inline void addPropCtoP(const Eigen::MatrixXd &adjGdC, EIGENMAT &gdInP) const
    {
        for (int i = 0; i < N - 1; i++)
        {
            gdInP.col(i) += adjGdC.row(6 * i + 5).transpose();
        }
        return;
    }

    inline void normalizeFDF(const Eigen::Vector3d &x,
                             Eigen::Vector3d &xNor,
                             Eigen::Matrix3d &G) const
    {
        const double a = x(0), b = x(1), c = x(2);
        const double aSqr = a * a, bSqr = b * b, cSqr = c * c;
        const double ab = a * b, bc = b * c, ca = c * a;
        const double xSqrNorm = aSqr + bSqr + cSqr;
        const double xNorm = sqrt(xSqrNorm);
        const double den = xSqrNorm * xNorm;
        xNor = x / xNorm;
        G(0, 0) = bSqr + cSqr;
        G(0, 1) = -ab;
        G(0, 2) = -ca;
        G(1, 0) = -ab;
        G(1, 1) = aSqr + cSqr;
        G(1, 2) = -bc;
        G(2, 0) = -ca;
        G(2, 1) = -bc;
        G(2, 2) = aSqr + bSqr;
        G /= den;
        return;
    }

    inline void positiveSmoothedL1(const double &x, double &f, double &df) const
    {
        const double pe = 1.0e-4;
        const double half = 0.5 * pe;
        const double f3c = 1.0 / (pe * pe);
        const double f4c = -0.5 * f3c / pe;
        const double d2c = 3.0 * f3c;
        const double d3c = 4.0 * f4c;

        if (x < pe)
        {
            f = (f4c * x + f3c) * x * x * x;
            df = (d3c * x + d2c) * x * x;
        }
        else
        {
            f = x - half;
            df = 1.0;
        }

        return;
    }

    template <typename EIGENVEC>
    inline void addTimeIntPenalty(const Eigen::VectorXi cons,
                                  const Eigen::VectorXi &idxHs,
                                  const std::vector<Eigen::MatrixXd> &cfgHs,
                                  const Eigen::Vector3d &ellipsoid,
                                  const double safeMargin,
                                  const double vMax,
                                  const double thrAccMin,
                                  const double thrAccMax,
                                  const double bdrMax,
                                  const double gAcc,
                                  const Eigen::Vector4d ci,
                                  double &cost,
                                  EIGENVEC &gdT,
                                  Eigen::MatrixXd &gdC,
                                  const double plan_t,
                                  const dynobs_tmp *dynobs_pointer)
    {
        pena = 0.0;
        pena_ball = 0.0;
        pena_sta = 0.0;
        pena_acc = 0.0;
        double vMaxSqr;
        if (dynobs_pointer->ball_number > 0)
        {
            if (vMax <= 2)
            {
                vMaxSqr = 32.0;
            }
            else
            {
                vMaxSqr = vMax * vMax * 8;
            }
        }
        // else if (dynobs_pointer->dyn_number >0)
        //  {vMaxSqr = vMax * vMax*1.9;}
        else
        {
            vMaxSqr = vMax * vMax;
        }

        const double thrAccMinSqr = thrAccMin * thrAccMin;
        const double thrAccMaxSqr = thrAccMax * thrAccMax;
        const double bdrMaxSqr = bdrMax * bdrMax;
        std::vector<bool> mask_dyn_idx(dynobs_pointer->dyn_number,false);
        Eigen::Vector3d pos, vel, acc, jer, sna;
        double step, alpha;
        double s1, s2, s3, s4, s5;
        Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4;
        int K;
        Eigen::Matrix3d rotM;
        double signedDist, signedDistSqr, signedDistCub;
        double gradSignedDt;
        Eigen::Vector3d h, zB, czB, xC, yB, xB;
        Eigen::Matrix3d dnczB, dzB, cdzB, dyB, dxB;
        Eigen::Vector3d outerNormal, point;
        double eNorm;
        Eigen::Vector3d eNormGd;
        Eigen::Matrix3d gradSdTxyz;
        Eigen::Vector3d gradSdT;
        double outerNormaldVel;
        Eigen::Matrix<double, 6, 3> gradSdCx, gradSdCy, gradSdCz, gradSdC;
        Eigen::Matrix<double, 6, 3> beta2dOuterNormalTp, beta0dOuterNormalTp;

        double violaVel, violaThrl, violaThrh, violaBdr;
        double violaVelPenaD, violaThrlPenaD, violaThrhPenaD, violaBdrPenaD;
        double violaVelPena, violaThrlPena, violaThrhPena, violaBdrPena;
        Eigen::Matrix<double, 6, 3> gradViolaVc, gradViolaThrlc, gradViolaThrhc, gradViolaBdrc;
        double gradViolaVt, gradViolaThrlt, gradViolaThrht, gradViolaBdrt;
        double fThr, sqrMagThr, sqrMagBdr;
        Eigen::Vector3d dfThr, dSqrMagThr, bdr, xyBdr;
        Eigen::Vector3d dSqrMagBdr, rotTrDotJer;
        Eigen::Matrix3d dBdr, dxyBdr;
        Eigen::Vector3d dJerSqrMagBdr;
        Eigen::Matrix3d dJerBdr, dJerxyBdr;
        Eigen::Vector3d ct_center;
        double omg, violaPos, violaPosPenaD, violaPosPena;
        double t_gap, t_now;
        double wei_dyn = ci(0) * dyn_coe;
        double wei_dyn_acc = ci(0) * 0.0025; // 0.0005; //
        double wei_ball = ci(0) * 24;
        double current_pena;
        int innerLoop, idx;
        constexpr double inv_a2 = 1 / 2.0 / 2.0, inv_b2 = 1.0;
        double inv_x, inv_y, inv_z;
        for (int i = 0; i < N; i++)
        {
            const auto &c = b.block<6, 3>(i * 6, 0);
            s1 = 0.0;
            step = T1(i) / cons(i);
            innerLoop = cons(i) + 1;
            // if (dynobs_pointer->ball_number > 0 && step > 0.1)
            // {
            //     step = 0.1;
            //     innerLoop = int(T1(i) / step) + 1;
            //     cons(i) = T1(i)/step;
            // }

            for (int j = 0; j < innerLoop; j++)
            {

                if (i > 0)
                {
                    t_now = plan_t + T1.head(i).sum() + s1;
                }
                else
                {
                    t_now = plan_t + s1;
                }
                s2 = s1 * s1;
                s3 = s2 * s1;
                s4 = s2 * s2;
                s5 = s4 * s1;
                beta0 << 1.0, s1, s2, s3, s4, s5;
                // cout << "mk3200" <<endl;
                beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
                beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
                beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
                beta4 << 0.0, 0.0, 0.0, 0.0, 24.0, 120.0 * s1;
                // cout << "mk3201" <<endl;
                alpha = 1.0 / cons(i) * j;
                pos = c.transpose() * beta0; // c_0->c_5
                vel = c.transpose() * beta1;
                acc = c.transpose() * beta2;
                jer = c.transpose() * beta3;
                sna = c.transpose() * beta4;

                h = acc;
                h(2) += gAcc;
                // normalizeFDF(h, zB, dzB);

                // Zb
                //  czB << 0.0, zB(2), -zB(1);
                //  cdzB << Eigen::RowVector3d::Zero(), dzB.row(2), -dzB.row(1);
                //  normalizeFDF(czB, yB, dnczB);
                //  //Yb
                //  xB = yB.cross(zB);
                //  dyB = dnczB * cdzB;
                //  dxB.col(0) = dyB.col(0).cross(zB) + yB.cross(dzB.col(0));
                //  dxB.col(1) = dyB.col(1).cross(zB) + yB.cross(dzB.col(1));
                //  dxB.col(2) = dyB.col(2).cross(zB) + yB.cross(dzB.col(2));
                //  rotM << xB, yB, zB;

                // gradSdTxyz.col(0) = dxB * jer;
                // gradSdTxyz.col(1) = dyB * jer;
                // gradSdTxyz.col(2) = dzB * jer;

                fThr = h.norm();
                dfThr = h / fThr;
                sqrMagThr = fThr * fThr;
                dSqrMagThr = 2 * h;
                // rotTrDotJer = rotM.transpose() * jer;
                // bdr = rotTrDotJer / fThr;
                // xyBdr << -bdr(1), bdr(0), 0.0;
                // sqrMagBdr = xyBdr.squaredNorm();
                // dBdr = -rotTrDotJer * dfThr.transpose() / (fThr * fThr) -
                //        rotM.transpose() * (dxB * rotTrDotJer(0) + dyB * rotTrDotJer(1) + dzB * rotTrDotJer(2)) / fThr;
                // dxyBdr << -dBdr.row(1), dBdr.row(0), Eigen::RowVector3d::Zero();
                // dSqrMagBdr = 2.0 * xyBdr.transpose() * dxyBdr;
                // dJerBdr = rotM.transpose() / fThr;
                // dJerxyBdr << -dJerBdr.row(1), dJerBdr.row(0), Eigen::RowVector3d::Zero();
                // dJerSqrMagBdr = 2.0 * xyBdr.transpose() * dJerxyBdr;

                // violaThrl = thrAccMinSqr - sqrMagThr;
                violaThrh = sqrMagThr - thrAccMaxSqr;
                // violaBdr = sqrMagBdr - bdrMaxSqr;

                omg = (j == 0 || j == innerLoop - 1) ? 0.5 : 1.0;

                idx = idxHs(i);
                K = cfgHs[idx].cols();
                // cout << "number of polydrons: "<< cfgHs.size() <<endl;
                // for (int k = 0; k < K; k++)
                // {
                //     outerNormal = cfgHs[idx].col(k).head<3>();
                //     point = cfgHs[idx].col(k).tail<3>();
                //     beta0dOuterNormalTp = beta0 * outerNormal.transpose();
                //     gradSdT = gradSdTxyz.transpose() * outerNormal;
                //     outerNormaldVel = outerNormal.dot(vel);
                //     beta2dOuterNormalTp = beta2 * outerNormal.transpose();
                //     gradSdCx = beta2dOuterNormalTp * dxB;
                //     gradSdCy = beta2dOuterNormalTp * dyB;
                //     gradSdCz = beta2dOuterNormalTp * dzB;

                //     eNormGd = (rotM.transpose() * outerNormal).array() * ellipsoid.array();
                //     eNorm = eNormGd.norm();
                //     eNormGd /= eNorm;
                //     signedDist = outerNormal.dot(pos - point) + eNorm;
                //     eNormGd.array() *= ellipsoid.array();

                //     signedDist += safeMargin;
                //     if (signedDist > 0)
                //     {
                //         signedDistSqr = signedDist * signedDist;
                //         signedDistCub = signedDist * signedDistSqr;
                //         gradSdC = beta0dOuterNormalTp +
                //                   gradSdCx * eNormGd(0) +
                //                   gradSdCy * eNormGd(1) +
                //                   gradSdCz * eNormGd(2);
                //         gradSignedDt = alpha * (outerNormaldVel +
                //                                 gradSdT(0) * eNormGd(0) +
                //                                 gradSdT(1) * eNormGd(1) +
                //                                 gradSdT(2) * eNormGd(2));
                //         gdC.block<6, 3>(i * 6, 0) += omg * step * ci(0) * 3.0 * signedDistSqr * gradSdC;
                //         gdT(i) += omg * ci(0) * (3.0 * signedDistSqr * gradSignedDt * step + signedDistCub / cons(i));
                //         pena += omg * step * ci(0) * signedDistCub;
                //     }
                // }
                for (int k = 0; k < K; k++)
                {
                    outerNormal = cfgHs[idx].col(k).head<3>();
                    violaPos = outerNormal.dot(pos - cfgHs[idx].col(k).tail<3>()) + 1.5*safeMargin;  //projection on the direction of normal vector
                    if (violaPos > 0.0 && T1(i) - step > s1 && step < s1 )
                    {
                        positiveSmoothedL1(violaPos, violaPosPena, violaPosPenaD);
                        gdC.block<6, 3>(i * 6, 0) += omg * step * ci(0) * violaPosPenaD * beta0 * outerNormal.transpose();
                        gdT(i) += omg * (ci(0) * violaPosPenaD * alpha * outerNormal.dot(vel) * step +
                                         ci(0) * violaPosPena / cons(i));
                        current_pena = omg * step  * violaPosPena;
                        pena += current_pena* ci(0);
                        pena_sta += current_pena;
                    }
                }
                // cout<<"gdT for polyH: "<< gdT<<endl;
                // cout<<"gdT for velocity: "<< gdT<<endl;
                // if (violaThrl > 0.0)
                // {
                //     violaThrlPenaD = violaThrl * violaThrl;
                //     violaThrlPena = violaThrlPenaD * violaThrl;
                //     violaThrlPenaD *= 3.0;
                //     gradViolaThrlc = -beta2 * dSqrMagThr.transpose();
                //     gradViolaThrlt = -alpha * dSqrMagThr.transpose() * jer;
                //     gdC.block<6, 3>(i * 6, 0) += omg * step * ci(2) * violaThrlPenaD * gradViolaThrlc;
                //     gdT(i) += omg * (ci(2) * violaThrlPenaD * gradViolaThrlt * step +
                //                      ci(2) * violaThrlPena / cons(i));
                //     pena += omg * step * ci(2) * violaThrlPena;
                // }

                if (violaThrh > 0.0)
                {
                    // violaThrhPenaD = violaThrh * violaThrh;
                    // violaThrhPena = violaThrhPenaD * violaThrh;
                    // violaThrhPenaD *= 3.0;
                    positiveSmoothedL1(violaThrh, violaThrhPena, violaThrhPenaD);
                    gradViolaThrhc = beta2 * dSqrMagThr.transpose();
                    gradViolaThrht = alpha * dSqrMagThr.transpose() * jer;
                    gdC.block<6, 3>(i * 6, 0) += omg * step * ci(2) * violaThrhPenaD * gradViolaThrhc;
                    gdT(i) += omg * (ci(2) * violaThrhPenaD * gradViolaThrht * step +
                                     ci(2) * violaThrhPena / cons(i));
                    pena += omg * step * ci(2) * violaThrhPena;
                }
                // cout<<"gdT for max accel: "<< gdT<<endl;

                // if (violaBdr > 0.0)
                // {
                //     // violaBdrPenaD = violaBdr * violaBdr;
                //     // violaBdrPena = violaBdrPenaD * violaBdr;
                //     // violaBdrPenaD *= 3.0;
                //     positiveSmoothedL1(violaBdr, violaBdrPena, violaBdrPenaD);
                //     gradViolaBdrc = beta2 * dSqrMagBdr.transpose() + beta3 * dJerSqrMagBdr.transpose();
                //     gradViolaBdrt = alpha * (dSqrMagBdr.dot(jer) + dJerSqrMagBdr.dot(sna));
                //     gdC.block<6, 3>(i * 6, 0) += omg * step * ci(3) * violaBdrPenaD * gradViolaBdrc;
                //     gdT(i) += omg * (ci(3) * violaBdrPenaD * gradViolaBdrt * step +
                //                      ci(3) * violaBdrPena / cons(i));
                //     pena += omg * step * ci(3) * violaBdrPena;
                // }

                // cout<<"gdT for max rotating: "<<gdT<<endl;
                // int dynn = dynobs_pointer->dyn_number;
                // cout << "mk320 " << dynobs_pointer->dyn_number<<endl<<"i: "<<i<<endl<<"j: "<<j<<endl;
                // if (i>0){
                // t_gap = T1(i-1)+s1;}
                // else{t_gap=  s1;}

                // cout<<"t_gap"
                for (int m = 0; m < dynobs_pointer->dyn_number; m++)
                {
                    if (m == 0)
                    {
                        t_gap = t_now - dynobs_pointer->time_stamp;
                    }
                    // t_gap = t_now - dynobs_pointer->time_stamp;
                    // cout << "mk321-1" <<endl<<t_gap<<endl;
                    double obj_prop_conv = pow(dynobs_pointer->max_accs[m](1) + dynobs_pointer->max_accs[m](2) * t_gap * t_gap, 0.5);
                    obj_prop_conv = obj_prop_conv > 0.5 ? 0.5 : obj_prop_conv;
                    Eigen::Vector3d conv_vec = {obj_prop_conv, obj_prop_conv, 0.0};
                    // conv_vec *= 0.5;
                    Eigen::Vector3d acc_vec = {dynobs_pointer->max_accs[m](0) * t_gap * t_gap / 2, dynobs_pointer->max_accs[m](0) * t_gap * t_gap / 2, 0};
                    ct_center = dynobs_pointer->centers[m] + t_gap * dynobs_pointer->vels[m];
                    Eigen::Vector3d check_vec = ((ct_center - pos).cwiseAbs() - dynobs_pointer->obs_sizes[m] * 0.5 - conv_vec);
                    Eigen::Vector3d check_vec_acc = check_vec - acc_vec;
                    double grad_prev_t = 0.0;
                    if ((check_vec_acc.array() < 0.0).all())
                    {
                        // Eigen::Vector3d dist_vec = pos - ct_center;
                        //   cout << "max_accs" <<endl<<dynobs_pointer->max_accs[m]<<endl;
                        Eigen::Vector3d dist_vec = pos - ct_center;
                        Eigen::Vector3d dJ_dP;
                        if ((check_vec.array() < safeMargin).all() && !mask_dyn_idx[m])  //
                        {
                            mask_dyn_idx[m] = true;
                            Eigen::Vector3d half_len = (dynobs_pointer->obs_sizes[m] * 0.5 + conv_vec).array() + safeMargin + 0.05;
                            Eigen::Vector3d axis = {dist_vec(0)>0?ct_center(0) + half_len(0): ct_center(0) - half_len(0), 
                            dist_vec(1)>0?ct_center(1) + half_len(1):ct_center(1) - half_len(1), 
                            dist_vec(2)>0?ct_center(2) + half_len(2):ct_center(2) - half_len(2)};
                            Eigen::Vector3d dist = pos - axis;
                            double dist_err2 = dist.squaredNorm();
                            current_pena = dist_err2 * omg * step;
                            pena += wei_dyn * current_pena;
                            pena_ball += current_pena;
                            dJ_dP = omg * step * wei_dyn * 2 *dist; // gradient!
                            gdC.block<6, 3>(i * 6, 0) += beta0 * dJ_dP.transpose();
                            gdT(i) += omg * wei_dyn * dist_err2 / innerLoop + dJ_dP.dot(vel ) * j / innerLoop;

//original cost and gradient:
                            // double ellip_dist2 = dist_vec(2) * dist_vec(2) * inv_z + dist_vec(0) * dist_vec(0) * inv_x + dist_vec(1) * dist_vec(1) * inv_y;
                            // double dist2_err = 2.5 - ellip_dist2;
                            // double dist2_err2 = dist2_err * dist2_err;
                            // double dist2_err3 = dist2_err2 * dist2_err;
                            // current_pena = wei_dyn * dist2_err3 * omg * step;
                            // pena += current_pena;
                            // pena_ball += current_pena;
                            // dJ_dP = omg * step * wei_dyn * 3 * dist2_err2 * (-2) * Eigen::Vector3d(inv_x * dist_vec(0), inv_y * dist_vec(1), inv_z * dist_vec(2)); // gradient!
                            // gdC.block<6, 3>(i * 6, 0) += beta0 * dJ_dP.transpose();
                            // // gdT(i) += omg * wei_dyn * dist2_err3 / innerLoop + dJ_dP.dot(vel - dynobs_pointer->vels[m]) * j / innerLoop;
                            // gdT(i) += omg * wei_dyn * dist2_err3 / innerLoop + dJ_dP.dot(vel ) * j / innerLoop;
                            grad_prev_t += dJ_dP.dot(-dynobs_pointer->vels[m]);
                        }
                        else if (t_gap < 3.0)
                        {
                            Eigen::Vector3d half_len = dynobs_pointer->obs_sizes[m] * 0.5 + conv_vec + acc_vec;
                            inv_z = 1 / (half_len(2) * half_len(2));
                            inv_x = 1 / (half_len(0) * half_len(0));
                            inv_y = 1 / (half_len(1) * half_len(1));
                            double ellip_dist2 = dist_vec(2) * dist_vec(2) * inv_z + dist_vec(0) * dist_vec(0) * inv_x + dist_vec(1) * dist_vec(1) * inv_y;
                            double dist2_err = 2.5 - ellip_dist2;
                            double dist2_err2 = dist2_err * dist2_err;
                            double dist2_err3 = dist2_err2 * dist2_err;
                            pena += wei_dyn_acc * dist2_err3 * omg * step;
                            pena_acc += wei_dyn_acc * dist2_err3 * omg * step;
                            dJ_dP = wei_dyn_acc * 3 * dist2_err2 * (-2) * Eigen::Vector3d(inv_x * dist_vec(0), inv_y * dist_vec(1), inv_z * dist_vec(2)); // gradient!
                            gdC.block<6, 3>(i * 6, 0) += beta0 * dJ_dP.transpose() * omg * step;
                            // gdT(i) += omg * (wei_dyn_acc * dist2_err3 / innerLoop + step * dJ_dP.dot(vel - dynobs_pointer->vels[m]) * j / innerLoop);
                            gdT(i) += omg * (wei_dyn_acc * dist2_err3 / innerLoop + step * dJ_dP.dot(vel ) * j / innerLoop);
                            grad_prev_t += dJ_dP.dot(-dynobs_pointer->vels[m]);
                        }

                        if (i > 0)
                        {
                            gdT.head(i).array() += omg * step * grad_prev_t;
                        }
                    }
                    // cout << "mk322" <<endl;
                }

                for (int m = 0; m < dynobs_pointer->ball_number; m++)
                {
                    if (m == 0)
                    {
                        t_gap = t_now - dynobs_pointer->ball_time_stamp;
                    }
                    // t_gap = t_now - dynobs_pointer->time_stamp;
                    // cout << "mk321" <<endl<<t_gap<<endl<<m<<"  j: "<<j<<"  innerloop:  "<<innerLoop<<endl;
                    ct_center = dynobs_pointer->ballpos[m] + t_gap * dynobs_pointer->ballvel[m] + 0.5 * t_gap * t_gap * dynobs_pointer->ballacc[m];
                    Eigen::Vector3d check_vec = ((ct_center - pos).cwiseAbs() - dynobs_pointer->ball_sizes[m] * 0.5);
                    if ((ct_center - pos).norm() < dynobs_pointer->ball_sizes[m](0) * 0.5) //((check_vec.array()<0.0).all())
                    {
                        double sa = dynobs_pointer->ball_sizes[m].squaredNorm() / 4;
                        // Eigen::Vector3d dist_vec = pos - ct_center;
                        //  cout<<"check_vec,ct_center,pos:\n"<<check_vec<<",   \n"<<ct_center<<",   \n"<<pos<<endl<<t_gap<<endl<<dynobs_pointer->ball_sizes[m](0)*0.5<<endl<<(ct_center - pos).norm()<<endl;
                        Eigen::Vector3d dist_vec = pos - ct_center;
                        double ellip_dist2 = dist_vec(2) * dist_vec(2) * inv_a2 + (dist_vec(0) * dist_vec(0) + dist_vec(1) * dist_vec(1)) * inv_b2;
                        double dist2_err = sa - ellip_dist2;
                        double dist2_err2 = dist2_err * dist2_err;
                        double dist2_err3 = dist2_err2 * dist2_err;
                        current_pena = wei_ball * dist2_err3 * omg * step;
                        pena += current_pena;
                        pena_ball += current_pena;
                        Eigen::Vector3d dJ_dP = wei_ball * 3 * dist2_err2 * (-2) * Eigen::Vector3d(inv_b2 * dist_vec(0), inv_b2 * dist_vec(1), inv_a2 * dist_vec(2)); // gradient!
                        gdC.block<6, 3>(i * 6, 0) += beta0 * dJ_dP.transpose() * omg * step;
                        gdT(i) += omg * (wei_ball * dist2_err3 / innerLoop + step * dJ_dP.dot(vel - dynobs_pointer->ballvel[m]) * j / innerLoop);
                        // cout<<"gradient: "<<beta0 * dJ_dP.transpose()* omg * step<<"\n"<<omg * (wei_ball * dist2_err3 / innerLoop + step * dJ_dP.dot(vel - dynobs_pointer->ballvel[m])*j/innerLoop)<<endl;
                        double grad_prev_t = dJ_dP.dot(-dynobs_pointer->ballvel[m]);
                        if (i > 0)
                        {
                            gdT.head(i).array() += omg * step * grad_prev_t;
                        }
                    }
                    // cout << "mk322" <<endl;
                }
                violaVel = vel.squaredNorm() - vMaxSqr;
                if (violaVel > 0.0)
                {
                    // violaVelPenaD = violaVel * violaVel;
                    // violaVelPena = violaVelPenaD * violaVel;
                    // violaVelPenaD *= 3.0;
                    positiveSmoothedL1(violaVel, violaVelPena, violaVelPenaD);
                    gradViolaVc = 2.0 * beta1 * vel.transpose();
                    gradViolaVt = 2.0 * alpha * vel.transpose() * acc;
                    gdC.block<6, 3>(i * 6, 0) += omg * step * ci(1) * violaVelPenaD * gradViolaVc;
                    gdT(i) += omg * ci(1) *(violaVelPenaD * gradViolaVt * step +violaVelPena / cons(i));
                    pena += omg * step * ci(1) * violaVelPena;
                }
                s1 += step;
            }
        }

        // end_time = clock();
        // double endtime=(double)(end_time-start_time)/CLOCKS_PER_SEC;
        // compute_time+=endtime;
        cost += pena;

        return;
    }

public:
    inline void reset(const Eigen::Matrix3d &headState,
                      const Eigen::Matrix3d &tailState,
                      const int &pieceNum)
    {
        N = pieceNum;
        headPVA = headState;
        tailPVA = tailState;
        T1.resize(N);
        A.create(6 * N, 6, 6);
        b.resize(6 * N, 3);
        gdC.resize(6 * N, 3);
        return;
    }

    inline void generate(const Eigen::MatrixXd &inPs,
                         const Eigen::VectorXd &ts)
    {
        T1 = ts;
        T2 = T1.cwiseProduct(T1);
        T3 = T2.cwiseProduct(T1);
        T4 = T2.cwiseProduct(T2);
        T5 = T4.cwiseProduct(T1);

        A.reset();
        b.setZero();

        A(0, 0) = 1.0;
        A(1, 1) = 1.0;
        A(2, 2) = 2.0;
        b.row(0) = headPVA.col(0).transpose();
        b.row(1) = headPVA.col(1).transpose();
        b.row(2) = headPVA.col(2).transpose();

        for (int i = 0; i < N - 1; i++)
        {
            A(6 * i + 3, 6 * i + 3) = 6.0;
            A(6 * i + 3, 6 * i + 4) = 24.0 * T1(i);
            A(6 * i + 3, 6 * i + 5) = 60.0 * T2(i);
            A(6 * i + 3, 6 * i + 9) = -6.0;
            A(6 * i + 4, 6 * i + 4) = 24.0;
            A(6 * i + 4, 6 * i + 5) = 120.0 * T1(i);
            A(6 * i + 4, 6 * i + 10) = -24.0;
            A(6 * i + 5, 6 * i) = 1.0;
            A(6 * i + 5, 6 * i + 1) = T1(i);
            A(6 * i + 5, 6 * i + 2) = T2(i);
            A(6 * i + 5, 6 * i + 3) = T3(i);
            A(6 * i + 5, 6 * i + 4) = T4(i);
            A(6 * i + 5, 6 * i + 5) = T5(i);
            A(6 * i + 6, 6 * i) = 1.0;
            A(6 * i + 6, 6 * i + 1) = T1(i);
            A(6 * i + 6, 6 * i + 2) = T2(i);
            A(6 * i + 6, 6 * i + 3) = T3(i);
            A(6 * i + 6, 6 * i + 4) = T4(i);
            A(6 * i + 6, 6 * i + 5) = T5(i);
            A(6 * i + 6, 6 * i + 6) = -1.0;
            A(6 * i + 7, 6 * i + 1) = 1.0;
            A(6 * i + 7, 6 * i + 2) = 2 * T1(i);
            A(6 * i + 7, 6 * i + 3) = 3 * T2(i);
            A(6 * i + 7, 6 * i + 4) = 4 * T3(i);
            A(6 * i + 7, 6 * i + 5) = 5 * T4(i);
            A(6 * i + 7, 6 * i + 7) = -1.0;
            A(6 * i + 8, 6 * i + 2) = 2.0;
            A(6 * i + 8, 6 * i + 3) = 6 * T1(i);
            A(6 * i + 8, 6 * i + 4) = 12 * T2(i);
            A(6 * i + 8, 6 * i + 5) = 20 * T3(i);
            A(6 * i + 8, 6 * i + 8) = -2.0;

            b.row(6 * i + 5) = inPs.col(i).transpose();
        }

        A(6 * N - 3, 6 * N - 6) = 1.0;
        A(6 * N - 3, 6 * N - 5) = T1(N - 1);
        A(6 * N - 3, 6 * N - 4) = T2(N - 1);
        A(6 * N - 3, 6 * N - 3) = T3(N - 1);
        A(6 * N - 3, 6 * N - 2) = T4(N - 1);
        A(6 * N - 3, 6 * N - 1) = T5(N - 1);
        A(6 * N - 2, 6 * N - 5) = 1.0;
        A(6 * N - 2, 6 * N - 4) = 2 * T1(N - 1);
        A(6 * N - 2, 6 * N - 3) = 3 * T2(N - 1);
        A(6 * N - 2, 6 * N - 2) = 4 * T3(N - 1);
        A(6 * N - 2, 6 * N - 1) = 5 * T4(N - 1);
        A(6 * N - 1, 6 * N - 4) = 2;
        A(6 * N - 1, 6 * N - 3) = 6 * T1(N - 1);
        A(6 * N - 1, 6 * N - 2) = 12 * T2(N - 1);
        A(6 * N - 1, 6 * N - 1) = 20 * T3(N - 1);

        b.row(6 * N - 3) = tailPVA.col(0).transpose();
        b.row(6 * N - 2) = tailPVA.col(1).transpose();
        b.row(6 * N - 1) = tailPVA.col(2).transpose();

        A.factorizeLU();
        A.solve(b);

        return;
    }

    inline double getTrajJerkCost() const
    {
        double objective = 0.0;
        for (int i = 0; i < N; i++)
        {
            objective += 36.0 * b.row(6 * i + 3).squaredNorm() * T1(i) +
                         144.0 * b.row(6 * i + 4).dot(b.row(6 * i + 3)) * T2(i) +
                         192.0 * b.row(6 * i + 4).squaredNorm() * T3(i) +
                         240.0 * b.row(6 * i + 5).dot(b.row(6 * i + 3)) * T3(i) +
                         720.0 * b.row(6 * i + 5).dot(b.row(6 * i + 4)) * T4(i) +
                         720.0 * b.row(6 * i + 5).squaredNorm() * T5(i);
        }
        return objective;
    }

    template <typename EIGENVEC, typename EIGENMAT>
    inline void evalTrajCostGrad(const Eigen::VectorXi &cons,
                                 const Eigen::VectorXi &idxHs,
                                 const std::vector<Eigen::MatrixXd> &cfgHs,
                                 const Eigen::Vector3d &ellipsoid,
                                 const double &safeMargin,
                                 const double &vMax,
                                 const double &thrAccMin,
                                 const double &thrAccMax,
                                 const double &bdrMax,
                                 const double &gAcc,
                                 const Eigen::Vector4d &ci,
                                 double &cost,
                                 EIGENVEC &gdT,
                                 EIGENMAT &gdInPs,
                                 const double plan_t,
                                 const dynobs_tmp *dynobs_pointer)
    {
        gdT.setZero();
        gdInPs.setZero();
        gdC.setZero();

        cost = getTrajJerkCost();
        // cout << "cost for jerk: " << cost <<endl;  //<<"b:"<<b<<endl<<"T1: "<<T1<<endl;
        addGradJbyT(gdT);
        addGradJbyC(gdC);
        // cout << "mk32,cost: " <<cost<<endl;
        addTimeIntPenalty(cons, idxHs, cfgHs, ellipsoid, safeMargin,
                          vMax, thrAccMin, thrAccMax, bdrMax,
                          gAcc, ci, cost, gdT, gdC, plan_t, dynobs_pointer);

        solveAdjGradC(gdC);
        addPropCtoT(gdC, gdT);
        addPropCtoP(gdC, gdInPs);
    }

    inline Trajectory getTraj(void) const
    {
        Trajectory traj;
        traj.reserve(N);
        for (int i = 0; i < N; i++)
        {
            traj.emplace_back(T1(i), b.block<6, 3>(6 * i, 0).transpose().rowwise().reverse());
        }
        return traj;
    }
};

class SE3GCOPTER
{
private:
    // Use C2 or Cinf diffeo
    bool c2dfm;

    // Use soft time or not
    bool softT;

    // Weight for time regularization term
    double rho;

    // Fixed total time
    double sumT;

    // Minimum Jerk Optimizer
    MINCO_S3 jerkOpt;

    // Temp variables for problem solving
    Eigen::MatrixXd iState;
    Eigen::MatrixXd fState;

    // Each col of cfgHs denotes a facet (outter_normal^T,point^T)^T
    std::vector<Eigen::MatrixXd> cfgVs;
    std::vector<Eigen::MatrixXd> cfgHs;
    Eigen::MatrixXd gdInPs;

    // Piece num for each polytope
    Eigen::VectorXi intervals;
    // Assignment vector for point in V-polytope
    Eigen::VectorXi idxVs;
    // Assignment vector for piece in H-polytope
    Eigen::VectorXi idxHs;

    int coarseN;
    int fineN;
    int dimFreeT;
    int dimFreeP;
    Eigen::VectorXd coarseT;
    Eigen::VectorXd fineT;
    Eigen::MatrixXd innerP;

    // Params for constraints
    Eigen::VectorXi cons;
    Eigen::Vector4d chi;

    Eigen::Vector3d ellipsoid;
    double safeMargin;
    double vMax;
    double thrAccMin;
    double thrAccMax;
    double bdrMax;
    double gAcc;

    // L-BFGS Solver Parameters
    lbfgs::lbfgs_parameter_t lbfgs_params;

    // for dynamic obstacles
    double plan_t;
    const dynobs_tmp *dynobs_pointer;

private:
    template <typename EIGENVEC>
    static inline void forwardT(const EIGENVEC &t,
                                Eigen::VectorXd &vecT,
                                bool soft,
                                const double &sT,
                                bool c2)
    {
        if (soft)
        {
            if (c2)
            {
                int M = vecT.size();
                for (int i = 0; i < M; i++)
                {
                    vecT(i) = t(i) > 0.0
                                  ? ((0.5 * t(i) + 1.0) * t(i) + 1.0)
                                  : 1.0 / ((0.5 * t(i) - 1.0) * t(i) + 1.0);
                }
            }
            else
            {
                vecT = t.array().exp();
            }
        }
        else
        {
            if (c2)
            {
                int Ms1 = t.size();
                for (int i = 0; i < Ms1; i++)
                {
                    vecT(i) = t(i) > 0.0
                                  ? ((0.5 * t(i) + 1.0) * t(i) + 1.0)
                                  : 1.0 / ((0.5 * t(i) - 1.0) * t(i) + 1.0);
                }
                vecT(Ms1) = 0.0;
                vecT /= 1.0 + vecT.sum();
                vecT(Ms1) = 1.0 - vecT.sum();
                vecT *= sT;
            }
            else
            {
                int Ms1 = t.size();
                vecT.head(Ms1) = t.array().exp();
                vecT(Ms1) = 0.0;
                vecT /= 1.0 + vecT.sum();
                vecT(Ms1) = 1.0 - vecT.sum();
                vecT *= sT;
            }
        }
        return;
    }

    template <typename EIGENVEC>
    static inline void backwardT(const Eigen::VectorXd &vecT,
                                 EIGENVEC &t,
                                 bool soft,
                                 bool c2)
    {
        if (soft)
        {
            if (c2)
            {
                int M = vecT.size();
                for (int i = 0; i < M; i++)
                {
                    t(i) = vecT(i) > 1.0
                               ? (sqrt(2.0 * vecT(i) - 1.0) - 1.0)
                               : (1.0 - sqrt(2.0 / vecT(i) - 1.0));
                    /*

                    t  = 2/[(tau-1)^2+1] tau<0
                       = 1/2[(tau+1)^2+1] tau>=0
                    */
                }
            }
            else
            {
                t = vecT.array().log();
            }
        }
        else
        {
            if (c2)
            {
                int Ms1 = t.size();
                t = vecT.head(Ms1) / vecT(Ms1);
                for (int i = 0; i < Ms1; i++)
                {
                    t(i) = t(i) > 1.0
                               ? (sqrt(2.0 * t(i) - 1.0) - 1.0)
                               : (1.0 - sqrt(2.0 / t(i) - 1.0));
                }
            }
            else
            {
                int Ms1 = t.size();
                t = (vecT.head(Ms1) / vecT(Ms1)).array().log();
            }
        }
        return;
    }

    template <typename EIGENVEC>
    static inline void forwardP(const EIGENVEC &p,
                                const Eigen::VectorXi &idVs,
                                const std::vector<Eigen::MatrixXd> &cfgPolyVs,
                                Eigen::MatrixXd &inP)
    {
        int M = inP.cols();
        Eigen::VectorXd q;
        int j = 0, k, idx;
        for (int i = 0; i < M; i++)
        {
            idx = idVs(i);
            k = cfgPolyVs[idx].cols() - 1;
            q = 2.0 / (1.0 + p.segment(j, k).squaredNorm()) * p.segment(j, k);
            inP.col(i) = cfgPolyVs[idx].rightCols(k) * q.cwiseProduct(q) +
                         cfgPolyVs[idx].col(0);
            j += k;
        }
        return;
    }

    static inline double objectiveNLS(void *ptrPOBs,
                                      const double *x,
                                      double *grad,
                                      const int n)
    {
        const Eigen::MatrixXd &pobs = *(Eigen::MatrixXd *)ptrPOBs;
        Eigen::Map<const Eigen::VectorXd> p(x, n);
        Eigen::Map<Eigen::VectorXd> gradp(grad, n);

        double qnsqr = p.squaredNorm();
        double qnsqrp1 = qnsqr + 1.0;
        double qnsqrp1sqr = qnsqrp1 * qnsqrp1;
        Eigen::VectorXd r = 2.0 / qnsqrp1 * p;

        Eigen::Vector3d delta = pobs.rightCols(n) * r.cwiseProduct(r) +
                                pobs.col(1) - pobs.col(0);
        double cost = delta.squaredNorm();
        Eigen::Vector3d gradR3 = 2 * delta;

        Eigen::VectorXd gdr = pobs.rightCols(n).transpose() * gradR3;
        gdr = gdr.array() * r.array() * 2.0;
        gradp = gdr * 2.0 / qnsqrp1 -
                p * 4.0 * gdr.dot(p) / qnsqrp1sqr;

        return cost;
    }

    template <typename EIGENVEC>
    static inline void backwardP(const Eigen::MatrixXd &inP,
                                 const Eigen::VectorXi &idVs,
                                 const std::vector<Eigen::MatrixXd> &cfgPolyVs,
                                 EIGENVEC &p)
    {
        int M = inP.cols();
        int j = 0, k, idx;

        // Parameters for tiny nonlinear least squares
        double minSqrD;
        lbfgs::lbfgs_parameter_t nls_params;
        lbfgs::lbfgs_load_default_parameters(&nls_params);
        nls_params.g_epsilon = FLT_EPSILON;
        nls_params.max_iterations = 128;

        Eigen::MatrixXd pobs;
        for (int i = 0; i < M; i++)
        {
            idx = idVs(i);
            k = cfgPolyVs[idx].cols() - 1;
            p.segment(j, k).setConstant(1.0 / (sqrt(k + 1.0) + 1.0));
            pobs.resize(3, k + 2);
            pobs << inP.col(i), cfgPolyVs[idx];
            lbfgs::lbfgs_optimize(k,
                                  p.data() + j,
                                  &minSqrD,
                                  &SE3GCOPTER::objectiveNLS,
                                  nullptr,
                                  nullptr,
                                  &pobs,
                                  &nls_params);

            j += k;
        }

        return;
    }

    template <typename EIGENVEC>
    static inline void addLayerTGrad(const Eigen::VectorXd &t,
                                     EIGENVEC &gradT,
                                     bool soft,
                                     const double &sT,
                                     bool c2)
    {
        if (soft)
        {
            if (c2)
            {
                int M = t.size();
                double denSqrt;
                for (int i = 0; i < M; i++)
                {
                    if (t(i) > 0)
                    {
                        gradT(i) *= t(i) + 1.0;
                    }
                    else
                    {
                        denSqrt = (0.5 * t(i) - 1.0) * t(i) + 1.0;
                        gradT(i) *= (1.0 - t(i)) / (denSqrt * denSqrt);
                    }
                }
            }
            else
            {
                int M = t.size();
                gradT.head(M).array() *= t.array().exp();
            }
        }
        else
        {
            if (c2)
            {
                int Ms1 = t.size();
                Eigen::VectorXd gFree = sT * gradT.head(Ms1);
                double gTail = sT * gradT(Ms1);
                Eigen::VectorXd dExpTau(Ms1);
                double expTauSum = 0.0, gFreeDotExpTau = 0.0;
                double denSqrt, expTau;
                for (int i = 0; i < Ms1; i++)
                {
                    if (t(i) > 0)
                    {
                        expTau = (0.5 * t(i) + 1.0) * t(i) + 1.0;
                        dExpTau(i) = t(i) + 1.0;
                        expTauSum += expTau;
                        gFreeDotExpTau += expTau * gFree(i);
                    }
                    else
                    {
                        denSqrt = (0.5 * t(i) - 1.0) * t(i) + 1.0;
                        expTau = 1.0 / denSqrt;
                        dExpTau(i) = (1.0 - t(i)) / (denSqrt * denSqrt);
                        expTauSum += expTau;
                        gFreeDotExpTau += expTau * gFree(i);
                    }
                }
                denSqrt = expTauSum + 1.0;
                gradT.head(Ms1) = (gFree.array() - gTail) * dExpTau.array() / denSqrt -
                                  (gFreeDotExpTau - gTail * expTauSum) * dExpTau.array() / (denSqrt * denSqrt);
                gradT(Ms1) = 0.0;
            }
            else
            {
                int Ms1 = t.size();
                Eigen::VectorXd gFree = sT * gradT.head(Ms1);
                double gTail = sT * gradT(Ms1);
                Eigen::VectorXd expTau = t.array().exp();
                double expTauSum = expTau.sum();
                double denom = expTauSum + 1.0;
                gradT.head(Ms1) = (gFree.array() - gTail) * expTau.array() / denom -
                                  (gFree.dot(expTau) - gTail * expTauSum) * expTau.array() / (denom * denom);
                gradT(Ms1) = 0.0;
            }
        }
        return;
    }

    template <typename EIGENVEC_0, typename EIGENVEC_1>
    static inline void addLayerPGrad(EIGENVEC_0 &p,
                                     const Eigen::VectorXi &idVs,
                                     const std::vector<Eigen::MatrixXd> &cfgPolyVs,
                                     const Eigen::MatrixXd &gradInPs,
                                     EIGENVEC_1 &grad)
    {
        int M = gradInPs.cols();

        int j = 0, k, idx;
        double qnsqr, qnsqrp1, qnsqrp1sqr;
        Eigen::VectorXd q, r, gdr;
        for (int i = 0; i < M; i++)
        {
            idx = idVs(i);
            k = cfgPolyVs[idx].cols() - 1;

            q = p.segment(j, k);
            qnsqr = q.squaredNorm();
            qnsqrp1 = qnsqr + 1.0;
            qnsqrp1sqr = qnsqrp1 * qnsqrp1;
            r = 2.0 / qnsqrp1 * q;
            gdr = cfgPolyVs[idx].rightCols(k).transpose() * gradInPs.col(i);
            gdr = gdr.array() * r.array() * 2.0;

            grad.segment(j, k) = gdr * 2.0 / qnsqrp1 -
                                 q * 4.0 * gdr.dot(q) / qnsqrp1sqr;

            j += k;
        }

        return;
    }

    static inline void splitToFineT(const Eigen::VectorXd &cT,
                                    const Eigen::VectorXi &intervs,
                                    Eigen::VectorXd &fT)
    {
        int M = intervs.size();
        int offset = 0;
        int inverv;
        for (int i = 0; i < M; i++)
        {
            inverv = intervs(i);
            fT.segment(offset, inverv).setConstant(cT(i) / inverv);
            offset += inverv;
        }
        return;
    }

    static inline void mergeToCoarseGradT(const Eigen::VectorXi &intervs,
                                          Eigen::VectorXd &fineGdT)
    {
        int M = intervs.size();
        int offset = 0;
        int inverv;
        for (int i = 0; i < M; i++)
        {
            inverv = intervs(i);
            fineGdT(i) = fineGdT.segment(offset, inverv).mean();
            offset += inverv;
        }
        return;
    }

    static inline double objectiveFunc(void *ptrObj,
                                       const double *x,
                                       double *grad,
                                       const int n)
    {
        SE3GCOPTER &obj = *(SE3GCOPTER *)ptrObj;
        const int dimT = obj.dimFreeT;
        const int dimP = obj.dimFreeP;
        const double rh = obj.rho;
        Eigen::Map<const Eigen::VectorXd> t(x, dimT);
        Eigen::Map<const Eigen::VectorXd> p(x + dimT, dimP);
        Eigen::Map<Eigen::VectorXd> gradt(grad, dimT);
        Eigen::VectorXd proxyGradT(obj.fineN);
        Eigen::Map<Eigen::VectorXd> gradp(grad + dimT, dimP);
        // cout<<"gradt: "<<gradt<<endl; //<<"gradp:"<<gradp<<endl
        forwardT(t, obj.coarseT, obj.softT, obj.sumT, obj.c2dfm);
        splitToFineT(obj.coarseT, obj.intervals, obj.fineT);
        forwardP(p, obj.idxVs, obj.cfgVs, obj.innerP);

        obj.jerkOpt.cost_total = 0;
        obj.jerkOpt.generate(obj.innerP, obj.fineT);
        // cout << "mk31" <<endl;
        obj.jerkOpt.evalTrajCostGrad(obj.cons, obj.idxHs, obj.cfgHs, obj.ellipsoid,
                                     obj.safeMargin, obj.vMax, obj.thrAccMin,
                                     obj.thrAccMax, obj.bdrMax, obj.gAcc, obj.chi,
                                     obj.jerkOpt.cost_total, proxyGradT, obj.gdInPs, obj.plan_t, obj.dynobs_pointer);

        obj.jerkOpt.cost_total += rh * obj.coarseT.sum();
        proxyGradT.array() += rh;

        mergeToCoarseGradT(obj.intervals, proxyGradT);
        // grad of T
        // T,P->tau kesi
        addLayerTGrad(t, proxyGradT, obj.softT, obj.sumT, obj.c2dfm);
        addLayerPGrad(p, obj.idxVs, obj.cfgVs, obj.gdInPs, gradp);

        gradt = proxyGradT.head(dimT);
        // cout << "final cost: " << cost <<endl;
        return obj.jerkOpt.cost_total;
    }

public:
    inline void gridMesh(const Eigen::Matrix3d &iState,
                         const Eigen::Matrix3d &fState,
                         const std::vector<Eigen::MatrixXd> &cfgPolyVs,
                         const double &gridResolution,
                         Eigen::VectorXi &intervalsVec) const
    {
        int M = intervalsVec.size();

        int curInterval, k;
        Eigen::Vector3d lastP, curP;
        curP = iState.col(0);
        for (int i = 0; i < M - 1; i++)
        {
            lastP = curP;
            k = cfgPolyVs[2 * i + 1].cols() - 1;
            curP = cfgPolyVs[2 * i + 1].rightCols(k).rowwise().sum() / (1.0 + k) +
                   cfgPolyVs[2 * i + 1].col(0);
            curInterval = ceil((curP - lastP).norm() / gridResolution);
            intervalsVec(i) = curInterval > 0 ? curInterval : 1;
        }
        lastP = curP;
        curP = fState.col(0);
        curInterval = ceil((curP - lastP).norm() / gridResolution);
        intervalsVec(M - 1) = curInterval > 0 ? curInterval : 1;

        return;
    }

    inline bool extractVs(const std::vector<Eigen::MatrixXd> &hPs,
                          std::vector<Eigen::MatrixXd> &vPs) const
    {
        const int M = hPs.size() - 1;

        vPs.clear();
        vPs.reserve(2 * M + 1);

        int nv;
        Eigen::MatrixXd curIH, curIV, curIOB;
        for (int i = 0; i < M; i++)
        {
            if (!geoutils::enumerateVs(hPs[i], curIV))
            {
                return false;
            }
            nv = curIV.cols();
            curIOB.resize(3, nv);
            curIOB << curIV.col(0), curIV.rightCols(nv - 1).colwise() - curIV.col(0);
            vPs.push_back(curIOB);

            curIH.resize(6, hPs[i].cols() + hPs[i + 1].cols());
            curIH << hPs[i], hPs[i + 1];
            if (!geoutils::enumerateVs(curIH, curIV))
            {
                return false;
            }
            nv = curIV.cols();
            curIOB.resize(3, nv);
            curIOB << curIV.col(0), curIV.rightCols(nv - 1).colwise() - curIV.col(0);
            vPs.push_back(curIOB);
        }

        if (!geoutils::enumerateVs(hPs.back(), curIV))
        {
            return false;
        }
        nv = curIV.cols();
        curIOB.resize(3, nv);
        curIOB << curIV.col(0), curIV.rightCols(nv - 1).colwise() - curIV.col(0);
        vPs.push_back(curIOB);

        return true;
    }

    inline bool setup(const double &rh,
                      const double &st,
                      const Eigen::MatrixXd &iniState,
                      const Eigen::MatrixXd &finState,
                      const std::vector<Eigen::MatrixXd> &cfgPolyHs,
                      const double &gridRes,
                      const int &itgSpaces,
                      const double &horiHalfLen, // 0
                      const double &vertHalfLen, // 0
                      const double &margin,      // 0
                      const double &vm,
                      const double &minThrAcc,
                      const double &maxThrAcc,
                      const double &bodyRateMax,
                      const double &g,
                      const Eigen::Vector4d &w,
                      bool c2diffeo,
                      const double plan_start_t,
                      const dynobs_tmp *dyn_pointer)
    {
        // Setup for optimization parameters
        // cout << "mk1" <<endl;
        c2dfm = c2diffeo;

        softT = rh > 0;
        if (softT)
        {
            rho = rh;
            sumT = 1.0; // optimize total time
        }
        else
        {
            rho = 0.0;
            sumT = st;
        }

        iState = iniState;
        fState = finState;
        plan_t = plan_start_t;
        dynobs_pointer = dyn_pointer;
        // cout << dynobs_pointer << endl;
        cfgHs = cfgPolyHs;
        coarseN = cfgHs.size();
        for (int i = 0; i < coarseN; i++)
        {
            cfgHs[i].topRows<3>().colwise().normalize(); // 6*n
        }
        if (!extractVs(cfgHs, cfgVs))
        {
            return false;
        }

        intervals.resize(coarseN);
        gridMesh(iState, fState, cfgVs, gridRes, intervals); // cfgVs dimension:2n-1
        fineN = intervals.sum();
        cons.resize(fineN);
        cons.setConstant(itgSpaces);

        idxVs.resize(fineN - 1);
        idxHs.resize(fineN);
        dimFreeT = softT ? coarseN : coarseN - 1; // softT is true tau size is n
        dimFreeP = 0;
        int offset = 0, interval;
        for (int i = 0; i < coarseN; i++)
        {
            interval = intervals(i);
            for (int j = 0; j < interval; j++)
            {
                if (j < interval - 1)
                {
                    idxVs(offset) = 2 * i;
                    dimFreeP += cfgVs[2 * i].cols() - 1;
                }
                else if (i < coarseN - 1)
                {
                    idxVs(offset) = 2 * i + 1;
                    dimFreeP += cfgVs[2 * i + 1].cols() - 1;
                }
                idxHs(offset) = i;
                offset++;
            }
        }

        chi = w;
        ellipsoid(0) = horiHalfLen;
        ellipsoid(1) = horiHalfLen;
        ellipsoid(2) = vertHalfLen;
        safeMargin = margin;
        vMax = vm;
        thrAccMin = minThrAcc;
        thrAccMax = maxThrAcc;
        bdrMax = bodyRateMax;
        gAcc = g;

        // Make a legal initial speed
        double tempNorm;
        tempNorm = iState.col(1).norm();
        iState.col(1) *= tempNorm > vMax ? (vMax / tempNorm) : 1.0;
        tempNorm = fState.col(1).norm();
        fState.col(1) *= tempNorm > vMax ? (vMax / tempNorm) : 1.0;

        // Setup for L-BFGS solver
        lbfgs::lbfgs_load_default_parameters(&lbfgs_params);

        // Allocate temp variables
        coarseT.resize(coarseN);
        fineT.resize(fineN);
        innerP.resize(3, fineN - 1);
        gdInPs.resize(3, fineN - 1);
        jerkOpt.reset(iniState, finState, fineN);
        //
        // hzcmy load param

        return true;
    }

    inline void setInitial(const std::vector<Eigen::MatrixXd> &cfgPolyVs,
                           const Eigen::VectorXi &intervs,
                           Eigen::VectorXd &vecT,
                           Eigen::MatrixXd &vecInP) const
    {
        constexpr double maxSpeedForAllocatiion = 10.0;

        int M = vecT.size();
        Eigen::Vector3d lastP, curP, delta;
        int offset, interv, k;

        offset = 0;
        curP = iState.col(0);
        for (int i = 0; i < M - 1; i++)
        {
            lastP = curP;
            interv = intervs(i);
            k = cfgPolyVs[2 * i + 1].cols() - 1;
            curP = cfgPolyVs[2 * i + 1].rightCols(k).rowwise().sum() / (1.0 + k) +
                   cfgPolyVs[2 * i + 1].col(0);
            delta = curP - lastP;
            vecT(i) = delta.norm() / std::min(vMax, maxSpeedForAllocatiion);
            delta /= interv;
            for (int j = 0; j < interv; j++)
            {
                vecInP.col(offset++) = (j + 1) * delta + lastP;
            }
        }
        interv = intervs(M - 1);
        lastP = curP;
        curP = fState.col(0);
        delta = curP - lastP;
        vecT(M - 1) = delta.norm() / std::min(vMax, maxSpeedForAllocatiion);
        delta /= interv;
        for (int j = 0; j < interv - 1; j++)
        {
            vecInP.col(offset++) = (j + 1) * delta + lastP;
        }

        return;
    }

    inline double optimize(Trajectory &traj,
                           const double &relCostTol)
    {
        double *x = new double[dimFreeT + dimFreeP];
        Eigen::Map<Eigen::VectorXd> t(x, dimFreeT);
        Eigen::Map<Eigen::VectorXd> p(x + dimFreeT, dimFreeP);

        setInitial(cfgVs, intervals, coarseT, innerP); // initialize the waypoint innerP and time vector  coarseT.

        backwardT(coarseT, t, softT, c2dfm);
        backwardP(innerP, idxVs, cfgVs, p);

        double minObjectivePenalty;
        lbfgs_params.mem_size = 8; // default
        lbfgs_params.past = 3;
        // lbfgs_params.delta = 1.0e-4;
        lbfgs_params.g_epsilon = 1.0e-5; // default
        lbfgs_params.min_step = 1.0e-20; // default
        lbfgs_params.max_iterations = 100;
        lbfgs_params.abs_curv_cond = 0;
        lbfgs_params.delta = relCostTol;
        // cout << "mk2" <<endl;
        // cout << dynobs_pointer << endl;
        cout << "gcopter dynamic obs number:" << dynobs_pointer->dyn_number << "///" << dynobs_pointer->ball_number << endl;
        lbfgs::lbfgs_optimize(dimFreeT + dimFreeP,
                              x,
                              &minObjectivePenalty,
                              &SE3GCOPTER::objectiveFunc,
                              nullptr,
                              nullptr,
                              this,
                              &lbfgs_params);

        // cout << "mk3" <<endl;

        forwardT(t, coarseT, softT, sumT, c2dfm);  //update final coarseT
        splitToFineT(coarseT, intervals, fineT);
        forwardP(p, idxVs, cfgVs, innerP);  //update the innerP

        cout << "Total cost (include time): " << jerkOpt.cost_total << "\npena: " << jerkOpt.pena << "\npena_staticObs: " << jerkOpt.pena_sta << "\npena_dynobjects: " << jerkOpt.pena_ball << "\npena_dyn_accdistrib: " << jerkOpt.pena_acc << endl;
        // std::cout<<"-------------------------------------\n";
        // std::cout<<"total time is "<<jerkOpt.compute_time<<" s";

        if (jerkOpt.pena_sta + jerkOpt.pena_ball > 0.5)
        {
            cout << "Optimized trajectory is not safe?"<<endl;
        }
        //     cout << "Traj re-optimize!" << endl;
        //     setInitial(cfgVs, intervals, coarseT, innerP); // initialize the waypoint innerP and time vector  coarseT.

        //     backwardT(coarseT, t, softT, c2dfm);
        //     backwardP(innerP, idxVs, cfgVs, p);
        //     double pena_coe = 2;
        //     jerkOpt.dyn_coe = 6.0;
        //     chi(0) *= pena_coe;
        //     rho = 1.0;
        //     chi(1) *= 0.5;
        //     chi(2) *= 1.0;
        //     lbfgs_params.delta *= 0.01;
        //     lbfgs::lbfgs_optimize(dimFreeT + dimFreeP,
        //                           x,
        //                           &minObjectivePenalty,
        //                           &SE3GCOPTER::objectiveFunc,
        //                           nullptr,
        //                           nullptr,
        //                           this,
        //                           &lbfgs_params);

        //     // cout << "mk3" <<endl;

        //     forwardT(t, coarseT, softT, sumT, c2dfm); // update final coarseT
        //     splitToFineT(coarseT, intervals, fineT);
        //     forwardP(p, idxVs, cfgVs, innerP); // update the innerP
        //     cout << "pena_staticObs: " << jerkOpt.pena_sta / pena_coe << "\npena_dynobjects: " << jerkOpt.pena_ball  / pena_coe << "\npena_dyn_accdistrib: " << jerkOpt.pena_acc / pena_coe << endl;
        // }
        delete[] x;
        jerkOpt.generate(innerP, fineT); // generate final trajectory
        traj = jerkOpt.getTraj();
        return jerkOpt.getTrajJerkCost();
    }
};

#endif
