
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>
 #include "fftw3.h"
#include <eigen3/Eigen/Dense>
#include <chrono>
#define PI 3.1415926
using namespace std;
int main()
{
    // Eigen::VectorXd input;
    int dim = 4; 
	// const int len = 128;
    int n[4] = {10,10,10,2};
    // n = (int *)fftw_malloc(sizeof(int)*4);
    // input.resize(len);
    // Eigen::Matrix<double, 1, len, Eigen::RowMajor> input;
    // double *in = NULL;
    double in1[n[0]][n[1]][n[2]][n[3]], in2[n[0]][n[1]][n[2]][n[3]],in3[n[0]][n[1]][n[2]][n[3]];
    // double ****in3;
//    in3 = new double ***[n[0]];
// for( int i=0; i<n[0]; i++ )
// {
//     in3[i] = new double **[n[1]];
//     for( int j=0; j<n[1]; j++ )
//     {
//          in3[i][j] = new double *[n[2]];
//              for( int k=0; k<n[2]; k++ )
//     {
//         in3[i][j][k] = new double [n[3]];
//     }
//     }
// }
	// double *in = NULL;
	fftw_complex *out1 = NULL, *out2 = NULL,*out3 = NULL;// fftwf_complex --> 即为float版本
	fftw_plan p,p_inv;
    const int peak_num = 4;
    double peaks[peak_num] = {0,0,0,0};
    vector<vector<int>>index(peak_num,vector<int>(3));
	// in  = (double *)fftw_malloc(sizeof(double) * n[0]*(n[1])*(n[2])*(n[3]));
	out1 = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) *  n[0]*(n[1])*(n[2])*(n[3]));
    out2 = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) *  n[0]*(n[1])*(n[2])*(n[3]));
    out3 = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) *  n[0]*(n[1])*(n[2])*(n[3]));
    p = fftw_plan_dft_r2c(dim, (int *)&n, (double *)&in1, out1, FFTW_MEASURE);
    p_inv = fftw_plan_dft_c2r(dim, (int *)&n,  out3, (double *)in3,FFTW_MEASURE);
	// double dx = 1.0 / len;
 
	// 输入纯实数
	// for (int i = 0; i < len; i++)
	// {
	// 	// in[i] = sin(2*PI * dx*i) + sin(4*PI * dx*i);
    //     input(i) =  sin(2*PI * dx*i) + sin(4*PI * dx*i);
    //     printf("%.2f ", input(i));
	// 	// printf("%.2f ", in[i]);
	// }
    for (int i = 0; i < n[0]; i++)
	{
            for (int j = 0; j < n[1]; j++)
	{
            for (int k = 0; k < n[2]; k++)
	{
         for (int m= 0; m < n[3]; m++)
         {
            if (i>2 &&  j>2 && k>1 && i < n[0]-1 && j < n[1]-1 && k < n[2]-2)
           in1[i][j][k][m] = 1;
           else in1[i][j][k][m] = 0;
            if (i>=0 &&  j>=0 && k>1 && i < n[0]-4 && j < n[1]-4 && k < n[2]-2)
           in2[i][j][k][m] = 1;
           else in2[i][j][k][m] = 0;
        //    printf("%.2f ", in1[i][j][k][m]);
         }
    }}}
	// printf("\n\n");
    // in = (double *) &input;
	// 傅里叶变换
    chrono::high_resolution_clock::time_point tic = chrono::high_resolution_clock::now();

	fftw_execute(p);
    fftw_execute_dft_r2c(p,  (double *)&in2, out2);

    // 输出幅度谱
	// for (int i = 0; i < len; i++)
	// {
	// 	float len = sqrt(out[i][0]*out[i][0] + out[i][1]*out[i][1]);
	// 	printf("%.2f ", len);
	// }
    for (int i = 0; i <n[0]*(n[1])*(n[2])*(n[3]); i++)
    {
        out3[i][0] = out1[i][0]*out2[i][0] + out1[i][1]*out2[i][1]; //ac-bd, conjugate: b = -b
        out3[i][1] = -out1[i][1]*out2[i][0] + out1[i][0]*out2[i][1];  //bc+ad
        // printf("{%.2f ,  %.2f}", out3[i][0],out3[i][1]);
    }
    fftw_execute(p_inv);
    std::cout<<in3[0][0]<<std::endl;
    double max = 0;
    int ii,jj,kk;
    for (int i = 0; i < n[0]; i++)
	{
            for (int j = 0; j < n[1]; j++)
	{
            for (int k = 0; k < n[2]; k++)
	{
        double value = 0;
         for (int m= 0; m < n[3]; m++)
         {
             value += in3[i][j][k][m];}
            //  if(in3[i][j][k][m]  > max)
            //  int id = 0;
            //  while (in3[i][j][k][m]  > peaks[id])
             for (int id = 0; id<peak_num; id++)
             {  if  (value  > peaks[id])
                 {peaks[id] = value;
                  index[id] ={ i,j,k};
                 break;
                 }
            //   id ++;
            //   ii = i; jj=j;kk=k;
             }
        //    std::cout << in3[i][j][k][m] << "  ";
         }}}
    chrono::high_resolution_clock::time_point toc = chrono::high_resolution_clock::now();
    double compTime = chrono::duration_cast<chrono::microseconds>(toc - tic).count() * 1.0e-3;
	std::cout << "time cost (ms)： " <<compTime<<std::endl;
    // std::cout << "\n"<<max<<"\n"<<ii<<"\n"<<jj<<"\n"<<kk<<std::endl;
    for (int i=0;i<peak_num;i++)
    std::cout <<"\n"<<peaks[i] << "\n"<<index[i][0]<< "\n"<<index[i][1]<< "\n"<<index[i][2]<<std::endl;
    std::cout << &out1 << "  "<<&out2<<std::endl;
	printf("\n");
 
//    Eigen::Vector3d test;
//    test<<1,2;
//    std::cout <<test<<std::endl;
	// 释放资源
	// fftw_destroy_plan(p);
	// fftw_free((double *)&in1);
	// fftw_free(out1);
    // int test[2][3][6];
    // int* t2;
    // t2 = (int *)&test;
    // t2[1][2][3] = 6;
	return 0;
}