#include <iostream>
#include <sstream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "fileIO.hpp"

using namespace std;
using namespace Eigen;

#ifndef _MATRIX_
#define _MATRIX_
typedef Matrix<double, 3, 4> Matrix34d;
#endif

namespace bundleadjustment
{
    class BundleAdjustment
    {
    private:
        int pointN;     //３次元点数
        int cameraN;    //カメラの数
        double f0;      //スケール
        double c;       //レーベンバーグ・マーカート法の定数

        int In;            //カメラに写った点の数
        int** I = NULL;    //第ｋ画像にa点が写っているとき１
        double** x = NULL; //x
        double** y = NULL; //y

        //変数
        Vector3d* X = NULL;;        //３次元点
        Matrix34d* cameraMat = NULL;//カメラ行列
        double*f = NULL;            //焦点距離
        double*u = NULL;            //光軸点
        double*v = NULL;            //光軸点
        Vector3d* t = NULL;         //並進
        Matrix3d* R = NULL;         //回転


        //誤差
        double error;       //再投影誤差
        double errorTilde;  //再投影誤差
        double* derror;     //1次微分

        double** p = NULL;  //1，2次微分計算用
        double** q = NULL;  //1，2次微分計算用
        double** r = NULL;  //1，2次微分計算用

        Matrix3d* hE = NULL;//点に関する2次微分
        MatrixXd* hF = NULL;//点とカメラパラメータに関する2次微分
        MatrixXd hG;        //カメラパラメータに関する2次微分

        VectorXd deltaXiP;  //方程式の解 点に関する
        VectorXd deltaXiF;  //方程式の解 カメラパラメータに関する
        VectorXd dP;        //1次微分 点に関する
        VectorXd dF;        //1次微分 カメラパラメータに関する

    public:
        BundleAdjustment();
        ~BundleAdjustment();


        void calcpqr();
        void calcError();

        Vector3d calcdpqrak(int alpha, int kappa, int l);
        void calcdError();

        void calcddError();
        void calchE();
        void calchF();
        void calchG();
        double calchkl(int k, int l, int alpha, int kappa);
        double calchEkl(int k, int l, int alpha);
        double calchFkl(int k, int l);
        double calchGkl(int k, int l);
        int offset(int k);

        /*solve*/
        void solveEquations();
        void setdPdF();
		void solveEquationsdXiF();
		void solveEquationsdXiP();

        /*tilde*/
        void calcErrorTilde();
		Vector3d calcpqrTildeak(int alpha, int kappa);
		double getdeltaXiF(int kappa, int param);
		Matrix3d getRotateMat(Vector3d l);

        void renewParams();

        /**/
        void bundleAdjustment(
            string file_xy,
            string file_3d,
            string file_K,
            string file_R,
            string file_t,
            int width,
            int height,
            double f0);
        void setParams(
            string file_xy,
            string file_3d,
            string file_K,
            string file_R,
            string file_t,
            int width,
            int height,
            double f0);
        void init();


        //set
        void setX(int alpha, Vector3d X);
        void setf(int kappa, double f);
        void setu(int kappa, double u);
        void setv(int kappa, double v);
        void sett(int kappa, Vector3d t);
        void setR(int kappa, Matrix3d R);
        void setCameraMat(int kappa, Matrix34d P);

    };
}
