
#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "BundleAdjustment.hpp"
#include "fileIO.hpp"

using namespace std;
using namespace Eigen;
using namespace bundleadjustment;

#ifndef _MATRIX_
#define _MATRIX_
typedef Matrix<double, 3, 4> Matrix34d;
#endif



//パラメータをセットする
void BundleAdjustment::setParams(
    string file_xy,
    string file_3d,
    string file_K,
    string file_R,
    string file_t,
    int width,
    int height,
    double f0
)
{
    /*
        ファイル読み込み
    */
    vector< vector<Vector2d> > data_xy;
    vector<Vector3d> data_3d;
    vector<Matrix3d> data_R;
    vector<Matrix3d> data_K;
    vector<Vector3d> data_t;
    readDataxy(data_xy, file_xy);
    readDataVec3d(data_3d, file_3d);
    readDataMat3d(data_K, file_K);
    readDataMat3d(data_R, file_R);
    readDataVec3d(data_t, file_t);

    int N = data_xy.size();
    int M = data_xy[0].size();
    this->f0 = f0;
    this->pointN = N;
    this->cameraN = M;
    cout << "camera N = " << M << "  point N = " << N << endl;

    //配列確保
    init();

    cout << "set xy" << endl;
    //x,y を挿入 Inを数える
    //左上を原点と仮定する
    int id;
    int cnti = 0;
    for(int kappa = 0; kappa < M; kappa++){
        id = kappa;
        for(int alpha = 0; alpha < N; alpha++){
            this->x[kappa][alpha] = -data_xy[alpha][id](1) + height/2;
            this->y[kappa][alpha] =  data_xy[alpha][id](0) - width/2;
            if(data_xy[alpha][id](0) >= 0){
                this->I[kappa][alpha] = 1;
                cnti++;
            }else{
                this->I[kappa][alpha] = 0;
            }
        }
    }
    this->In = cnti;


    /*
        正規化
    */
    cout << "normalization" << endl;
    Vector3d t1 = data_t[0];
    Vector3d t2 = data_t[1];
    Matrix3d R1 = data_R[0];
    Vector3d j;
    j << 0, 1, 0;
    double s = j.dot(R1.transpose() * (t2 -t1));
    //t
    for(int i = 0; i < M; i++){
        data_t[i] = 1.0 / s * R1.transpose() * (data_t[i]- t1);
        sett(i, data_t[i]);
    }
    //Matrix3d R
    for(int i = 0; i < M; i++){
        data_R[i] = R1.transpose() * data_R[i];
        setR(i, data_R[i]);
    }
    for(int i = 0; i < N; i++){
        data_3d[i] = 1.0 / s * R1.transpose() * (data_3d[i] - t1);
        setX(i, data_3d[i]);
    }
    double u, v, f;
    Matrix34d P;
    for(int i = 0; i < M; i++){
        f = data_K[i](0, 0);
        u = data_K[i](0, 2);
        v = data_K[i](1, 2);
        setf(i, f);
        setu(i, u);
        setv(i, v);
        P << data_K[i] * data_R[i].transpose(), -data_K[i] * data_R[i].transpose() * data_t[i];
        setCameraMat(i, P);
    }

    cout << "output ply" << endl;
    outDataPly("init_3d.ply", data_3d.data(), data_3d.size());
}

/*
    手順11.１ バンドル調整
*/
void BundleAdjustment::bundleAdjustment(
    string file_xy,
    string file_3d,
    string file_K,
    string file_R,
    string file_t,
    int width,
    int height,
    double f0)
{
    double e = 0;       //再投影誤差
    double eTilde = 0;  //再投影誤差
    //double delta = this->In * pow(0.01, 2) / pow(this->f0, 2);  //ループを抜けるしきい値
    double delta = 0.0001;
    cout << "delta = " << delta << endl;

    //手順11.1 1． 初期値X_α，f_k,(u_k,v_k)，t_k，R_kを与え，それに対応する再投影誤差Eを計算し，c=0.0001と置く
    setParams(file_xy, file_3d, file_K, file_R, file_t, width, height, f0);    //初期値を与える
    calcpqr();      //再投影誤差Eを計算するためにpqrを計算する
    calcError();    //再投影誤差Eを計算する
    e = this->f0 * sqrt(this->error / (2*this->In - (3*this->pointN + 9*this->cameraN - 7)) );
    this->c = 0.0001;

    cout << "e     = " << e << "\n";
    cout << "c     = " << this->c << "\n";

    int loop = 0;   //ループ回数
    while (true) {
        cout << "loop = " << loop << "\n";

        //手順11.1 2． １階および２階微分∂E/∂xi_k，∂^2E/∂xi_k*∂xi_l(k,l=1,...,3N+9M-7)を計算する
        calcdError();   //1階微分の計算 ∂E/∂xi_k
        calcddError();  //2階微分の計算 ∂^2E/∂xi_k*∂xi_l


        do {
            //手順11.1 3. 連立1次方程式をといてΔxi_k(k=1,...,3N+9M-7)を計算する
            solveEquations();   //連立1次方程式を解く

            //手順11.1 4. X_α~，f_k~,(u_k~,v_k~)，t_k~，R_k~を更新する
            //手順11.1 5. X_α~，f_k~,(u_k~,v_k~)，t_k~，R_k~に対するE~を計算し，
            //            E~>Eならc<-10cとしてステップ3に戻る
            calcErrorTilde();   //更新ようにX_α~，f_k~,(u_k~,v_k~)，t_k~，R_k~をE~を計算する

            eTilde = this->f0 * sqrt(this->errorTilde / (2*this->In - (3*this->pointN + 9*this->cameraN - 7)));
            cout << "e     = " << e << "\n";
            cout << "e~    = " << eTilde << "\n";

            if ( eTilde > e ) {
                this->c = 10.0 * this->c;
                cout << "c     = " << this->c << "\n";
                calcdError();
                calcddError();  //2階微分の計算 ∂^2E/∂xi_k*∂xi_l
            }
            // E~>Eならc<-10cとしてステップ3に戻る
        } while ( eTilde > e );

        //手順11.1 6. 未知数を更新し，｜E~-E｜<δであれば終了する．
        //            そうでなければ，E<-E~，c<-c/10としてステップ2へ戻る
        renewParams();  //未知数の更新
        calcpqr();      //pqrの更新
        calcError();    //更新

        //途中経過の表示
        {
            eTilde = this->f0 * sqrt(this->errorTilde / (2*this->In - (3*this->pointN + 9*this->cameraN - 7)));
            cout << "e     = " << e << "\n";
            cout << "e~    = " << eTilde << "\n";


            loop++;
            vector<Matrix3d> R; //3次元点
            vector<Vector3d> K; //3次元点
            vector<Vector3d> t; //3次元点
            for (int i = 0; i < this->cameraN; i++) {
                R.push_back(this->R[i]);
                K.push_back(Vector3d(this->f[i], this->u[i], this->v[i]));
                t.push_back(this->t[i]);
            }
            ostringstream stream;
            stream << loop;
            string result = stream.str();
            string filename = "bundle_out/loop_" + result + ".ply";
            cout << filename << endl;
            outDataPly(filename, this->X, this->pointN);
            string out_file;
            out_file = "bundle_out/loop_" + result + "_R";
            out_Mat3d(R, out_file);
            out_file = "bundle_out/loop_" + result + "_K";
            out_Vec3d(K, out_file);
            out_file = "bundle_out/loop_" + result + "_t";
            out_Vec3d(t, out_file);
        }

        // ｜E~-E｜>δ，E<-E~，c<-c/10としてステップ2へ戻る
        if ( abs(eTilde - e) <= delta ) {
            break;
        } else {
            e = eTilde;
            this->c = this->c / 10.0;
            cout << "c     = " << this->c << "\n";
        }
    }


}



//式11.12 ｐｑｒの計算
void BundleAdjustment::calcpqr()
{
    Matrix34d P;
    Vector3d pqr;
    Vector4d XYZ1;
    int alpha, kappa;
    int N = this->pointN;
    int M = this->cameraN;

    for(kappa = 0; kappa < M; kappa++){
        P << this->cameraMat[kappa];
        for(alpha = 0; alpha < N; alpha++){
            XYZ1 << this->X[alpha], 1.0;
            pqr = P * XYZ1;
            this->p[kappa][alpha] = pqr(0);
            this->q[kappa][alpha] = pqr(1);
            this->r[kappa][alpha] = pqr(2);
        }
    }
}

//式１１．１３ Eの計算
void BundleAdjustment::calcError()
{

    int N = this->pointN;
    int M = this->cameraN;
    double f0 = this->f0;
    double e = 0;
    double sqrt1, sqrt2;
    int alpha, kappa;

    for(kappa = 0; kappa < M; kappa++){
        for(alpha = 0; alpha < N; alpha++){
            if(this->I[kappa][alpha] == 1){
				sqrt1 = this->p[kappa][alpha] / this->r[kappa][alpha] - this->x[kappa][alpha] / f0;
				sqrt2 = this->q[kappa][alpha] / this->r[kappa][alpha] - this->y[kappa][alpha] / f0;
				e += pow(sqrt1, 2) + pow(sqrt2, 2);
			}
        }
    }
    this->error = e;
}

//式１１．１６ １１．１７ 11.18 11.20 パラメータの微分
Vector3d BundleAdjustment::calcdpqrak(int alpha, int kappa, int l)
{
    int N3 = this->pointN * 3;
    int M9 = this->cameraN * 9;
    Vector3d dpqr = Vector3d::Zero();
    int beta, lambda;
    Matrix34d P;
    int id;
    Vector3d r_k1, r_k2, r_k3;
    Matrix3d R;

    if(l < N3){
    //3次元点に関する
        beta = l / 3;
        if(beta != alpha){
            return dpqr;
        }
        P = this->cameraMat[kappa];
        id = l % 3;
        dpqr(0) = P(0, id);
        dpqr(1) = P(1, id);
        dpqr(2) = P(2, id);

    }else{
    //カメラパラメータに関する
        lambda = (l - N3) / 9;
        if(lambda != kappa){
            return dpqr;
        }

        id = (l - N3) % 9;
        switch (id) {
            case 0:
                dpqr(0) = (this->p[kappa][alpha] - this->u[kappa] / this->f0 * this->r[kappa][alpha]) / this->f[kappa];
    			dpqr(1) = (this->q[kappa][alpha] - this->v[kappa] / this->f0 * this->r[kappa][alpha]) / this->f[kappa];
    			dpqr(2) = 0;
                break;
            case 1:
                dpqr(0) = this->r[kappa][alpha] / this->f0;
                dpqr(1) = 0;
                dpqr(2) = 0;
                break;
            case 2:
                dpqr(0) = 0;
                dpqr(1) = this->r[kappa][alpha] / this->f0;
                dpqr(2) = 0;
                break;
            case 3:
            case 4:
            case 5:
                id = id - 3;
                R = this->R[kappa];
                r_k1 << R(0, 0), R(1, 0), R(2, 0);
			    r_k2 << R(0, 1), R(1, 1), R(2, 1);
		        r_k3 << R(0, 2), R(1, 2), R(2, 2);
                dpqr(0) = -(this->f[kappa] * r_k1(id) + this->u[kappa] * r_k3(id));
			    dpqr(1) = -(this->f[kappa] * r_k2(id) + this->v[kappa] * r_k3(id));
			    dpqr(2) = -this->f0 * r_k3(id);
                break;
            case 6:
            case 7:
            case 8:
                id = id - 6;
                R = this->R[kappa];
                r_k1 << R(0, 0), R(1, 0), R(2, 0);
                r_k2 << R(0, 1), R(1, 1), R(2, 1);
                r_k3 << R(0, 2), R(1, 2), R(2, 2);
                Vector3d vec1, vec2;
                vec2 = this->X[alpha] - this->t[kappa];
                vec1 = this->f[kappa] * r_k1 + this->u[kappa] * r_k3;
                dpqr(0) = (vec1.cross(vec2))(id);
                vec1 = this->f[kappa] * r_k2 + this->v[kappa] * r_k3;
                dpqr(1) = (vec1.cross(vec2))(id);
                vec1 = this->f0 * r_k3;
                dpqr(2) = (vec1.cross(vec2))(id);
                break;
        }
    }
    return dpqr;
}

//式11.14 Eの一次微分 partial E / partial xi_k
void BundleAdjustment::calcdError()
{
    int N = this->pointN;
    int M = this->cameraN;
    int N3 = this->pointN * 3;
    int M9 = this->cameraN * 9;
    int alpha, kappa;
    Vector3d dpqr;
    Vector3d pqr;
    double eql1, eql2, eqr1, eqr2;
    double e;

    //点
    for(int l = 0; l < N3; l++){
        e = 0;
        alpha = l / 3;
        for(kappa = 0; kappa < M; kappa++){
            if(this->I[kappa][alpha]){
                dpqr = calcdpqrak(alpha, kappa, l);
                pqr << this->p[kappa][alpha], this->q[kappa][alpha], this->r[kappa][alpha];
                eql1 = pqr(0) / pqr(2) - this->x[kappa][alpha] / this->f0;
				eqr1 = pqr(1) / pqr(2) - this->y[kappa][alpha] / this->f0;
				eql2 = pqr(2) * dpqr(0) - pqr(0) * dpqr(2);
				eqr2 = pqr(2) * dpqr(1) - pqr(1) * dpqr(2);
				e += (eql1 * eql2 + eqr1 * eqr2) / pow(pqr(2), 2);
            }
        }
        this->derror[l] = 2 * e;
    }

    //カメラパラメータ
    int N3_M9 = N3 + M9;
    for(int l = N3; l < N3_M9; l++){
        e = 0;
        kappa = (l - N3) / 9;
        for(alpha = 0; alpha < N; alpha++){
            if(this->I[kappa][alpha]){
                dpqr = calcdpqrak(alpha, kappa, l);
                pqr << this->p[kappa][alpha], this->q[kappa][alpha], this->r[kappa][alpha];
                eql1 = pqr(0) / pqr(2) - this->x[kappa][alpha] / this->f0;
				eqr1 = pqr(1) / pqr(2) - this->y[kappa][alpha] / this->f0;
				eql2 = pqr(2) * dpqr(0) - pqr(0) * dpqr(2);
				eqr2 = pqr(2) * dpqr(1) - pqr(1) * dpqr(2);
				e += (eql1 * eql2 + eqr1 * eqr2) / pow(pqr(2), 2);
            }
        }
        this->derror[l] = 2 * e;
    }
}

//式11.15 Eの二次微分
void BundleAdjustment::calcddError()
{
    //点に関するヘッセ行列 (3*3) * N
    calchE();

    //点とカメラパラメータに関するヘッセ行列 (3* (9M-7)) * N
    calchF();

    //カメラパラメータに関するヘッセ行列 (9M-7) * (9M-7)
    calchG();
}

//点に関するヘッセ行列 (3*3) * N
void BundleAdjustment::calchE()
{
    int N = this->pointN;
    double dde = 0;
    int start, end;
    int i, j;
    double c = this->c;
    for(int alpha = 0; alpha < N; alpha++){
        start = alpha * 3;
        end = start + 3;
        for(int l = start; l < end; l++){
            for(int k = l; k < end; k++){
                //ヘッセ行列の要素を計算
                dde = calchEkl(l, k, alpha);

                i = l % 3;
                j = k % 3;
                //if(l == k){
                if(i == j){
                    this->hE[alpha](i, j) = dde * (1.0+c);
                }else{
                    this->hE[alpha](i, j) = dde;
                    this->hE[alpha](j, i) = dde;
                }
            }//k
        }//l
    }//alpha


}
//式11.15 Eの二次微分 partial^2 E / (partial xi_k partial xi_l) 点に関するヘッセ行列要素の計算
double BundleAdjustment::calchEkl(int k, int l, int alpha)
{
    int M = this->cameraN;
    double e = 0;
	Vector3d dpqr_k;
    Vector3d dpqr_l;
    Vector3d pqr;
	double eqr1, eql1, eqr2, eql2;

    for(int kappa = 0; kappa < M; kappa++){
        if(this->I[kappa][alpha] == 1){		//K画像にa点が含まれているとき加算
			 dpqr_k = calcdpqrak(alpha, kappa, k);
			 dpqr_l = calcdpqrak(alpha, kappa, l);
             pqr << this->p[kappa][alpha], this->q[kappa][alpha], this->r[kappa][alpha];
             eql1 = pqr(2) * dpqr_k(0) - pqr(0) * dpqr_k(2);
			 eqr1 = pqr(2) * dpqr_k(1) - pqr(1) * dpqr_k(2);
			 eql2 = pqr(2) * dpqr_l(0) - pqr(0) * dpqr_l(2);
			 eqr2 = pqr(2) * dpqr_l(1) - pqr(1) * dpqr_l(2);
			 e += (eql1 * eql2 + eqr1 * eqr2) / pow(pqr(2), 4);
		 }
    }
    //cout <<  "e - " <<  e << endl;
    //cin >> l;
    return e * 2.0;
}

//点とカメラパラメータに関するヘッセ行列 (3* (9M-7)) * N
void BundleAdjustment::calchF()
{
    int N = this->pointN;
    int N3 = N * 3;
    int M9 = this->cameraN * 9;
    int N3_M9 = N3 + M9;
    double dde = 0;
    int start, end;
    int i = 0, j = 0;
    int alpha, kappa;
    int off;

    for(alpha = 0; alpha < N; alpha++){
        this->hF[alpha] = MatrixXd::Zero(3, M9-7);
        start = alpha * 3;
        end = start + 3;
        for(int l = start; l < end; l++){//テンに関する
            i = l % 3;
            j = 0;
            for(int k = N3; k < N3_M9; k++){//画像に関する
                off = offset(k);
                if(off == 0){
                    dde = calchFkl(k, l);
                    this->hF[alpha](i, j) = dde;
                    j++;
                }
            }
        }
    }//alpha
}
//式11.15 Eの二次微分 partial^2 E / (partial xi_k partial xi_l) 点とカメラパラメータに関するヘッセ行列要素の計算
double BundleAdjustment::calchFkl(int k, int l)
{
    int N = this->pointN;
    int N3 = N * 3;
    int M = this->cameraN;
    double e = 0;
    Vector3d dpqr_k;
    Vector3d dpqr_l;
    Vector3d pqr;
    double eqr1, eql1, eqr2, eql2;
    int alpha, kappa;
    kappa = (k - N3) / 9;
    alpha = l / 3;

    //cout << " k = " << k << " l = " << l << endl;
    if(this->I[kappa][alpha] == 1){		//K画像にa点が含まれているとき加算
         dpqr_k = calcdpqrak(alpha, kappa, k);
         dpqr_l = calcdpqrak(alpha, kappa, l);
         pqr << this->p[kappa][alpha], this->q[kappa][alpha], this->r[kappa][alpha];
         eql1 = pqr(2)  * dpqr_k(0) - pqr(0) * dpqr_k(2);
         eqr1 = pqr(2)  * dpqr_k(1) - pqr(1) * dpqr_k(2);
         eql2 = pqr(2)  * dpqr_l(0) - pqr(0) * dpqr_l(2);
         eqr2 = pqr(2)  * dpqr_l(1) - pqr(1) * dpqr_l(2);
         e += (eql1 * eql2 + eqr1 * eqr2) / pow(pqr(2), 4);
    }
    return e * 2;
}


//カメラパラメータに関するヘッセ行列 (9M-7) * (9M-7)
void BundleAdjustment::calchG()
{
    int N = this->pointN;
    int N3 = N * 3;
    int M = this->cameraN;
    int M9 = M * 9;
    int N3_M9 = N3 + M9;
    double dde = 0;
    int start, end;
    int i = 0, j = 0;
    int l2, k2;
    int alpha, kappa;
    int off;
    double c = this->c;
    for(kappa = 0; kappa < M; kappa++){
        start = kappa * 9 + N3;
        end = start + 9;
        //i = 0;
        for(int k = start; k < end; k++){
            off = offset(k);
            if(off == 0){
                j = i;
                for(int l = k; l < end; l++){

                    off = offset(l);
                    if(off == 0){
                    //cout << "i = " << i << " j = " << j << endl;
                        dde = calchGkl(k, l);
                        //cout << "a" << endl;
                        if(i == j){
                            this->hG(i, j) = dde * (1.0 + c);
                        }else{
                            this->hG(i, j) = dde;
                            this->hG(j, i) = dde;
                        }
                        j++;
                    }//off
                }
                i++;
            }//off
        }
    }
}
//式11.15 Eの二次微分 partial^2 E / (partial xi_k partial xi_l) カメラパラメータに関するヘッセ行列要素の計算
double BundleAdjustment::calchGkl(int k, int l)
{
    int N = this->pointN;
    int N3 = N * 3;
    double e = 0;
	Vector3d dpqr_k;
    Vector3d dpqr_l;
    Vector3d pqr;
	double eqr1, eql1, eqr2, eql2;
    int kappa;
    kappa = (k - N3) / 9;

    for(int alpha = 0; alpha < N; alpha++){
        if(this->I[kappa][alpha] == 1){		//K画像にa点が含まれているとき加算
			 dpqr_k = calcdpqrak(alpha, kappa, k);
			 dpqr_l = calcdpqrak(alpha, kappa, l);
             pqr << this->p[kappa][alpha], this->q[kappa][alpha], this->r[kappa][alpha];
			 eql1 = pqr(2) * dpqr_k(0) - pqr(0) * dpqr_k(2);
			 eqr1 = pqr(2) * dpqr_k(1) - pqr(1) * dpqr_k(2);
			 eql2 = pqr(2) * dpqr_l(0) - pqr(0) * dpqr_l(2);
			 eqr2 = pqr(2) * dpqr_l(1) - pqr(1) * dpqr_l(2);
			 e += (eql1 * eql2 + eqr1 * eqr2) / pow(pqr(2), 4);
		 }
    }
    return e * 2;
}

//カメラパラメータに関する微分 未知数調整用
int BundleAdjustment::offset(int k)
{
    int N3 = this->pointN * 3;
    k = k - N3;
    switch (k) {
    case 3:     //t1
    case 4:     //t1
    case 5:     //t1
    case 6:     //R1
    case 7:     //R1
    case 8:     //R1
    case 13:    //t22
        return 1;
        break;
    default:
        break;
    }
    return 0;
}

//E〜を計算する
void BundleAdjustment::calcErrorTilde()
{
    int N = this->pointN;
    int M = this->cameraN;
    double f0 = this->f0;
    double e = 0;
    double sqrt1, sqrt2;
    Vector3d pqrTilde;

    for(int kappa = 0; kappa < M; kappa++){
        for(int alpha = 0; alpha < N; alpha++){
            if(this->I[kappa][alpha] == 1){
                pqrTilde = calcpqrTildeak(alpha, kappa);
                sqrt1 = pqrTilde(0) / pqrTilde(2) - this->x[kappa][alpha] / f0;
				sqrt2 = pqrTilde(1) / pqrTilde(2) - this->y[kappa][alpha] / f0;
				e += (sqrt1*sqrt1) + (sqrt2*sqrt2);
			}
        }
    }
    this->errorTilde = e;
}

//E〜を計算するためにpqr~を計算する
Vector3d BundleAdjustment::calcpqrTildeak(int alpha, int kappa)
{
    double f0 = this->f0;
    Vector4d XTilde;
    double fTilde;
    double uTilde, vTilde;
    Vector3d tTilde;
    Matrix3d RTilde;

    //点
    XTilde(0) = this->X[alpha](0) + this->deltaXiP(alpha * 3 + 0);
	XTilde(1) = this->X[alpha](1) + this->deltaXiP(alpha * 3 + 1);
	XTilde(2) = this->X[alpha](2) + this->deltaXiP(alpha * 3 + 2);
	XTilde(3) = 1.0;

    //カメラパラメータ
    fTilde = this->f[kappa] + getdeltaXiF(kappa, 0);
    uTilde = this->u[kappa]; // + getdeltaXiF(kappa, 1);
    vTilde = this->v[kappa]; // + getdeltaXiF(kappa, 2);

    Vector3d deltat;
    deltat << getdeltaXiF(kappa, 3), getdeltaXiF(kappa, 4), getdeltaXiF(kappa, 5);
    tTilde = this->t[kappa] + deltat;

    Vector3d deltaOmega;
    deltaOmega << getdeltaXiF(kappa, 6), getdeltaXiF(kappa, 7), getdeltaXiF(kappa, 8);
    Matrix3d deltaR;
    deltaR = getRotateMat(deltaOmega);
    RTilde = deltaR * this->R[kappa];

    //カメラ行列の計算
    Matrix3d KTilde;
    KTilde <<
        fTilde, 0, uTilde,
        0, fTilde, vTilde,
        0, 0, f0;
    Matrix34d ItTilde;
    Matrix34d PTilde;
	ItTilde << Matrix3d::Identity(), -tTilde;
	PTilde = KTilde * RTilde.transpose() * ItTilde;

    Vector3d pqrTilde;
	pqrTilde = PTilde * XTilde;

    return pqrTilde;
}

//pqr~を計算用のカメラパラメータ
double BundleAdjustment::getdeltaXiF(int kappa, int param)
{
    double delta = 0;
    switch (kappa) {
    case 0:
        switch (param) {
		case 0:
		case 1:
		case 2:
			delta = this->deltaXiF(kappa * 9 + param);
			break;
		default:
            delta = 0;
			break;
		}
        break;
    case 1:
        switch (param) {
        case 0:
        case 1:
        case 2:
        case 3:
            delta = this->deltaXiF(kappa * 9 + param - 6);
            break;
        case 4:
            delta = 0;
            break;
        default:
            delta = this->deltaXiF(kappa * 9 - 7 + param);
            break;
        }
        break;
    default:
        delta = this->deltaXiF(kappa * 9 - 7 + param);
        break;
    }
    return delta;
}

//pqr~を計算用のカメラパラメータ
Matrix3d BundleAdjustment::getRotateMat(Vector3d l)
{
    Matrix3d R;
    double l1 = l(0);
    double l2 = l(1);
    double l3 = l(2);

	double theta = l.norm();

    /*if(theta > 0.1e-10){
        l1 /= theta;
        l2 /= theta;
        l3 /= theta;
    }*/
	theta = sqrt(theta);

	R <<
		cos(theta)+l1*l1*(1-cos(theta)), l1*l2*(1-cos(theta))-l3*sin(theta), l1*l3*(1-cos(theta))+l2*sin(theta),
		l2*l1*(1-cos(theta))+l3*sin(theta), cos(theta)+l2*l2*(1-cos(theta)), l2*l3*(1-cos(theta))-l1*sin(theta),
		l3*l1*(1-cos(theta))-l2*sin(theta), l3*l2*(1-cos(theta))+l1*sin(theta), cos(theta)+l3*l3*(1-cos(theta));
	return R;
}

//式11.10 未知数の更新
void BundleAdjustment::renewParams()
{
    int N = this->pointN;
    int M = this->cameraN;

    //点
    int id = 0;
    for(int i = 0; i < N; i++){
        id = i * 3;
        this->X[i](0) += this->deltaXiP(id);
		this->X[i](1) += this->deltaXiP(id + 1);
		this->X[i](2) += this->deltaXiP(id + 2);
    }

    //カメラパラメータ
    Vector3d deltat;
    Vector3d deltaOmega;
    Matrix3d deltaR;
    Matrix3d K;
    Matrix34d It;
    for(int kappa = 0; kappa < M; kappa ++){
        this->f[kappa] += getdeltaXiF(kappa, 0);

        deltat << getdeltaXiF(kappa, 3), getdeltaXiF(kappa, 4), getdeltaXiF(kappa, 5);
        this->t[kappa] = this->t[kappa] + deltat;

        deltaOmega << getdeltaXiF(kappa, 6), getdeltaXiF(kappa, 7), getdeltaXiF(kappa, 8);
        deltaR = getRotateMat(deltaOmega);
        this->R[kappa] = deltaR * this->R[kappa];
        K <<
            this->f[kappa], 0, this->u[kappa],
            0, this->f[kappa], this->v[kappa],
            0, 0, this->f0;
            It << Matrix3d::Identity(), - this->t[kappa];
            this->cameraMat[kappa] = K * this->R[kappa].transpose() * It;
    }

}



/*
    手順11.2 連立一次方程式
*/
//手順11.2 連立一次方程式
void BundleAdjustment::solveEquations()
{
    //delta xi_P と delta xi_F をセットする
    setdPdF();

    // 1 ∇X_αEを置く
    // 2 ９M-7次 連立１次方程式をxi_Fについて解く
    solveEquationsdXiF();

    // 3 第点に関するΔX_α、ΔY_α、ΔZ_αを計算する
    solveEquationsdXiP();
}

//delta xi_P、delta xi_F をセットする (3*N)(9*M-7)
void BundleAdjustment::setdPdF()
{
    int N3 = this->pointN * 3;
    int M9 = this->cameraN * 9;
    int N3_M9 = N3 + M9;

    for(int l = 0; l < N3; l++){
        this->dP(l) = this->derror[l];
    }
    int off;
    int fl = 0;
    for(int l = N3; l < N3_M9; l++){
        off = offset(l);
        if(off == 0){
            this->dF(fl) = this->derror[l];
            fl++;
        }
    }
}

//手順11.2 2 ９M-7次 連立１次方程式をxi_Fについて解く
//式11.24 delta xi_F
void BundleAdjustment::solveEquationsdXiF()
{
    int N = this->pointN;
	int M9_7 = this->cameraN * 9 - 7;

	Matrix3d invEa;
	MatrixXd traFainvEa(M9_7, 3);
	MatrixXd traFainvEaFa(M9_7, M9_7);
	Vector3d nabXaE;
	VectorXd traFainvEanabXaE(M9_7);

	traFainvEaFa = MatrixXd::Zero(M9_7, M9_7);
	traFainvEanabXaE = VectorXd::Zero(M9_7);
	for(int i = 0; i < N; i++){
		invEa = this->hE[i].inverse();
		traFainvEa = this->hF[i].transpose() * invEa;

        //nabla Xa E = partial E / partial xi
		nabXaE <<
			this->derror[i * 3],
			this->derror[i * 3 + 1],
			this->derror[i * 3 + 2];
		traFainvEaFa += traFainvEa * this->hF[i];
		traFainvEanabXaE += traFainvEa * nabXaE;
	}
	MatrixXd matl(M9_7, M9_7);
	VectorXd matr(M9_7);

    //xi_Fについて解く
	matl = this->hG - traFainvEaFa;
	matr = traFainvEanabXaE - this->dF;
	FullPivLU<MatrixXd> solver(matl);
	this->deltaXiF = solver.solve(matr);
}

//手順11.2 3 第点に関するΔX_α、ΔY_α、ΔZ_αを計算する
//式11.25 delta xi_F
void BundleAdjustment::solveEquationsdXiP()
{
    int N = this->pointN;
    Matrix3d invEa;
	Vector3d FadXiF;
	Vector3d nabXaE;
	Vector3d dXa;

	for(int i = 0; i < N; i++){
		invEa = this->hE[i].inverse();
		FadXiF = this->hF[i] * this->deltaXiF;
		nabXaE <<
			this->derror[i * 3],
			this->derror[i * 3 + 1],
			this->derror[i * 3 + 2];

		dXa = - invEa * (FadXiF + nabXaE);
		this->deltaXiP(i * 3 + 0) = dXa(0);
		this->deltaXiP(i * 3 + 1) = dXa(1);
		this->deltaXiP(i * 3 + 2) = dXa(2);
	}
}


//配列確保
void BundleAdjustment::init()
{
    int N = this->pointN;
    int M = this->cameraN;
    int i, j;

    /*入力データ*/
    this->I = new int*[M];
    this->x = new double*[M];
    this->y = new double*[M];
    for(i = 0; i < M; i++){
		this->x[i] = new double[N];
		this->y[i] = new double[N];
		this->I[i] = new int[N];
	}

    /*変数*/
    this->X = new Vector3d[this->pointN];
	for(i = 0; i < N; i++){
		this->X[i] = Vector3d::Zero();
	}
    this->cameraMat = new Matrix34d[M];
    this->f = new double[M];
    this->u = new double[M];
    this->v = new double[M];
    this->t = new Vector3d[M];
    this->R = new Matrix3d[M];

    cout << "a" << endl;
    /*誤差，連立方程式計算*/
    this->p = new double*[M];
    this->q = new double*[M];
    this->r = new double*[M];
    for(i = 0; i < M; i++){
		this->p[i] = new double[N];
		this->q[i] = new double[N];
		this->r[i] = new double[N];
	}

    int N3 = N * 3;
    int M9 = M * 9;
    int M9_7 = M9 - 7;
    int N3_M9 = N3 + M9;

    this->derror = new double[N3_M9];
    this->hE = new Matrix3d[N];
    this->hF = new MatrixXd[N];
    for(i = 0; i < N; i++){
		this->hF[i] = MatrixXd(3, M9_7);
	}
    this->hG = MatrixXd(M9_7, M9_7);
    this->deltaXiP = VectorXd(N3);
    this->deltaXiF = VectorXd(M9_7);
    this->dP = VectorXd(N3);
    this->dF = VectorXd(M9_7);
}

BundleAdjustment::BundleAdjustment()
{

}
//配列開放
BundleAdjustment::~BundleAdjustment()
{
    int N = this->pointN;
    int M = this->cameraN;


    if(this->p != NULL){
		cout << "delete" << endl;
		for(int i = 0; i < M; i++){
            delete this->x[i];
            delete this->y[i];
            delete this->I[i];
			delete this->p[i];
            delete this->q[i];
            delete this->r[i];
		}
        delete[] this->x;
        delete[] this->y;
        delete[] this->I;
		delete[] this->p;
        delete[] this->q;
        delete[] this->r;
	}
    delete[] this->X;
    delete[] this->f;
    delete[] this->u;
    delete[] this->v;
    delete[] this->t;
    delete[] this->R;
    delete[] this->cameraMat;
    delete[] this->derror;
    delete[] this->hE;
    delete[] this->hF;


}


/*
    パラメータセット
*/
void BundleAdjustment::setX(int alpha, Vector3d X)
{
    this->X[alpha] = X;
}
void BundleAdjustment::setf(int kappa, double f)
{
    this->f[kappa] = f;
}
void BundleAdjustment::setu(int kappa, double u)
{
    this->u[kappa] = u;
}
void BundleAdjustment::setv(int kappa, double v)
{
    this->v[kappa] = v;
}
void BundleAdjustment::sett(int kappa, Vector3d t)
{
    this->t[kappa] = t;
}
void BundleAdjustment::setR(int kappa, Matrix3d R)
{
    this->R[kappa] = R;
}
void BundleAdjustment::setCameraMat(int kappa, Matrix34d P)
{
    this->cameraMat[kappa] = P;
}
