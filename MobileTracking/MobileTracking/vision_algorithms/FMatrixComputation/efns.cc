////////////////////////////////////////////////////////////
// Calculation for the fundamental matrix using extended FNS method.
// using 'Eigen3' library
////////////////////////////////////////////////////////////

#include "FMatrixComputation.hpp"

using namespace Eigen;

// core of extended FNS method.
bool
FMatrixComputation::efns_core(
                              Vector9d &theta,
                              std::vector<Vector9d> Xi,
                              std::vector<Matrix9d> V0,
                              int      Num,
                              int      Max_Iter,
                              double   Conv_EPS
                              )
{
    Matrix9d     M, L, X, Y, Ptd, tM, tL;
    Vector9d     th, thdag, thp, thhat, v1, v2;
    double       tVt, tXi, rp, rm;
    int          count;
    
    // copy
    th = theta;
    
    count = 0;
    do
    {
        
        // Matrix L and M
        M = L = ZeroMat9;
        for (int al = 0; al < Num; al++)
        {
            tVt = th.dot(V0[al] * th);
            tXi = th.dot(Xi[al]);
            
            M += (Xi[al] * Xi[al].transpose()) / tVt;
            L += sqr(tXi/tVt) * V0[al];
        }
        M /= (double)Num;
        L /= (double)Num;
        
        // set theda_\dag
        thdag = theta_dag(th);
        thdag /= thdag.norm();
        
        Ptd = I9 - thdag * thdag.transpose();
        
        X = M - L;
        Y = Ptd * X * Ptd;
        
        // Solve eigenvectors and eigenvalues
        SelfAdjointEigenSolver<Matrix9d> es(Y);
        
        // Two vectors corresponding to the smallest two eigenvalues
        v1 = es.eigenvectors().col(0);
        v2 = es.eigenvectors().col(1);
        
        thhat = th.dot(v1) * v1 + th.dot(v2) * v2;
        thp = Ptd * thhat;
        thp /= thp.norm();
        
        rp = (thp + th).norm();
        rm = (thp - th).norm();
        
        // check convergence
        if (rp < Conv_EPS || rm < Conv_EPS)
            break;
        
        // update
        th = th + thp;
        th /= th.norm();
    }
    while (++count < Max_Iter);
    
    if (count >= Max_Iter)
        return false;
    
    return true;
}

// extended FNS method
bool
FMatrixComputation::efns(
                         std::vector<Vector2d> pos0,
                         std::vector<Vector2d> pos1,
                         int      Num,
                         Vector9d &theta,
                         int      Max_Iter,
                         double   Conv_EPS,
                         double   f0
                         )
{
    std::vector<Vector9d>     Xi(Num);
    std::vector<Matrix9d>     V0(Num);
    Vector9d     th, newth;
    
    // compute Xi and V0 before iteration
    for (int al = 0; al < Num; al++)
    {
        Xi[al] = convert_vec9(pos0[al], pos1[al], f0);
        V0[al] = make_cov_mat(pos0[al], pos1[al], f0);
    }
    
    // initialize theta by Taubin
    taubin_core(Xi, V0, Num, th);
    
    // call FNS core
    if (!efns_core(th, Xi, V0, Num, Max_Iter, Conv_EPS))
        return false;     // fail
    
    // final F
    theta = th;
    
    return true;
}
