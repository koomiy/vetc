#pragma once

#include <ros/ros.h>
#include <custom_msgs/mpc_to_nlp.h>
#include "cppad/cppad.hpp"
#include "cppad/ipopt/solve.hpp"
#include "cppad/example/cppad_eigen.hpp"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <cmath>

using namespace std;
using namespace Eigen;
using CppAD::AD;
using CppAD::NearEqual;

namespace vetc{

#define PREDICTION_STEPS_NUM 5

class FG_eval
{
public: 
    typedef CPPAD_TESTVECTOR(AD<double>) Evector;   // Evectorを、Eigen::Matrix<AD<double>, Eigen::Dynamic, 1>(要素がAD<double>型の可変長eigenベクトル)として型定義

    void operator()(Evector& fg, const Evector& x); // ()演算子のオーバーロード

private: 
    int N = PREDICTION_STEPS_NUM;

    // こいつらは一回計算するだけでいいから、#pragma onceがついてるここで計算したほうがいい。それか、デフォルトコンストラクタでやるかどっちか。
    Vector2d q;
    Vector2d h;
    double   r;
    Matrix2d Q;
    Matrix2d H;
    double   R;

    Matrix<double, 2*N, 2*N> Qhat;
    Matrix<double, N, N>     Rhat;

    Matrix2d             A;
    Matrix<double, 2, 1> B;

    Matrix<double, 2*N, 2> Ahat;
    Matrix<double, 2*N, N> Bhat;

    Matrix<double, 3*N, 3*N> F;
    Matrix<double, 2*N, 3*N> G;

    // 状態コストQ, 終端コストH, 入力コストR
    q = (1.0, 1.0);
    h = Vector2d(10.0, 10.0);
    R = 1.0;
    Q = q.asDiagonal();
    H = h.asDiagonal();

    // Qhat, Rhat
    Qhat = Zeros(2*N, 2*N); // 行列の部分を参照して、そこに入れたい行列を代入するってな操作
    for (int i = 0; i < N; i++){
        if(i == N-1) Qhat.block(2*i,2*i, 2,2) = H;
        else Qhat.block(2*i,2*i, 2,2) = Q;
    }
    Matrix<double, N, 1> rhat;
    for (int i = 0; i < N; i++){
        rhat[i] = R;
    }
    Rhat = rhat.asDiagonal();

    // 状態方程式行列A, B
    A << 0.0            , 1.0                 , 
         -(1/J)*(Ng*Ksp), -(1/J)*(D + Ke*Kt/R);
    B << 0.0, Kt/(R*J);

    // Ahat, Bhat
    Ahat = Zeros(2*N, 2);
    for (int i = 0; i < N; i++){
        if (i == 0) {
            Ahat.block(2*i,0, 2,1) = A;
        } else {
            Ahat.block(2*i,0, 2,1) = Ahat.block(2*(i-1),0, 2,1)*A;
        }
    }
    Bhat = Zeros(2*N, N);
    for (int i = 0; i < N: i++){    // 要検算
        if(i == 0){
            for (int j = 0; j < N-i; j++){
                Bhat.block(2*i,0, 2*(N-i),(N-i)).block(2*j,j, 2,1) = B;
            }
        } else {
            for (int j = 0; j < N-i; j++){
                Bhat.block(2*i,0, 2*(N-i),(N-i)).block(2*j,j, 2,1) = A*Bhat.block(2*(i-1),0, 2, 1);
            }
        }
    }

    // 目的関数行列F、制約関数行列G
    F = Zeros(3*N, 3*N);
    F.block(0,0, 2*N,2*N) = Qhat;
    F.block(2*N,2*N, N,N) = Rhat;
    G = Zeros(2*N, 3*N);
    Matrix<double, 2*N, 2*N> I = Identity(2*N, 2*N);
    G.block(0,0, 2*N,2*N) = I;
    G.block(0,2*N, 2*N,N) = -Bhat;

};

class solveNLP
{
public: 
    typedef CPPAD_TESTVECTOR(AD<double>) Evector;

    solveNLP();
    void spin();

private: 
    // mpcから現在状態を受け取るCallback関数
    void mpcCallback(const custom_msgs::mpc_to_nlp& sub_mpc_curstate);

    // ハンドラと、パブリッシャ・サブスクライバの定義
    ros::NodeHandle nh;
    ros::Subscriber sub_curstate;
    ros::Publisher  pub_input;

    // ========== 環境変数 ========== //
    bool   success;     // 正常動作判定
    int    N;           // 予測ステップ数
    double dt;          // 時刻幅
    // ============================== //

    // ========== 独立変数 ========== //
    Evector xk; // 現在の状態
    Evector x;  // 予測状態と将来入力
    // ============================== //

    // ========== ソルバー用の変数 ========== //
    int nx;   // 独立変数の個数
    int ng;   // 制約条件の個数
    Evector xi; // xの初期値
    Evector xl; // xの下限
    Evector xu; // xの上限
    Evector gl; // g(x)の下限
    Evector gu; // g(x)の上限

    vector<double> sol; // 解の格納庫
    FG_eval fg_eval;    // 目的関数と制約条件を計算するオブジェクト
    CppAD::ipopt::solve_result<Evector> solution;   // 解を返す変数
    // ======================================== //

    // ========== NLPソルバーのオプション設定 ========== //
    std::string options;

    // 処理内容をプリントしない
    options += "Integer print_level 0\n";
    options += "String  Sb          yes\n";

    // 最大反復回数
    options += "Integer max_iter    10\n";

    // 一次近似における必要精度
    options += "Numeric tol         1e-6\n";

    // 導関数のテスト??
    options += "String  derivative_test             second-order\n";

    // 有限差分評価時における、ランダム摂動量の最大値
    options += "Numeric point_perturbation_radius   0.\n";
    // ================================================== //

};

}