#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <custom_msgs/mpc_bw_nlp.h>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <cppad/example/cppad_eigen.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <cmath>

using namespace std;
using namespace Eigen;
using CppAD::AD;

// #define PREDICTION_STEPS_NUM 5
#define N 5

class FG_eval
{
public: 
    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;   // ADvectorを、Eigen::Matrix<AD<double>, Eigen::Dynamic, 1>(要素がAD<double>型の可変長eigenベクトル)として型定義
    typedef CPPAD_TESTVECTOR(double) Dvector;

    Vector2d xk;
    FG_eval(Dvector xk) {
        this->xk[0] = xk[0];
        this->xk[1] = xk[1];
    }
    void operator()(ADvector& fg, const ADvector& x); // ()演算子のオーバーロード

private: 
    // 予測ステップ数
    //int N = PREDICTION_STEPS_NUM;
    //static int N = 5;

    // パラメータ
    double J = 1.0;
    double D = 1.0;
    double Kt = 1.0;
    double Ksp = 1.0;
    double Re = 1.0;
    double Ke = 1.0;
    double Ng = 1.0;

    // こやつらは別途setup_mpcパッケージかなんか用意して、F,Gだけ送るようにした方がいいかもな
    // setup_mpcでF,Gの計算は一回しか行わないけど、常にメッセージだけ送り続けるみたいな
    Vector2d q;
    Vector2d h;
    double   r;
    Matrix2d Q;
    Matrix2d H;
    double   R;

    Matrix<double, 2*N, 2*N> Qhat;
    Matrix<double, N, N>     Rhat;

    Matrix2d A;
    Vector2d B;

    Matrix<double, 2*N, 2> Ahat;
    Matrix<double, 2*N, N> Bhat;

    Matrix<double, 3*N, 3*N> F;
    Matrix<double, 2*N, 3*N> G;

    Matrix<double, 3*N, 1> si; // state and input

};

class solveNLP
{
public: 
    typedef CPPAD_TESTVECTOR(double) Dvector;

    solveNLP();
    void spin();

    Dvector xk; // 現在の状態

private: 
    // mpcから現在状態を受け取るCallback関数
    bool mpcService(custom_msgs::mpc_bw_nlp::Request& req_mpc_states,
                    custom_msgs::mpc_bw_nlp::Response& res_nlp_input);

    // ハンドラと、サービスサーバーの定義
    ros::NodeHandle nh;
    ros::ServiceServer server;

    // ========== 環境変数 ========== //
    bool   success;     // 正常動作判定
    //int    N;           // 予測ステップ数
    double dt;          // 時刻幅
    // ============================== //

    double Ng = 1.0;  // ギア比

    // ========== 独立変数 ========== //
    //Dvector xk; // 現在の状態
    Dvector x;  // 予測状態と将来入力
    // ============================== //

    // ========== ソルバー用の変数 ========== //
    int nx;   // 独立変数の個数
    int ng;   // 制約条件の個数
    Dvector xi; // xの初期値
    Dvector xl; // xの下限
    Dvector xu; // xの上限
    Dvector gl; // g(x)の下限
    Dvector gu; // g(x)の上限

    std::string options;    // オプション設定
    vector<double> sol; // 解の格納庫
    // FG_eval fg_eval;    // 目的関数と制約条件を計算するオブジェクト
    CppAD::ipopt::solve_result<Dvector> solution;   // 解を返す変数
    // ======================================== //

};

// とりあえずビルドは通るようになった(230807)
// 今後、より可読性の高い構造に変える