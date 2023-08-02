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