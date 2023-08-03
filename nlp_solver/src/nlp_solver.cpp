#include "../include/nlp_solver.hpp"

using namespace std;
using namespace Eigen;
using CppAD::AD;
using CppAD::NearEqual;

namespace vetc{

void FG_eval::operator()(Evector& fg, const Evector& x){
    // サイズが異常のとき、強制終了
    assert(fg.size() == 1 + 2*N); // 目的関数で1、制約関数で2*N
    assert(x.size() == 2*N + 2*N);  // 予測状態軌跡で2*N、将来入力軌跡で2*N

    // 現在状態の読み込み
    Evector xk = solveNLP::xk;  // これで問題なく読み込めているか不安

    // 目的関数f(x)、制約関数g(x)
    fg[0] = (1/2)*x.transpose()*F*x + xk.transpose()*Q*xk;  // f(x)
    Evector g;  // この定義は良くない、サイズは決まってるからね。
    g = G*x - Ahat*xk;
    for (int i = 0; i < 2*N; i++){
        fg[i+1] = g[i]; // g(x)
    }

    return;
    
}

solveNLP::solveNLP() : 
    success(true), 
    N(PREDICTION_STEPS_NUM), 
    dt(0.05), 
    nx(2*N + 2*N), 
    ng(2*N)
{
    // サブスクライバのハンドラ立ち上げ
    sub_curstate = nh.subscribe("/mpc_to_nlp", 10, &solveNLP::mpcCallback, this);

    // パブリッシャのハンドラ立ち上げ
    pub_input = nh.advertise<float64>("nlp_to_mpc", 10);    // メッセージ型はfloat64という書き方でいいのか

    // 独立変数の初期化
    xi.resize(nx);
    for (int i = 0; i < nx; i++){    xi[i] = 0.0;}

    // 独立変数の上限、下限を設定
    xl.resize(nx);
    xu.resize(nx);
    for (int i = 0; i < 2*N; i++){
        // モーター角度の制限
        xl[2*i] = 0.0;
        xu[2*i] = (1/Ng)*(M_PI/2);

        // モーター角速度の制限(めっちゃ適当)
        xl[2*i+1] = -10.0;
        xu[2*i+1] = 10.0;
    }
    for (int i = 2*N; i < nx; i++){
        // 入力電圧の制限
        xl[2*i] = 0.0;
        xu[2*i] = 12.0;

        // 仮想入力の等号制約
        xl[2*i+1] = 1.0;
        xu[2*i+1] = 1.0;
    }

    // 制約関数の上限と下限を設定
    gl.resize(ng);
    gu.resize(ng);
    for (i = 0; i < ng; i++){
        // 等号制約
        gl[i] = 0.0;
        gu[i] = 0.0;
    }

}

void solveNLP::mpcCallback(const custom_msgs::mpc_to_nlp& sub_mpc_curstate){
    xk = sub_mpc_curstate;

}

void solveNLP::spin(){
    ros::Rate loop_rate(20);
    while (ros::ok() && success){
        // Callback関数の呼び出し
        ros::spinOnce();

        // 非線形問題を解く
        CppAD::ipopt::solve<Evector, FG_eval>(
            options, xi, xl, xu, gl, gu, fg_eval, solution
        );

        // 解の一部の整合性評価
        success &= solution.status == CppAD::ipopt::solve_result<Evector>::success;

        // コストのプリントアウト
        auto cost = solution.obj_value;
        cout << "Cost: " << cost << endl;

        // モーター角度・角速度、および入力電圧の最適解
        sol.push_back(solution.x[0]);   // θm
        sol.push_back(solution.x[1]);   // dθm
        sol.push_back(solution.x[2*N]); // V

        // 入力電圧の送信
        pub_input.publish(sol[2]);

        loop_rate.sleep();
    }
}

}   // end vetc namespace
